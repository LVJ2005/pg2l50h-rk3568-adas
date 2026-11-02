//////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2019 PANGO MICROSYSTEMS, INC
// ALL RIGHTS REVERVED.
//
// THE SOURCE CODE CONTAINED HEREIN IS PROPRIETARY TO PANGO MICROSYSTEMS, INC.
// IT SHALL NOT BE REPRODUCED OR DISCLOSED IN WHOLE OR IN PART OR USED BY
// PARTIES WITHOUT WRITTEN AUTHORIZATION FROM THE OWNER.
//
//////////////////////////////////////////////////////////////////////////////
//
// Library:
// Filename:ips2l_pcie_dma.v
//////////////////////////////////////////////////////////////////////////////
module ips2l_pcie_dma #(
    parameter                           DEVICE_TYPE = 3'd0      ,   //3'd0:EP,3'd1:Legacy EP,3'd4:RC
    parameter   integer                 AXIS_SLAVE_NUM = 3      ,
    parameter                           ADDR_WIDTH  = 4'd12

)(
    input                               clk                     ,            //gen1:62.5MHz,gen2:125MHz
    input                               rst_n                   ,

    input           [7:0]               i_cfg_pbus_num          ,
    input           [4:0]               i_cfg_pbus_dev_num      ,
    input           [2:0]               i_cfg_max_rd_req_size   ,
    input           [2:0]               i_cfg_max_payload_size  ,
    //**********************************************************************
    //axis master interface
    input                               i_axis_master_tvld      ,
    output  wire                        o_axis_master_trdy      ,
    input           [127:0]             i_axis_master_tdata     ,
    input           [3:0]               i_axis_master_tkeep     ,
    input                               i_axis_master_tlast     ,
    input           [7:0]               i_axis_master_tuser     ,
    output  wire    [2:0]               o_trgt1_radm_pkt_halt   ,
//    input           [5:0]               i_radm_grant_tlp_type   ,
    //**********************************************************************
    //axis_slave0 interface
    input                               i_axis_slave0_trdy      ,
    output  wire                        o_axis_slave0_tvld      ,
    output  wire    [127:0]             o_axis_slave0_tdata     ,
    output  wire                        o_axis_slave0_tlast     ,
    output  wire                        o_axis_slave0_tuser     ,
    //axis_slave1 interface
    input                               i_axis_slave1_trdy      ,
    output  wire                        o_axis_slave1_tvld      ,
    output  wire    [127:0]             o_axis_slave1_tdata     ,
    output  wire                        o_axis_slave1_tlast     ,
    output  wire                        o_axis_slave1_tuser     ,
    //axis_slave2 interface
    input                               i_axis_slave2_trdy      ,
    output  wire                        o_axis_slave2_tvld      ,
    output  wire    [127:0]             o_axis_slave2_tdata     ,
    output  wire                        o_axis_slave2_tlast     ,
    output  wire                        o_axis_slave2_tuser     ,
    //**********************************************************************
    //credit interface
    //from pcie
    input                               i_cfg_ido_req_en        ,
    input                               i_cfg_ido_cpl_en        ,
    input           [7:0]               i_xadm_ph_cdts          ,
    input           [11:0]              i_xadm_pd_cdts          ,
    input           [7:0]               i_xadm_nph_cdts         ,
    input           [11:0]              i_xadm_npd_cdts         ,
    input           [7:0]               i_xadm_cplh_cdts        ,
    input           [11:0]              i_xadm_cpld_cdts        ,
    //**********************************************************************
    //apb interface
    input                               i_apb_psel              ,
    input           [8:0]               i_apb_paddr             ,
    input           [31:0]              i_apb_pwdata            ,
    input           [3:0]               i_apb_pstrb             ,
    input                               i_apb_pwrite            ,
    input                               i_apb_penable           ,
    output  wire                        o_apb_prdy              ,
    output  wire    [31:0]              o_apb_prdata            ,
    //cross_4kb_boundary
    output  wire                        o_cross_4kb_boundary    ,

    //**********************************************************************
    // dma write user interface(FPGA-->PC)
    output                            o_dma_write_data_req        ,
    output        [ADDR_WIDTH-1:0]    o_dma_write_addr            ,
    input         [127:0]             i_dma_write_data            ,

    //**********************************************************************
    // dma read user interface(PC-->FPGA)
    input                             i_dma_read_data_req        ,
    input         [ADDR_WIDTH-1:0]    i_dma_read_addr            ,
    output        [127:0]             o_dma_read_data             ,

    // Full-frame DMA / MSI interface
    input                             i_msi_enabled               ,
    output reg                        o_msi_req                   ,
    input                             i_msi_grant                 ,
    output reg                        o_frame_start               ,
    input                             i_frame_stream_ready        ,
    output reg                        o_frame_source_req          ,
    input                             i_frame_source_ack          

    //debug
    //output  wire    [159:0]             o_dbg_bus
);

//**********************************************************************
//from rx to tx
wire        [2:0]               mrd_tc;
wire        [2:0]               mrd_attr;
wire        [9:0]               mrd_length;
wire        [15:0]              mrd_id;
wire        [7:0]               mrd_tag;
wire        [63:0]              mrd_addr;

wire                            cpld_req_vld;
wire                            cpld_req_rdy;
wire                            cpld_tx_rdy;
//cpld rcv
wire                            cpld_rcv;
wire        [7:0]               cpld_tag;
wire                            tag_full;
//from dma_controller to tx
wire                            user_define_header_flag;
wire                            user_define_data_flag;
//mwr req & ack
wire                            ctrl_mwr32_req;
wire                            ctrl_mwr32_req_ack;
wire                            ctrl_mwr64_req;
wire                            ctrl_mwr64_req_ack;
wire                            tx_mwr32_req;
wire                            tx_mwr32_req_ack;
wire                            tx_mwr64_req;
wire                            tx_mwr64_req_ack;
//mrd req & ack
wire                            mrd32_req;
wire                            mrd32_req_ack;
wire                            mrd64_req;
wire                            mrd64_req_ack;
//req information
wire        [9:0]               ctrl_req_length;
wire        [63:0]              ctrl_req_addr;
wire        [31:0]              ctrl_req_data;
wire        [9:0]               tx_req_length;
wire        [63:0]              tx_req_addr;
wire        [31:0]              tx_req_data;
wire                            tx_user_define_data_flag;
wire                            mwr_tx_busy;


//axis_slave0 interface
wire                            dma_axis_slave0_trdy ;
wire                            dma_axis_slave0_tvld ;
wire        [127:0]             dma_axis_slave0_tdata;
wire                            dma_axis_slave0_tlast;
wire                            dma_axis_slave0_tuser;
//axis_slave1 interface
wire                            dma_axis_slave1_trdy ;
wire                            dma_axis_slave1_tvld ;
wire        [127:0]             dma_axis_slave1_tdata;
wire                            dma_axis_slave1_tlast;
wire                            dma_axis_slave1_tuser;
//axis_slave2 interface
wire                            dma_axis_slave2_trdy ;
wire                            dma_axis_slave2_tvld ;
wire        [127:0]             dma_axis_slave2_tdata;
wire                            dma_axis_slave2_tlast;
wire                            dma_axis_slave2_tuser;

//**********************************************************************
//bar0 rd interface
wire                            bar0_rd_clk_en/*synthesis PAP_MARK_DEBUG="1"*/;
wire        [ADDR_WIDTH-1:0]    bar0_rd_addr;
wire        [127:0]             bar0_rd_data/*synthesis PAP_MARK_DEBUG="1"*/;
//bar1 wr interface
wire                            bar1_wr_en/*synthesis PAP_MARK_DEBUG="1"*/;
wire        [ADDR_WIDTH-1:0]    bar1_wr_addr/*synthesis PAP_MARK_DEBUG="1"*/;
wire        [127:0]             bar1_wr_data/*synthesis PAP_MARK_DEBUG="1"*/;
wire        [15:0]              bar1_wr_byte_en/*synthesis PAP_MARK_DEBUG="1"*/;
//bar2 rd interface
wire                            bar2_rd_clk_en;
wire        [ADDR_WIDTH-1:0]    bar2_rd_addr;
wire        [127:0]             bar2_rd_data;
//**********************************************************************
//rst tlp cnt
wire                            tx_restart;
wire        [63:0]              dma_check_result;
//debug bus
//wire        [42:0]              dbg_bus_rx_ctrl;
//wire        [43:0]              dbg_bus_mrd_tx;
//wire        [72:0]              dbg_bus_mwr_tx;

//assign o_dbg_bus = {
//                    dbg_bus_mwr_tx, //159:87
//                    dbg_bus_mrd_tx, //86:43
//                    dbg_bus_rx_ctrl //42:0
//                    };

ips2l_pcie_dma_rx_top #(
    .DEVICE_TYPE                (DEVICE_TYPE                ),  //3'd0:EP,3'd1:Legacy EP,3'd4:RC
    .ADDR_WIDTH                 (ADDR_WIDTH                 )
)
u_ips2l_pcie_dma_rx_top
(
    .clk                        (clk                        ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                      (rst_n                      ),
    .i_cfg_max_rd_req_size      (i_cfg_max_rd_req_size      ),  //input [2:0]
    //**********************************************************************
    //axis master interface
    .i_axis_master_tvld         (i_axis_master_tvld         ),
    .o_axis_master_trdy         (o_axis_master_trdy         ),
    .i_axis_master_tdata        (i_axis_master_tdata        ),
    .i_axis_master_tkeep        (i_axis_master_tkeep        ),
    .i_axis_master_tlast        (i_axis_master_tlast        ),
    .i_axis_master_tuser        (i_axis_master_tuser        ),
    .o_trgt1_radm_pkt_halt      (o_trgt1_radm_pkt_halt      ),
//    .i_radm_grant_tlp_type      (i_radm_grant_tlp_type      ),
    //**********************************************************************
    //bar0 rd interface
    .i_bar0_rd_clk_en           (bar0_rd_clk_en             ),
    .i_bar0_rd_addr             (bar0_rd_addr               ),
    .o_bar0_rd_data             (bar0_rd_data               ),
    //bar2 rd interface
    .i_bar2_rd_clk_en           (i_dma_read_data_req          ),
    .i_bar2_rd_addr             (i_dma_read_addr              ),
    .o_bar2_rd_data             (o_dma_read_data              ),
    //bar1 wr interface
    .o_bar1_wr_en               (bar1_wr_en                 ),
    .o_bar1_wr_addr             (bar1_wr_addr               ),
    .o_bar1_wr_data             (bar1_wr_data               ),
    .o_bar1_wr_byte_en          (bar1_wr_byte_en            ),
    //**********************************************************************
    //to tx top
    //req rcv
    .o_mrd_tc                   (mrd_tc                     ),
    .o_mrd_attr                 (mrd_attr                   ),
    .o_mrd_length               (mrd_length                 ),
    .o_mrd_id                   (mrd_id                     ),
    .o_mrd_tag                  (mrd_tag                    ),
    .o_mrd_addr                 (mrd_addr                   ),

    .o_cpld_req_vld             (cpld_req_vld               ),
    .i_cpld_req_rdy             (cpld_req_rdy               ),
    .i_cpld_tx_rdy              (cpld_tx_rdy                ),
    //cpld rcv
    .o_cpld_rcv                 (cpld_rcv                   ),
    .o_cpld_tag                 (cpld_tag                   ),
    .i_tag_full                 (tag_full                   ),
    //rst tlp cnt
    .o_dma_check_result         (dma_check_result           ),
    .i_tx_restart               (tx_restart                 )

    //.o_dbg_bus_rx_ctrl          (dbg_bus_rx_ctrl            )
);

// Full-frame DMA registers in BAR1. A frame is split into 2 KiB requests;
// the PCIe TX block further splits each request according to MPS.
localparam [ADDR_WIDTH-1:0] FRAME_BUF0_ADDR_REG = 12'h130;
localparam [ADDR_WIDTH-1:0] FRAME_BUF1_ADDR_REG = 12'h140;
localparam [ADDR_WIDTH-1:0] FRAME_CHUNK_REG     = 12'h150;
localparam [ADDR_WIDTH-1:0] FRAME_CTRL_REG      = 12'h160;
localparam [ADDR_WIDTH-1:0] FRAME_RELEASE_REG   = 12'h170;

localparam [3:0] FRAME_IDLE         = 4'd0;
localparam [3:0] FRAME_WAIT_STREAM  = 4'd1;
localparam [3:0] FRAME_ISSUE        = 4'd2;
localparam [3:0] FRAME_WAIT_BUSY    = 4'd3;
localparam [3:0] FRAME_WAIT_DONE    = 4'd4;
localparam [3:0] FRAME_HANDOFF      = 4'd5;
localparam [3:0] FRAME_WAIT_BUFFER  = 4'd6;
localparam [3:0] FRAME_WAIT_SOURCE  = 4'd7;

reg [63:0] frame_buf0_addr;
reg [63:0] frame_buf1_addr;
reg [15:0] frame_chunk_count;
reg [15:0] frame_chunk_index;
reg [3:0]  frame_state;
reg [1:0]  frame_host_owned;
reg        frame_buffer_select;
reg        frame_dma_enable;
reg        frame_mwr32_req;
reg        frame_mwr64_req;

wire frame_ctrl_write;
wire frame_start_write;
wire frame_stop_write;
wire frame_release_write;
wire [1:0] frame_release_mask;
wire frame_path_active;
wire frame_req_ack;
wire [63:0] frame_selected_addr;
wire [63:0] frame_req_addr;
wire frame_next_buffer_available;

assign frame_ctrl_write = bar1_wr_en && (bar1_wr_addr == FRAME_CTRL_REG);
assign frame_start_write = frame_ctrl_write && bar1_wr_data[0];
assign frame_stop_write = frame_ctrl_write && bar1_wr_data[1];
assign frame_release_write = bar1_wr_en && (bar1_wr_addr == FRAME_RELEASE_REG);
assign frame_release_mask = frame_release_write ? bar1_wr_data[1:0] : 2'b00;
assign frame_path_active = frame_dma_enable || (frame_state != FRAME_IDLE);
assign frame_selected_addr = frame_buffer_select ? frame_buf1_addr : frame_buf0_addr;
assign frame_req_addr = frame_selected_addr + {37'd0, frame_chunk_index, 11'd0};
assign frame_req_ack = frame_mwr64_req ? tx_mwr64_req_ack : tx_mwr32_req_ack;
assign frame_next_buffer_available =
    !frame_host_owned[~frame_buffer_select] ||
    frame_release_mask[~frame_buffer_select];

assign tx_mwr32_req = frame_path_active ? frame_mwr32_req : ctrl_mwr32_req;
assign tx_mwr64_req = frame_path_active ? frame_mwr64_req : ctrl_mwr64_req;
assign ctrl_mwr32_req_ack = frame_path_active ? 1'b0 : tx_mwr32_req_ack;
assign ctrl_mwr64_req_ack = frame_path_active ? 1'b0 : tx_mwr64_req_ack;
assign tx_req_length = frame_path_active ? 10'd512 : ctrl_req_length;
assign tx_req_addr = frame_path_active ? frame_req_addr : ctrl_req_addr;
assign tx_req_data = frame_path_active ? 32'd0 : ctrl_req_data;
assign tx_user_define_data_flag =
    frame_path_active ? 1'b0 : user_define_data_flag;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        frame_buf0_addr <= 64'd0;
        frame_buf1_addr <= 64'd0;
        frame_chunk_count <= 16'd1350;
        frame_chunk_index <= 16'd0;
        frame_state <= FRAME_IDLE;
        frame_host_owned <= 2'b00;
        frame_buffer_select <= 1'b0;
        frame_dma_enable <= 1'b0;
        frame_mwr32_req <= 1'b0;
        frame_mwr64_req <= 1'b0;
        o_msi_req <= 1'b0;
        o_frame_start <= 1'b0;
        o_frame_source_req <= 1'b0;
    end else begin
        o_frame_start <= 1'b0;

        if (bar1_wr_en && (bar1_wr_addr == FRAME_BUF0_ADDR_REG))
            frame_buf0_addr <= {32'd0, bar1_wr_data[31:0]};
        if (bar1_wr_en && (bar1_wr_addr == FRAME_BUF1_ADDR_REG))
            frame_buf1_addr <= {32'd0, bar1_wr_data[31:0]};
        if (bar1_wr_en && (bar1_wr_addr == FRAME_CHUNK_REG) &&
            (bar1_wr_data[15:0] != 16'd0))
            frame_chunk_count <= bar1_wr_data[15:0];

        if (frame_release_write)
            frame_host_owned <= frame_host_owned & ~frame_release_mask;

        if (frame_start_write)
            frame_dma_enable <= 1'b1;
        if (frame_stop_write) begin
            frame_dma_enable <= 1'b0;
            frame_host_owned <= 2'b00;
            if ((frame_state == FRAME_IDLE) ||
                (frame_state == FRAME_WAIT_STREAM) ||
                (frame_state == FRAME_WAIT_BUFFER)) begin
                frame_state <= FRAME_IDLE;
                frame_mwr32_req <= 1'b0;
                frame_mwr64_req <= 1'b0;
                o_msi_req <= 1'b0;
                o_frame_source_req <= 1'b0;
            end
        end

        case (frame_state)
            FRAME_IDLE: begin
                frame_mwr32_req <= 1'b0;
                frame_mwr64_req <= 1'b0;
                o_msi_req <= 1'b0;
                o_frame_source_req <= 1'b0;
                if ((frame_dma_enable || frame_start_write) &&
                    !frame_stop_write && i_msi_enabled &&
                    (frame_buf0_addr != 64'd0) &&
                    (frame_buf1_addr != 64'd0) &&
                    (frame_chunk_count != 16'd0)) begin
                    frame_buffer_select <= 1'b0;
                    frame_chunk_index <= 16'd0;
                    frame_host_owned <= 2'b00;
                    o_frame_source_req <= 1'b1;
                    frame_state <= FRAME_WAIT_SOURCE;
                end
            end

            // DDR read_req must stay high until ACK, including on frame zero.
            // Without this state the FIFO never fills and DMA cannot start.
            FRAME_WAIT_SOURCE: begin
                if (!frame_dma_enable || frame_stop_write) begin
                    o_frame_source_req <= 1'b0;
                    frame_state <= FRAME_IDLE;
                end else if (o_frame_source_req && i_frame_source_ack) begin
                    o_frame_source_req <= 1'b0;
                    // Reset the packer only after DDR has acknowledged and
                    // cleared its read FIFO, so no stale beat can be prefetched.
                    o_frame_start <= 1'b1;
                    frame_state <= FRAME_WAIT_STREAM;
                end
            end

            FRAME_WAIT_STREAM: begin
                if (i_frame_stream_ready) begin
                    frame_state <= FRAME_ISSUE;
                end
            end

            FRAME_ISSUE: begin
                if (frame_req_addr[63:32] != 32'd0)
                    frame_mwr64_req <= 1'b1;
                else
                    frame_mwr32_req <= 1'b1;

                if (frame_req_ack) begin
                    frame_mwr32_req <= 1'b0;
                    frame_mwr64_req <= 1'b0;
                    frame_state <= FRAME_WAIT_BUSY;
                end
            end

            FRAME_WAIT_BUSY: begin
                if (mwr_tx_busy)
                    frame_state <= FRAME_WAIT_DONE;
            end

            FRAME_WAIT_DONE: begin
                if (!mwr_tx_busy) begin
                    if (!frame_dma_enable || frame_stop_write) begin
                        frame_state <= FRAME_IDLE;
                    end else if (frame_chunk_index ==
                                 (frame_chunk_count - 16'd1)) begin
                        frame_host_owned[frame_buffer_select] <= 1'b1;
                        o_msi_req <= 1'b1;
                        frame_state <= FRAME_HANDOFF;
                    end else begin
                        frame_chunk_index <= frame_chunk_index + 16'd1;
                        frame_state <= FRAME_ISSUE;
                    end
                end
            end

            FRAME_HANDOFF: begin
                if (o_msi_req && i_msi_grant) begin
                    o_msi_req <= 1'b0;
                    if (!frame_dma_enable || frame_stop_write) begin
                        frame_state <= FRAME_IDLE;
                    end else begin
                        frame_buffer_select <= ~frame_buffer_select;
                        frame_chunk_index <= 16'd0;
                        if (frame_next_buffer_available) begin
                            o_frame_source_req <= 1'b1;
                            frame_state <= FRAME_WAIT_SOURCE;
                        end else begin
                            frame_state <= FRAME_WAIT_BUFFER;
                        end
                    end
                end
            end

            FRAME_WAIT_BUFFER: begin
                if (!frame_host_owned[frame_buffer_select] ||
                    frame_release_mask[frame_buffer_select]) begin
                    frame_chunk_index <= 16'd0;
                    o_frame_source_req <= 1'b1;
                    frame_state <= FRAME_WAIT_SOURCE;
                end
            end

            default: frame_state <= FRAME_IDLE;
        endcase
    end
end

ips2l_pcie_dma_controller #(
    .DEVICE_TYPE                (DEVICE_TYPE                ),  //3'd0:EP,3'd1:Legacy EP,3'd4:RC
    .ADDR_WIDTH                 (ADDR_WIDTH                 )
)
u_ips2l_pcie_dma_controller
(
    .clk                        (clk                        ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                      (rst_n                      ),
    //**********************************************************************
    //bar1 wr interface
    .i_bar1_wr_en               (bar1_wr_en                 ),
    .i_bar1_wr_addr             (bar1_wr_addr               ),
    .i_bar1_wr_data             (bar1_wr_data               ),
    .i_bar1_wr_byte_en          (bar1_wr_byte_en            ),
    //**********************************************************************
    //apb interface
    .i_apb_psel                 (i_apb_psel                 ),
    .i_apb_paddr                (i_apb_paddr                ),
    .i_apb_pwdata               (i_apb_pwdata               ),
    .i_apb_pstrb                (i_apb_pstrb                ),
    .i_apb_pwrite               (i_apb_pwrite               ),
    .i_apb_penable              (i_apb_penable              ),
    .o_apb_prdy                 (o_apb_prdy                 ),
    .o_apb_prdata               (o_apb_prdata               ),
    //**********************************************************************
    .o_user_define_data_flag    (user_define_data_flag      ),

    //**********************************************************************
    //to tx top
    //mwr req & ack
    .o_mwr32_req                (ctrl_mwr32_req             ),
    .i_mwr32_req_ack            (ctrl_mwr32_req_ack         ),
    .o_mwr64_req                (ctrl_mwr64_req             ),
    .i_mwr64_req_ack            (ctrl_mwr64_req_ack         ),
    //mrd req & ack
    .o_mrd32_req                (mrd32_req                  ),
    .i_mrd32_req_ack            (mrd32_req_ack              ),
    .o_mrd64_req                (mrd64_req                  ),
    .i_mrd64_req_ack            (mrd64_req_ack              ),
    //req information
    .o_req_length               (ctrl_req_length            ),
    .o_req_addr                 (ctrl_req_addr              ),
    .o_req_data                 (ctrl_req_data              ),
    .o_cross_4kb_boundary       (o_cross_4kb_boundary       ),
    //rst tlp cnt
    .i_dma_check_result         (dma_check_result           ),
    .o_tx_restart               (tx_restart                 )
);

ips2l_pcie_dma_tx_top #(
//    .DEVICE_TYPE                (DEVICE_TYPE                ),          //3'd0:EP,3'd1:Legacy EP,3'd4:RC
    .ADDR_WIDTH                 (ADDR_WIDTH                 )
)
u_ips2l_pcie_dma_tx_top
(
    .clk                        (clk                        ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                      (rst_n                      ),
    .i_cfg_pbus_num             (i_cfg_pbus_num             ),  //input [7:0]
    .i_cfg_pbus_dev_num         (i_cfg_pbus_dev_num         ),  //input [4:0]
    .i_cfg_max_rd_req_size      (i_cfg_max_rd_req_size      ),  //input [2:0]
    .i_cfg_max_payload_size     (i_cfg_max_payload_size     ),  //input [2:0]
    //**********************************************************************
    //axis_slave0 interface
    .i_axis_slave0_trdy         (dma_axis_slave0_trdy       ),
    .o_axis_slave0_tvld         (dma_axis_slave0_tvld       ),
    .o_axis_slave0_tdata        (dma_axis_slave0_tdata      ),
    .o_axis_slave0_tlast        (dma_axis_slave0_tlast      ),
    .o_axis_slave0_tuser        (dma_axis_slave0_tuser      ),
    //axis_slave1 interface
    .i_axis_slave1_trdy         (dma_axis_slave1_trdy       ),
    .o_axis_slave1_tvld         (dma_axis_slave1_tvld       ),
    .o_axis_slave1_tdata        (dma_axis_slave1_tdata      ),
    .o_axis_slave1_tlast        (dma_axis_slave1_tlast      ),
    .o_axis_slave1_tuser        (dma_axis_slave1_tuser      ),
    //axis_slave2 interface
    .i_axis_slave2_trdy         (dma_axis_slave2_trdy       ),
    .o_axis_slave2_tvld         (dma_axis_slave2_tvld       ),
    .o_axis_slave2_tdata        (dma_axis_slave2_tdata      ),
    .o_axis_slave2_tlast        (dma_axis_slave2_tlast      ),
    .o_axis_slave2_tuser        (dma_axis_slave2_tuser      ),
    //**********************************************************************
    //from dma_controller
    .i_user_define_data_flag    (tx_user_define_data_flag   ),

    .i_mwr32_req                (tx_mwr32_req               ),
    .o_mwr32_req_ack            (tx_mwr32_req_ack           ),
    .i_mwr64_req                (tx_mwr64_req               ),
    .o_mwr64_req_ack            (tx_mwr64_req_ack           ),

    .i_mrd32_req                (mrd32_req                  ),
    .o_mrd32_req_ack            (mrd32_req_ack              ),
    .i_mrd64_req                (mrd64_req                  ),
    .o_mrd64_req_ack            (mrd64_req_ack              ),

    .i_req_length               (tx_req_length              ),
    .i_req_addr                 (tx_req_addr                ),
    .i_req_data                 (tx_req_data                ),

    //**********************************************************************
    //bar0 rd interface
    .o_bar0_rd_clk_en           (bar0_rd_clk_en             ),
    .o_bar0_rd_addr             (bar0_rd_addr               ),
    .i_bar0_rd_data             (bar0_rd_data               ),
    //bar2 rd interface
    .o_bar2_rd_clk_en           (o_dma_write_data_req         ),
    .o_bar2_rd_addr             (o_dma_write_addr             ),
    .i_bar2_rd_data             (i_dma_write_data             ),
    //**********************************************************************
    //from rx top
    //req rcv
    .i_mrd_tc                   (mrd_tc                     ),
    .i_mrd_attr                 (mrd_attr                   ),
    .i_mrd_length               (mrd_length                 ),
    .i_mrd_id                   (mrd_id                     ),
    .i_mrd_tag                  (mrd_tag                    ),
    .i_mrd_addr                 (mrd_addr                   ),

    .i_cpld_req_vld             (cpld_req_vld               ),
    .o_cpld_req_rdy             (cpld_req_rdy               ),
    .o_cpld_tx_rdy              (cpld_tx_rdy                ),
    //cpld rcv
    .i_cpld_rcv                 (cpld_rcv                   ),
    .i_cpld_tag                 (cpld_tag                   ),
    .o_tag_full                 (tag_full                   ),
    //rst tlp cnt
    .i_tx_restart               (tx_restart                 ),
    .o_mwr_tx_busy             (mwr_tx_busy                )
    //debug
    //.o_dbg_bus_mrd_tx           (dbg_bus_mrd_tx             ),
    //.o_dbg_bus_mwr_tx           (dbg_bus_mwr_tx             )
);

generate
    if (AXIS_SLAVE_NUM == 1)
    begin:axis_slave_num_1
        ips2l_pcie_dma_tlp_tx_mux #(
            .AXIS_SLAVE_NUM             (AXIS_SLAVE_NUM         )
        )
        u_pcie_dma_tlp_tx_mux
        (
            .clk                         (clk                   ),   //gen1:62.5MHz,gen2:125MHz
            .rst_n                       (rst_n                 ),
            //**********************************************************************
            //from dma
            //axis_slave0 interface
            .o_dma_axis_slave0_trdy      (dma_axis_slave0_trdy  ),
            .i_dma_axis_slave0_tvld      (dma_axis_slave0_tvld  ),
            .i_dma_axis_slave0_tdata     (dma_axis_slave0_tdata ),
            .i_dma_axis_slave0_tlast     (dma_axis_slave0_tlast ),
            .i_dma_axis_slave0_tuser     (dma_axis_slave0_tuser ),
            //axis_slave1 interface
            .o_dma_axis_slave1_trdy      (dma_axis_slave1_trdy  ),
            .i_dma_axis_slave1_tvld      (dma_axis_slave1_tvld  ),
            .i_dma_axis_slave1_tdata     (dma_axis_slave1_tdata ),
            .i_dma_axis_slave1_tlast     (dma_axis_slave1_tlast ),
            .i_dma_axis_slave1_tuser     (dma_axis_slave1_tuser ),
            //axis_slave2 interface
            .o_dma_axis_slave2_trdy      (dma_axis_slave2_trdy  ),
            .i_dma_axis_slave2_tvld      (dma_axis_slave2_tvld  ),
            .i_dma_axis_slave2_tdata     (dma_axis_slave2_tdata ),
            .i_dma_axis_slave2_tlast     (dma_axis_slave2_tlast ),
            .i_dma_axis_slave2_tuser     (dma_axis_slave2_tuser ),
            //credit interface
            //from pcie
            .i_cfg_ido_req_en            (i_cfg_ido_req_en      ),
            .i_cfg_ido_cpl_en            (i_cfg_ido_cpl_en      ),
            .i_xadm_ph_cdts              (i_xadm_ph_cdts        ),
            .i_xadm_pd_cdts              (i_xadm_pd_cdts        ),
            .i_xadm_nph_cdts             (i_xadm_nph_cdts       ),
            .i_xadm_npd_cdts             (i_xadm_npd_cdts       ),
            .i_xadm_cplh_cdts            (i_xadm_cplh_cdts      ),
            .i_xadm_cpld_cdts            (i_xadm_cpld_cdts      ),
            //pcie_axis_slave0
            .i_pcie_axis_slave_trdy      (i_axis_slave0_trdy    ),
            .o_pcie_axis_slave_tvld      (o_axis_slave0_tvld    ),
            .o_pcie_axis_slave_tdata     (o_axis_slave0_tdata   ),
            .o_pcie_axis_slave_tlast     (o_axis_slave0_tlast   ),
            .o_pcie_axis_slave_tuser     (o_axis_slave0_tuser   )
        );

        assign o_axis_slave1_tvld  = 1'b0;
        assign o_axis_slave1_tdata = 128'b0;
        assign o_axis_slave1_tlast = 1'b0;
        assign o_axis_slave1_tuser = 1'b0;

        assign o_axis_slave2_tvld  = 1'b0;
        assign o_axis_slave2_tdata = 128'b0;
        assign o_axis_slave2_tlast = 1'b0;
        assign o_axis_slave2_tuser = 1'b0;
    end
    else if (AXIS_SLAVE_NUM == 2)
    begin:axis_slave_num_2
        ips2l_pcie_dma_tlp_tx_mux #(
            .AXIS_SLAVE_NUM             (AXIS_SLAVE_NUM         )
        )
        u_pcie_dma_tlp_tx_mux
        (
            .clk                         (clk                   ),   //gen1:62.5MHz,gen2:125MHz
            .rst_n                       (rst_n                 ),
            //**********************************************************************
            //from dma
            //axis_slave0 interface
            .o_dma_axis_slave0_trdy      (                      ),
            .i_dma_axis_slave0_tvld      (1'b0                  ),
            .i_dma_axis_slave0_tdata     (128'b0                ),
            .i_dma_axis_slave0_tlast     (1'b0                  ),
            .i_dma_axis_slave0_tuser     (1'b0                  ),
            //axis_slave1 interface
            .o_dma_axis_slave1_trdy      (dma_axis_slave1_trdy  ),
            .i_dma_axis_slave1_tvld      (dma_axis_slave1_tvld  ),
            .i_dma_axis_slave1_tdata     (dma_axis_slave1_tdata ),
            .i_dma_axis_slave1_tlast     (dma_axis_slave1_tlast ),
            .i_dma_axis_slave1_tuser     (dma_axis_slave1_tuser ),
            //axis_slave2 interface
            .o_dma_axis_slave2_trdy      (dma_axis_slave2_trdy  ),
            .i_dma_axis_slave2_tvld      (dma_axis_slave2_tvld  ),
            .i_dma_axis_slave2_tdata     (dma_axis_slave2_tdata ),
            .i_dma_axis_slave2_tlast     (dma_axis_slave2_tlast ),
            .i_dma_axis_slave2_tuser     (dma_axis_slave2_tuser ),
            //credit interface
            //from pcie
            .i_cfg_ido_req_en            (i_cfg_ido_req_en      ),
            .i_cfg_ido_cpl_en            (i_cfg_ido_cpl_en      ),
            .i_xadm_ph_cdts              (i_xadm_ph_cdts        ),
            .i_xadm_pd_cdts              (i_xadm_pd_cdts        ),
            .i_xadm_nph_cdts             (i_xadm_nph_cdts       ),
            .i_xadm_npd_cdts             (i_xadm_npd_cdts       ),
            .i_xadm_cplh_cdts            (i_xadm_cplh_cdts      ),
            .i_xadm_cpld_cdts            (i_xadm_cpld_cdts      ),
            //pcie_axis_slave0
            .i_pcie_axis_slave_trdy      (i_axis_slave1_trdy    ),
            .o_pcie_axis_slave_tvld      (o_axis_slave1_tvld    ),
            .o_pcie_axis_slave_tdata     (o_axis_slave1_tdata   ),
            .o_pcie_axis_slave_tlast     (o_axis_slave1_tlast   ),
            .o_pcie_axis_slave_tuser     (o_axis_slave1_tuser   )
        );

        assign dma_axis_slave0_trdy = i_axis_slave0_trdy    ;
        assign o_axis_slave0_tvld   = dma_axis_slave0_tvld  ;
        assign o_axis_slave0_tdata  = dma_axis_slave0_tdata ;
        assign o_axis_slave0_tlast  = dma_axis_slave0_tlast ;
        assign o_axis_slave0_tuser  = dma_axis_slave0_tuser ;

        assign o_axis_slave2_tvld  = 1'b0;
        assign o_axis_slave2_tdata = 128'b0;
        assign o_axis_slave2_tlast = 1'b0;
        assign o_axis_slave2_tuser = 1'b0;
    end
    else if (AXIS_SLAVE_NUM == 3)
    begin:axis_slave_num_3
        assign dma_axis_slave0_trdy = i_axis_slave0_trdy    ;
        assign o_axis_slave0_tvld   = dma_axis_slave0_tvld  ;
        assign o_axis_slave0_tdata  = dma_axis_slave0_tdata ;
        assign o_axis_slave0_tlast  = dma_axis_slave0_tlast ;
        assign o_axis_slave0_tuser  = dma_axis_slave0_tuser ;

        assign dma_axis_slave1_trdy = i_axis_slave1_trdy    ;
        assign o_axis_slave1_tvld   = dma_axis_slave1_tvld  ;
        assign o_axis_slave1_tdata  = dma_axis_slave1_tdata ;
        assign o_axis_slave1_tlast  = dma_axis_slave1_tlast ;
        assign o_axis_slave1_tuser  = dma_axis_slave1_tuser ;

        assign dma_axis_slave2_trdy = i_axis_slave2_trdy    ;
        assign o_axis_slave2_tvld   = dma_axis_slave2_tvld  ;
        assign o_axis_slave2_tdata  = dma_axis_slave2_tdata ;
        assign o_axis_slave2_tlast  = dma_axis_slave2_tlast ;
        assign o_axis_slave2_tuser  = dma_axis_slave2_tuser ;
    end
endgenerate

endmodule