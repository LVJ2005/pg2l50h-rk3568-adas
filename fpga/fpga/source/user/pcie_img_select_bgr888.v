module pcie_img_select_bgr888 (
    input                           clk,
    input                           rst_n,

    input                           line_full_flag,
    input                           frame_start,
    output wire                     frame_stream_ready,

    output reg                      ch0_data_req,
    input      [127:0]              ch0_data,
    input                           ch0_data_empty,
    output reg                      ch1_data_req,
    input      [127:0]              ch1_data,
    input                           ch1_data_empty,
    output reg                      ch2_data_req,
    input      [127:0]              ch2_data,
    input                           ch2_data_empty,
    output reg                      ch3_data_req,
    input      [127:0]              ch3_data,
    input                           ch3_data_empty,

    input                           dma_wr_data_req,
    input      [11:0]              dma_wr_addr,
    output reg [127:0]              dma_wr_data
);

localparam [11:0] ROW_NUM          = 12'd720;
localparam [11:0] HALF_ROW_NUM     = 12'd360;
localparam [6:0]  PAIRS_PER_LINE   = 7'd80;
localparam [6:0]  HALF_PAIRS       = 7'd40;
localparam [10:0] OUTPUT_PREFILL    = 11'd256;

function [7:0] expand5;
    input [4:0] v;
    begin
        expand5 = {v, v[4:2]};
    end
endfunction

function [7:0] expand6;
    input [5:0] v;
    begin
        expand6 = {v, v[5:4]};
    end
endfunction

function [23:0] rgb565_to_bgr24;
    input [15:0] p;
    reg [7:0] r;
    reg [7:0] g;
    reg [7:0] b;
    begin
        r = expand5(p[15:11]);
        g = expand6(p[10:5]);
        b = expand5(p[4:0]);
        rgb565_to_bgr24 = {r, g, b};
    end
endfunction

function [383:0] pack_rgb565_pair_to_bgr888;
    input [127:0] a;
    input [127:0] b;
    begin
        pack_rgb565_pair_to_bgr888 = {
            rgb565_to_bgr24(b[127:112]),
            rgb565_to_bgr24(b[111:96]),
            rgb565_to_bgr24(b[95:80]),
            rgb565_to_bgr24(b[79:64]),
            rgb565_to_bgr24(b[63:48]),
            rgb565_to_bgr24(b[47:32]),
            rgb565_to_bgr24(b[31:16]),
            rgb565_to_bgr24(b[15:0]),
            rgb565_to_bgr24(a[127:112]),
            rgb565_to_bgr24(a[111:96]),
            rgb565_to_bgr24(a[95:80]),
            rgb565_to_bgr24(a[79:64]),
            rgb565_to_bgr24(a[63:48]),
            rgb565_to_bgr24(a[47:32]),
            rgb565_to_bgr24(a[31:16]),
            rgb565_to_bgr24(a[15:0])
        };
    end
endfunction

reg [383:0] curr_pair;
reg [383:0] next_pair;
reg         next_pair_valid;
reg         curr_pair_last;
reg         next_pair_last;
reg [1:0]   out_phase;

reg [6:0]   fetch_pair_cnt;
reg [11:0]  fetch_row_cnt;
reg         fetch_half;
reg [127:0] fetch_first_beat;
reg         primed;

reg         stream_ready;
reg         line_full_went_low;
reg         read_valid_dly;
reg [1:0]   read_ch_dly;
reg [127:0] read_data_mux;
reg         source_empty_mux;

reg         dma_wr_data_req_d;
reg [11:0]  dma_wr_addr_d;

wire        fetch_left_half;
wire        fetch_top_half;
wire [1:0]  issue_ch;
wire        issue_read;
wire        finishing_pair_capture;
wire        fetch_buffer_needs_pair;
wire        producer_advance;
wire        producer_can_finish_pair;
wire        captured_pair_last;
wire        will_consume_next;
reg [127:0] producer_data;

wire        out_fifo_rst;
wire        out_fifo_wr_en;
wire [127:0] out_fifo_wr_data;
wire        out_fifo_full;
wire [10:0] out_fifo_wr_level;
wire        out_fifo_rd_en;
wire [127:0] out_fifo_rd_data;
wire        out_fifo_empty;
wire [10:0] out_fifo_rd_level;
wire        dma_word_advance;

assign fetch_left_half = (fetch_pair_cnt < HALF_PAIRS);
assign fetch_top_half = (fetch_row_cnt <= HALF_ROW_NUM);
assign issue_ch = fetch_top_half ? (fetch_left_half ? 2'd0 : 2'd1)
                                 : (fetch_left_half ? 2'd2 : 2'd3);

always @(*) begin
    case (issue_ch)
        2'd0: source_empty_mux = ch0_data_empty;
        2'd1: source_empty_mux = ch1_data_empty;
        2'd2: source_empty_mux = ch2_data_empty;
        default: source_empty_mux = ch3_data_empty;
    endcase
end

always @(*) begin
    case (read_ch_dly)
        2'd0: read_data_mux = ch0_data;
        2'd1: read_data_mux = ch1_data;
        2'd2: read_data_mux = ch2_data;
        default: read_data_mux = ch3_data;
    endcase
end

always @(*) begin
    case (out_phase)
        2'd0: producer_data = curr_pair[127:0];
        2'd1: producer_data = curr_pair[255:128];
        default: producer_data = curr_pair[383:256];
    endcase
end

assign finishing_pair_capture = read_valid_dly && fetch_half;
assign producer_can_finish_pair =
    (out_phase != 2'd2) || next_pair_valid || curr_pair_last;
assign producer_advance =
    primed && !out_fifo_full && producer_can_finish_pair;
assign captured_pair_last =
    (fetch_row_cnt == ROW_NUM) &&
    (fetch_pair_cnt == (PAIRS_PER_LINE - 7'd1));
assign will_consume_next =
    producer_advance && (out_phase == 2'd2) && next_pair_valid;
assign fetch_buffer_needs_pair =
    (!primed) || (!next_pair_valid) || will_consume_next;
assign issue_read =
    stream_ready && fetch_buffer_needs_pair &&
    !finishing_pair_capture && !source_empty_mux;

assign out_fifo_rst = !rst_n || frame_start;
assign out_fifo_wr_en = producer_advance;
assign out_fifo_wr_data = producer_data;

assign dma_word_advance =
    dma_wr_data_req &&
    (!dma_wr_data_req_d || (dma_wr_addr != dma_wr_addr_d));
assign out_fifo_rd_en = dma_word_advance && !out_fifo_empty;

assign frame_stream_ready =
    stream_ready && (out_fifo_rd_level >= OUTPUT_PREFILL);

afifo_128i_128o_512 output_prefetch_fifo (
    .wr_clk             (clk),
    .wr_rst             (out_fifo_rst),
    .wr_en              (out_fifo_wr_en),
    .wr_data            (out_fifo_wr_data),
    .wr_full            (out_fifo_full),
    .wr_water_level     (out_fifo_wr_level),
    .almost_full        (),
    .rd_clk             (clk),
    .rd_rst             (out_fifo_rst),
    .rd_en              (out_fifo_rd_en),
    .rd_data            (out_fifo_rd_data),
    .rd_empty           (out_fifo_empty),
    .rd_water_level     (out_fifo_rd_level),
    .almost_empty       ()
);

always @(posedge clk) begin
    if (!rst_n || frame_start) begin
        ch0_data_req <= 1'b0;
        ch1_data_req <= 1'b0;
        ch2_data_req <= 1'b0;
        ch3_data_req <= 1'b0;

        curr_pair <= 384'd0;
        next_pair <= 384'd0;
        next_pair_valid <= 1'b0;
        curr_pair_last <= 1'b0;
        next_pair_last <= 1'b0;
        out_phase <= 2'd0;

        fetch_pair_cnt <= 7'd0;
        fetch_row_cnt <= 12'd1;
        fetch_half <= 1'b0;
        fetch_first_beat <= 128'd0;
        primed <= 1'b0;

        stream_ready <= 1'b0;
        line_full_went_low <= 1'b0;
        read_valid_dly <= 1'b0;
        read_ch_dly <= 2'd0;

        dma_wr_data_req_d <= 1'b0;
        dma_wr_addr_d <= 12'd0;
        dma_wr_data <= 128'd0;
    end else begin
        ch0_data_req <= 1'b0;
        ch1_data_req <= 1'b0;
        ch2_data_req <= 1'b0;
        ch3_data_req <= 1'b0;

        // Reject a stale line_full level from the previous frame. The DDR
        // FIFOs must first report empty, then refill to the new-frame level.
        if (!line_full_flag)
            line_full_went_low <= 1'b1;
        if (line_full_went_low && line_full_flag)
            stream_ready <= 1'b1;

        if (issue_read) begin
            case (issue_ch)
                2'd0: ch0_data_req <= 1'b1;
                2'd1: ch1_data_req <= 1'b1;
                2'd2: ch2_data_req <= 1'b1;
                default: ch3_data_req <= 1'b1;
            endcase
        end

        if (read_valid_dly) begin
            if (!fetch_half) begin
                fetch_first_beat <= read_data_mux;
                fetch_half <= 1'b1;
            end else begin
                if (!primed) begin
                    curr_pair <=
                        pack_rgb565_pair_to_bgr888(fetch_first_beat,
                                                   read_data_mux);
                    primed <= 1'b1;
                    curr_pair_last <= captured_pair_last;
                end else begin
                    next_pair <=
                        pack_rgb565_pair_to_bgr888(fetch_first_beat,
                                                   read_data_mux);
                    next_pair_valid <= 1'b1;
                    next_pair_last <= captured_pair_last;
                end

                fetch_half <= 1'b0;
                if (fetch_pair_cnt == (PAIRS_PER_LINE - 7'd1)) begin
                    fetch_pair_cnt <= 7'd0;
                    if (fetch_row_cnt == ROW_NUM)
                        fetch_row_cnt <= 12'd1;
                    else
                        fetch_row_cnt <= fetch_row_cnt + 1'b1;
                end else begin
                    fetch_pair_cnt <= fetch_pair_cnt + 1'b1;
                end
            end
        end

        read_valid_dly <= issue_read;
        if (issue_read)
            read_ch_dly <= issue_ch;

        if (producer_advance) begin
            if (out_phase == 2'd2) begin
                out_phase <= 2'd0;
                if (next_pair_valid) begin
                    curr_pair <= next_pair;
                    curr_pair_last <= next_pair_last;
                    next_pair_valid <= 1'b0;
                end else begin
                    primed <= 1'b0;
                end
            end else begin
                out_phase <= out_phase + 1'b1;
            end
        end

        dma_wr_data_req_d <= dma_wr_data_req;
        if (dma_wr_data_req)
            dma_wr_addr_d <= dma_wr_addr;

        if (dma_word_advance) begin
            if (!out_fifo_empty)
                dma_wr_data <= out_fifo_rd_data;
            else
                dma_wr_data <= 128'd0;
        end else if (!dma_wr_data_req) begin
            dma_wr_data <= 128'd0;
        end
    end
end

endmodule