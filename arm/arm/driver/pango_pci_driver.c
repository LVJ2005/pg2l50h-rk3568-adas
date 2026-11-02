#include "pango_pci_driver.h"
#include "id_config.h"

//#define NDEBUG

#ifndef NDEBUG
#define LOG(msg ...) printk(KERN_ALERT msg)
#else
#define LOG(msg ...)
#endif

#define PCI_DRIVER_DEV_COUNT 1
#define PCI_DRIVER_DEV_NAME "pango_pci_driver"

struct file_operations pango_cdev_fops = {
	.owner   = THIS_MODULE,
	.llseek  = pango_cdev_llseek,
	.read    = pango_cdev_read,
	.write   = pango_cdev_write,
	.open    = pango_cdev_open,
	.unlocked_ioctl = pango_cdev_ioctl,
	.mmap    = pango_cdev_mmap,
	.release = pango_cdev_release,
};

struct PciDriverDevInfo pci_dev_info = {
	._dev            = 0,
	._dev_firstminor = 0,
	._dev_count      = PCI_DRIVER_DEV_COUNT,
	._dev_name       = PCI_DRIVER_DEV_NAME,
};

struct pci_device_id pci_pango_device_ids[] = {
	{ PCI_DEVICE(PCI_PANGO_DEFAULT_VENDOR_ID, PCI_PANGO_DEFAULT_DEVICE_ID) },
	{ 0, },
};

struct PciPango pci_info = {
	._pango_pci_driver = {
		._pci_bar     = 1,
		._pci_io_size = 0,
		._pci_io      = NULL,
		._pci_io_buff = NULL,
		._pci_driver  = {
			.name     = PCI_DRIVER_DEV_NAME,
			.id_table = pci_pango_device_ids,
			.probe    = pci_driver_probe,
			.remove   = pci_driver_remove,
		},
	},
	._cdev_class = NULL,
};


struct pci_dev *op_dev;

static wait_queue_head_t dma_wait_queue;
static void *frame_data_buf[FRAME_BUFFER_COUNT];
static dma_addr_t frame_dma_addr[FRAME_BUFFER_COUNT];
static void *frame_data_pool;
static dma_addr_t frame_dma_pool_addr;
static size_t frame_pool_size;
static size_t frame_buffer_size;
static size_t dma_line_buffer_size;
static spinlock_t frame_state_lock;
static unsigned long frame_ready_mask;
static unsigned int frame_irq_expected_index;
static unsigned int frame_selected_index;
static bool frame_selected_valid;
static bool frame_dma_running;
static unsigned int frame_buffer_seq[FRAME_BUFFER_COUNT];
static unsigned int frame_sequence;
static unsigned long frame_irq_count;
static unsigned long frame_overrun_count;
static unsigned long frame_timeout_count;
static int pango_msi_irq = -1;
static bool pango_msi_enabled;
static unsigned int dma_line_delay_us = 20;
static unsigned int dma_frame_tail_delay_us = 300;
static bool frame_batch_mode = true;
module_param(dma_line_delay_us, uint, 0644);
MODULE_PARM_DESC(dma_line_delay_us,
		 "Compatibility parameter; ignored by full-frame MSI DMA");
module_param(dma_frame_tail_delay_us, uint, 0644);
MODULE_PARM_DESC(dma_frame_tail_delay_us,
		 "Compatibility parameter; ignored by full-frame MSI DMA");
module_param(frame_batch_mode, bool, 0644);
MODULE_PARM_DESC(frame_batch_mode,
		 "Compatibility parameter; full-frame MSI DMA is always used");

static void free_capture_buffers(void)
{
	int i;

	if (dma_info.addr_r.data_buf) {
		dma_free_coherent(&op_dev->dev, dma_line_buffer_size,
				  dma_info.addr_r.data_buf, dma_info.addr_r.addr);
		dma_info.addr_r.data_buf = NULL;
	}

	if (dma_info.addr_w.data_buf) {
		dma_free_coherent(&op_dev->dev, dma_line_buffer_size,
				  dma_info.addr_w.data_buf, dma_info.addr_w.addr);
		dma_info.addr_w.data_buf = NULL;
	}

	if (frame_data_pool) {
		dma_free_coherent(&op_dev->dev, frame_pool_size,
				  frame_data_pool, frame_dma_pool_addr);
		frame_data_pool = NULL;
		frame_dma_pool_addr = 0;
		frame_pool_size = 0;
	}
	for (i = 0; i < FRAME_BUFFER_COUNT; i++) {
		frame_data_buf[i] = NULL;
		frame_dma_addr[i] = 0;
	}

	frame_buffer_size = 0;
	dma_line_buffer_size = 0;
	frame_ready_mask = 0;
	frame_irq_expected_index = 0;
	frame_selected_index = 0;
	frame_selected_valid = false;
	frame_dma_running = false;
	frame_sequence = 0;
	frame_irq_count = 0;
	frame_overrun_count = 0;
	frame_timeout_count = 0;
	msi_operation.msi_flag = 0;
	msi_operation.frame_index = 0;
	msi_operation.frame_size = 0;
	msi_operation.frame_seq = 0;
}

static int alloc_capture_buffers(size_t line_size)
{
	int i;

	if (!op_dev || line_size != IMAGE_LINE_BYTES)
		return -EINVAL;

	if (dma_info.addr_r.data_buf && dma_info.addr_w.data_buf &&
	    frame_data_buf[0] && frame_buffer_size == line_size * IMAGE_FRAME_LINES &&
	    dma_line_buffer_size == line_size)
		return 0;

	free_capture_buffers();

	dma_info.addr_r.data_buf = dma_alloc_coherent(&op_dev->dev, line_size,
						      &dma_info.addr_r.addr, GFP_KERNEL);
	if (!dma_info.addr_r.data_buf)
		goto fail;

	dma_info.addr_w.data_buf = dma_alloc_coherent(&op_dev->dev, line_size,
						      &dma_info.addr_w.addr, GFP_KERNEL);
	if (!dma_info.addr_w.data_buf)
		goto fail;

	dma_line_buffer_size = line_size;
	frame_buffer_size = line_size * IMAGE_FRAME_LINES;
	frame_pool_size = frame_buffer_size * FRAME_BUFFER_COUNT;
	frame_data_pool = dma_alloc_coherent(&op_dev->dev, frame_pool_size,
					     &frame_dma_pool_addr, GFP_KERNEL);
	if (!frame_data_pool)
		goto fail;

	for (i = 0; i < FRAME_BUFFER_COUNT; i++) {
		frame_data_buf[i] = (void *)((char *)frame_data_pool + (i * frame_buffer_size));
		frame_dma_addr[i] = frame_dma_pool_addr + (i * frame_buffer_size);
		memset(frame_data_buf[i], 0, frame_buffer_size);
	}

	dma_info.addr_r.addr_size = ((dma_info.addr_r.addr >> 32) > 0) ? 1 : 0;
	dma_info.addr_w.addr_size = ((dma_info.addr_w.addr >> 32) > 0) ? 1 : 0;
	frame_ready_mask = 0;
	frame_irq_expected_index = 0;
	frame_selected_index = 0;
	frame_selected_valid = false;
	frame_dma_running = false;
	memset(frame_buffer_seq, 0, sizeof(frame_buffer_seq));
	frame_sequence = 0;
	msi_operation.frame_size = frame_buffer_size;
	msi_operation.frame_index = 0;
	msi_operation.frame_seq = 0;
	return 0;

fail:
	free_capture_buffers();
	return -ENOMEM;
}

int pango_cdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	size_t req_size;
	int ret;

	req_size = (size_t)(vma->vm_end - vma->vm_start);
	if (!frame_data_pool || frame_pool_size == 0)
		return -ENODEV;
	if (req_size > frame_pool_size)
		return -EINVAL;

	ret = dma_mmap_coherent(&op_dev->dev, vma, frame_data_pool,
				frame_dma_pool_addr, req_size);
	return ret;
}

///////////////////////////////////////////////////////////////////////

loff_t pango_cdev_llseek(struct file *filp, loff_t off, int whence)
{
	struct PciPango *pci_pango = &pci_info;
	loff_t newpos = 0;
	
	//LOG("pango_cdev_llseek.\n");
	
	switch (whence)
	{
	case 0: /* SEEK_SET */
	{
		newpos = off;
	}
		break;
	case 1: /* SEEK_CUR */
	{
		newpos = filp->f_pos + off;
	}
		break;
	case 2: /* SEEK_END */
	{
		newpos = pci_pango->_pango_pci_driver._pci_io_size + off;
	}
		break;
	default: /* can't happen */
	{
		return -EINVAL;
	}	
		break;
	}
	
	if (newpos < 0)
	{
		return -EINVAL;
	}
	
	filp->f_pos = newpos;
	
	return newpos;
}

static void ReadConfig(struct pci_dev * pdev)
{
	int i;
	u8 valb;
	u16 valw;
	u32 valdw;
	u8 id;
	u8 next;
	int pos;
	unsigned long reg_base, reg_len;

	/* Read PCI configuration space 读取PCI配置空间*/
	printk(KERN_INFO "PCI Configuration Space:\n");
	for(i=0; i< 1024; i++)
	{
		pci_read_config_dword(pdev, i*4, &valdw);
		command_operation.get_pci_dev_info.data[i] = valdw;
	}

	/* Now read each element - one at a time 现在读取每个元素——一次读取一个*/

	/* Read Vendor ID 读取厂商ID*/
	pci_read_config_word(pdev, PCI_VENDOR_ID, &valw);
	printk("Vendor ID: 0x%x, \n", valw);
	command_operation.get_pci_dev_info.vendor_id = valw;

	/* Read Device ID 读取设备ID*/
	pci_read_config_word(pdev, PCI_DEVICE_ID, &valw);
	printk("Device ID: 0x%x, \n", valw);
	command_operation.get_pci_dev_info.device_id = valw;

	/* Read Command Register 读取命令寄存器*/
	pci_read_config_word(pdev, PCI_COMMAND, &valw);
	printk("Cmd Reg: 0x%x, \n", valw);
	command_operation.get_pci_dev_info.cmd_reg = valw;

	/* Read Status Register 读取状态寄存器*/
	pci_read_config_word(pdev, PCI_STATUS, &valw);
	printk("Stat Reg: 0x%x, \n", valw);
	command_operation.get_pci_dev_info.status_reg = valw;

	/* Read Revision ID 阅读修订ID*/
	pci_read_config_byte(pdev, PCI_REVISION_ID, &valb);
	printk("Revision ID: 0x%x, \n", valb);
	command_operation.get_pci_dev_info.revision_id = valb;

	/* Read Class Code */
	/*
	pci_read_config_dword(pdev, PCI_CLASS_PROG, &valdw);
	printk("Class Code: 0x%lx, ", valdw);
	valdw &= 0x00ffffff;
	printk("Class Code: 0x%lx, ", valdw);
	*/
	/* Read Reg-level Programming Interface 读取reg级编程接口*/
	pci_read_config_byte(pdev, PCI_CLASS_PROG, &valb);
	printk("Class Prog: 0x%x, \n", valb);
	command_operation.get_pci_dev_info.class_prog = valb;

	/* Read Device Class */
	pci_read_config_word(pdev, PCI_CLASS_DEVICE, &valw);
	printk("Device Class: 0x%x, \n", valw);
	command_operation.get_pci_dev_info.class_device = valw;

	  /* Read Cache Line 读缓存线*/
	  pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &valb);
	  printk("Cache Line Size: 0x%x, \n", valb);
	
	  /* Read Latency Timer 读延迟计时器*/
	  pci_read_config_byte(pdev, PCI_LATENCY_TIMER, &valb);
	  printk("Latency Timer: 0x%x, \n", valb);
	
	  /* Read Header Type 读头类型*/
	  pci_read_config_byte(pdev, PCI_HEADER_TYPE, &valb);
	  printk("Header Type: 0x%x, \n", valb);
	
	  /* Read BIST */
	  pci_read_config_byte(pdev, PCI_BIST, &valb);
	  printk("BIST: 0x%x\n", valb);

	/* Read all 6 BAR registers 读取所有6个BAR寄存器*/
	for(i=0; i<=5; i++)
	{
		/* Physical address & length 物理地址和长度*/
		reg_base = pci_resource_start(pdev, i);
		reg_len = pci_resource_len(pdev, i);
		printk("BAR%d: Addr:0x%lx Len:0x%lx,  ", i, reg_base, reg_len);
		command_operation.get_pci_dev_info.bar[i].bar_base = reg_base;
		command_operation.get_pci_dev_info.bar[i].bar_len = reg_len;

		/* Flags */
		if((pci_resource_flags(pdev, i) & IORESOURCE_MEM))
		  printk("Region is for memory\n");
		else if((pci_resource_flags(pdev, i) & IORESOURCE_IO))
		  printk("Region is for I/O\n");
	}
	printk("\n");

	  /* Read CIS Pointer 读CIS指针*/
	  pci_read_config_dword(pdev, PCI_CARDBUS_CIS, &valdw);
	  printk("CardBus CIS Pointer: 0x%x, \n", valdw);
	
	  /* Read Subsystem Vendor ID 读取子系统供应商ID*/
	  pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, &valw);
	  printk("Subsystem Vendor ID: 0x%x, \n", valw);
	
	  /* Read Subsystem Device ID 读取子系统设备ID*/
	  pci_read_config_word(pdev, PCI_SUBSYSTEM_ID, &valw);
	  printk("Subsystem Device ID: 0x%x\n", valw);
	
	  /* Read Expansion ROM Base Address 读取扩展ROM的基地地址*/
	  pci_read_config_dword(pdev, PCI_ROM_ADDRESS, &valdw);
	  printk("Expansion ROM Base Address: 0x%x\n", valdw);
	
	  /* Read IRQ Line 读IRQ线*/
	  pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &valb);
	  printk("IRQ Line: 0x%x, ", valb);
	
	  /* Read IRQ Pin 读IRQ引脚*/
	  pci_read_config_byte(pdev, PCI_INTERRUPT_PIN, &valb);
	  printk("IRQ Pin: 0x%x, ", valb);

	/* Read Min Gnt */
	pci_read_config_byte(pdev, PCI_MIN_GNT, &valb);
	printk("Min Gnt: 0x%x, ", valb);
	command_operation.get_pci_dev_info.min_gnt = valb;

	/* Read Max Lat */
	pci_read_config_byte(pdev, PCI_MAX_LAT, &valb);
	printk("Max Lat: 0x%x\n", valb);
	command_operation.get_pci_dev_info.max_lat = valb;
	
	if(pci_find_capability(pdev, PCI_CAP_ID_EXP))
	{
		pos = pci_find_capability(pdev, PCI_CAP_ID_EXP);
		pci_read_config_word(pdev, pos + PCI_EXP_LNKSTA, &valw);
		command_operation.get_pci_dev_info.link_speed = (valw & 0x0003);
		command_operation.get_pci_dev_info.link_width = (valw & 0x03f0) >> 4;
		printk("Link Speed: %d\n", command_operation.get_pci_dev_info.link_speed);
		printk("Link Width: x%d\n", command_operation.get_pci_dev_info.link_width);
		
		pci_read_config_word(pdev, pos + PCI_EXP_DEVCTL, &valw);
		command_operation.get_pci_dev_info.mps = 128 << ((valw & PCI_EXP_DEVCTL_PAYLOAD) >> 5);
		command_operation.get_pci_dev_info.mrrs = 128 << ((valw & PCI_EXP_DEVCTL_READRQ) >> 12);
		printk("MPS: %d\n", command_operation.get_pci_dev_info.mps);
		printk("MRRS: %d\n", command_operation.get_pci_dev_info.mrrs);
		
	}
	else
	{
		printk("Cannot find PCI Express Capabilities\n");
		command_operation.get_pci_dev_info.link_speed = 0;
		command_operation.get_pci_dev_info.link_width = 0;
		command_operation.get_pci_dev_info.mps = 0;
		command_operation.get_pci_dev_info.mrrs = 0;
	}

	pci_read_config_word(pdev, PCI_STATUS, &valw);
	command_operation.cap_info.cap_error = 0;
	if (!(valw & PCI_STATUS_CAP_LIST))
	{
		command_operation.cap_info.cap_status = 0;
	}
	else
	{
		command_operation.cap_info.cap_status = 1;
		for(i = 0; i < 256; i++)
		{
			command_operation.cap_info.cap_buf[i].flag = 0;
		}
		pci_read_config_byte(pdev, PCI_CAPABILITY_LIST, &valb);
		valb &= ~3;
		while(valb)
		{

			pci_read_config_byte(pdev, valb + PCI_CAP_LIST_ID, &id);		
			pci_read_config_byte(pdev, valb + PCI_CAP_LIST_NEXT, &next);
			next &= ~3;
			command_operation.cap_info.cap_buf[valb].flag = 1;
			command_operation.cap_info.cap_buf[valb].id = id;
			command_operation.cap_info.cap_buf[valb].addr_offset = valb;
			command_operation.cap_info.cap_buf[valb].next_offset = next;
			printk("cap id = %x; addr_offset = %x; next_offset = %x\n", id, valb, next);
			if(id == 0xff)
			{
				command_operation.cap_info.cap_error = 1;
				break;
			}
			valb = next;
		}
	}

}

ssize_t pango_cdev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	if (copy_to_user(buf, &command_operation, sizeof(COMMAND_OPERATION)))
		return -EFAULT;
	return 1;
}

ssize_t pango_cdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	return 1;
}

static void set_dma_w_r(unsigned int value, struct PciPango *pci_pango)
{
	iowrite32(value, pci_pango->_pango_pci_driver._pci_io + CMD_REG_OFFSET);
}

static void set_dma_addr(DMA_ADDR* dma_addr, struct PciPango *pci_pango)
{
	iowrite32(dma_addr->addr & 0x00000000ffffffff, pci_pango->_pango_pci_driver._pci_io + RW_ADDR_LO_OFFSET);
	if(dma_addr->addr_size)
		iowrite32(dma_addr->addr >> 32, pci_pango->_pango_pci_driver._pci_io + RW_ADDR_HI_OFFSET);
}

static inline void dma_mmio_posted_write_flush(struct PciPango *pci_pango)
{
	(void)ioread32(pci_pango->_pango_pci_driver._pci_io + CMD_REG_OFFSET);
}

static inline void frame_reg_write(unsigned int offset, unsigned int value)
{
	iowrite32(value, pci_info._pango_pci_driver._pci_io + offset);
}

static int frame_dma_program_buffers(void)
{
	if (!frame_data_pool || frame_buffer_size != IMAGE_FRAME_BYTES)
		return -EINVAL;
	if ((frame_dma_addr[0] >> 32) || (frame_dma_addr[1] >> 32))
		return -ERANGE;
	if ((frame_buffer_size % FRAME_DMA_CHUNK_BYTES) != 0)
		return -EINVAL;

	frame_reg_write(FRAME_CONTROL_OFFSET, FRAME_CONTROL_STOP);
	dma_mmio_posted_write_flush(&pci_info);
	frame_reg_write(FRAME_BUF0_LO_OFFSET, lower_32_bits(frame_dma_addr[0]));
	frame_reg_write(FRAME_BUF1_LO_OFFSET, lower_32_bits(frame_dma_addr[1]));
	frame_reg_write(FRAME_CHUNK_COUNT_OFFSET, FRAME_DMA_CHUNK_COUNT);
	wmb();
	dma_mmio_posted_write_flush(&pci_info);
	return 0;
}

static void frame_dma_stop(void)
{
	unsigned long flags;

	if (!pci_info._pango_pci_driver._pci_io)
		return;

	frame_reg_write(FRAME_CONTROL_OFFSET, FRAME_CONTROL_STOP);
	dma_mmio_posted_write_flush(&pci_info);

	/* FPGA finishes at most the active 2 KiB request, then enters IDLE. */
	msleep(20);
	if (pango_msi_irq >= 0)
		synchronize_irq(pango_msi_irq);

	spin_lock_irqsave(&frame_state_lock, flags);
	frame_dma_running = false;
	frame_ready_mask = 0;
	frame_selected_valid = false;
	msi_operation.msi_flag = 0;
	spin_unlock_irqrestore(&frame_state_lock, flags);
	wake_up_interruptible(&dma_wait_queue);
}

static irqreturn_t pango_frame_msi_handler(int irq, void *dev_id)
{
	unsigned long flags;
	unsigned int completed;

	dma_rmb();
	spin_lock_irqsave(&frame_state_lock, flags);
	if (!frame_dma_running || !frame_data_pool) {
		spin_unlock_irqrestore(&frame_state_lock, flags);
		return IRQ_HANDLED;
	}

	completed = frame_irq_expected_index;
	if (frame_ready_mask & (1UL << completed))
		frame_overrun_count++;

	frame_sequence++;
	frame_buffer_seq[completed] = frame_sequence;
	frame_ready_mask |= 1UL << completed;
	frame_irq_expected_index ^= 1U;
	frame_irq_count++;
	frame_timeout_count = 0;

	msi_operation.msi_flag = 1;
	msi_operation.frame_index = completed;
	msi_operation.frame_size = frame_buffer_size;
	msi_operation.frame_seq = frame_sequence;
	spin_unlock_irqrestore(&frame_state_lock, flags);

	wake_up_interruptible(&dma_wait_queue);
	return IRQ_HANDLED;
}

long pango_cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct PciPango *pci_pango = &pci_info;
	unsigned char dest_buf[100];
	unsigned int temp_data = 0;
	unsigned int i = 0;
	int result = 0;

	if (down_interruptible(&pci_pango->_sem)) {
		printk(KERN_ALERT "********* pango_cdev_read interruptible *********\n");
		return -ERESTARTSYS;
	}

	switch (cmd) {
	case PCI_READ_DATA_CMD:
		if (copy_from_user(&config_operation, (COMMAND_OPERATION *)arg, sizeof(COMMAND_OPERATION))) {
			result = -EFAULT;
			break;
		}
		pci_read_config_dword(op_dev, config_operation.addr, &config_operation.data);
		if (copy_to_user((COMMAND_OPERATION *)arg, &config_operation, sizeof(COMMAND_OPERATION)))
			result = -EFAULT;
		break;

	case PCI_WRITE_DATA_CMD:
		if (copy_from_user(&config_operation, (COMMAND_OPERATION *)arg, sizeof(COMMAND_OPERATION))) {
			result = -EFAULT;
			break;
		}
		pci_write_config_dword(op_dev, config_operation.addr, config_operation.data);
		break;

	case PCI_MAP_ADDR_CMD:
		if (copy_from_user(&dma_operation, (DMA_OPERATION *)arg,
				   sizeof(DMA_OPERATION))) {
			result = -EFAULT;
			break;
		}

		if ((dma_operation.current_len * 4) != IMAGE_LINE_BYTES) {
			printk(KERN_ERR "frame DMA requires %u-byte BGR888 lines, got %u\n",
			       IMAGE_LINE_BYTES, dma_operation.current_len * 4);
			result = -EINVAL;
			break;
		}
		if (frame_dma_running) {
			result = -EBUSY;
			break;
		}

		result = alloc_capture_buffers(IMAGE_LINE_BYTES);
		if (result)
			break;
		result = frame_dma_program_buffers();
		if (result) {
			free_capture_buffers();
			break;
		}

		dma_info.cmd.data.length = dma_operation.current_len - 1;
		dma_info.cmd.data.addr_type = dma_info.addr_r.addr_size;
		printk(KERN_INFO
		       "frame DMA ping-pong: buf0=%pad buf1=%pad size=%zu chunks=%u\n",
		       &frame_dma_addr[0], &frame_dma_addr[1], frame_buffer_size,
		       FRAME_DMA_CHUNK_COUNT);
		msi_operation.frame_size = frame_buffer_size;
		break;

	case PCI_WRITE_TO_KERNEL_CMD:
		spin_lock(&dma_info.addr_r.lock);
		if (copy_from_user(&dma_operation, (DMA_OPERATION *)arg, sizeof(DMA_OPERATION))) {
			spin_unlock(&dma_info.addr_r.lock);
			result = -EFAULT;
			break;
		}
		memcpy(dma_info.addr_r.data_buf, dma_operation.data.write_buf,
		       dma_operation.current_len * 4);
		memset(dma_info.addr_w.data_buf, 0, dma_operation.current_len * 4);
		spin_unlock(&dma_info.addr_r.lock);
		break;

	case PCI_DMA_READ_CMD:
		spin_lock(&dma_info.addr_r.lock);
		dma_info.cmd.data.op_type = 0;
		set_dma_w_r(dma_info.cmd.value, pci_pango);
		set_dma_addr(&dma_info.addr_r, pci_pango);
		spin_unlock(&dma_info.addr_r.lock);
		break;

	case PCI_DMA_WRITE_CMD:
	{
		unsigned long flags;
		bool start_dma = false;

		if (!pango_msi_enabled || !frame_data_pool) {
			result = -ENODEV;
			break;
		}

		spin_lock_irqsave(&frame_state_lock, flags);
		if (!frame_dma_running) {
			frame_ready_mask = 0;
			frame_irq_expected_index = 0;
			frame_selected_index = 0;
			frame_selected_valid = false;
			memset(frame_buffer_seq, 0, sizeof(frame_buffer_seq));
			frame_sequence = 0;
			frame_timeout_count = 0;
			msi_operation.msi_flag = 0;
			msi_operation.frame_index = 0;
			msi_operation.frame_size = frame_buffer_size;
			msi_operation.frame_seq = 0;
			frame_dma_running = true;
			start_dma = true;
		}
		spin_unlock_irqrestore(&frame_state_lock, flags);

		if (start_dma) {
			wmb();
			frame_reg_write(FRAME_CONTROL_OFFSET, FRAME_CONTROL_START);
			dma_mmio_posted_write_flush(&pci_info);
			printk(KERN_INFO "full-frame DMA started with hardware MSI\n");
		}
		break;
	}

	case PCI_READ_FROM_KERNEL_CMD:
	{
		int wait_ret;
		unsigned long flags;
		unsigned int ready_index;
		void *ready_frame;

		up(&pci_pango->_sem);
		wait_ret = wait_event_interruptible_timeout(
			dma_wait_queue,
			READ_ONCE(frame_ready_mask) != 0 ||
			!READ_ONCE(frame_dma_running),
			HZ * 2);
		if (down_interruptible(&pci_pango->_sem))
			return -ERESTARTSYS;
		if (wait_ret < 0) {
			result = -ERESTARTSYS;
			break;
		}
		if (wait_ret == 0) {
			frame_timeout_count++;
			printk_ratelimited(KERN_ERR
				"frame DMA timeout #%lu: running=%u irq=%lu ready=0x%lx; check the 2026-08 full-frame FPGA bitstream and MSI Enable+\n",
				frame_timeout_count, frame_dma_running ? 1U : 0U,
				frame_irq_count, frame_ready_mask);
			result = -ETIMEDOUT;
			break;
		}

		spin_lock_irqsave(&frame_state_lock, flags);
		if (!frame_ready_mask) {
			spin_unlock_irqrestore(&frame_state_lock, flags);
			result = -ENODATA;
			break;
		}

		if (!frame_selected_valid) {
			if ((frame_ready_mask & 0x3UL) == 0x3UL)
				ready_index = (frame_buffer_seq[0] <= frame_buffer_seq[1]) ?
					      0U : 1U;
			else
				ready_index = (frame_ready_mask & 0x1UL) ? 0U : 1U;
			frame_selected_index = ready_index;
			frame_selected_valid = true;
		} else {
			ready_index = frame_selected_index;
		}

		ready_frame = frame_data_buf[ready_index];
		msi_operation.msi_flag = 1;
		msi_operation.frame_index = ready_index;
		msi_operation.frame_size = frame_buffer_size;
		msi_operation.frame_seq = frame_buffer_seq[ready_index];
		spin_unlock_irqrestore(&frame_state_lock, flags);

		if (!ready_frame) {
			result = -ENODATA;
			break;
		}
		if (arg != 0UL &&
		    copy_to_user((void __user *)arg, ready_frame, frame_buffer_size))
			result = -EFAULT;
		break;
	}

	case PCI_UMAP_ADDR_CMD:
		frame_dma_stop();
		free_capture_buffers();
		break;

	case PCI_GET_MSI_FLAG:
	{
		unsigned long flags;
		MSI_OPERATION current_msi;

		spin_lock_irqsave(&frame_state_lock, flags);
		current_msi = msi_operation;
		if (!frame_selected_valid)
			current_msi.msi_flag = 0;
		spin_unlock_irqrestore(&frame_state_lock, flags);

		if (copy_to_user((MSI_OPERATION *)arg, &current_msi,
				 sizeof(current_msi)))
			result = -EFAULT;
		break;
	}

	case PCI_CLEAR_MSI_FLAG:
	{
		unsigned long flags;
		unsigned int release_index;

		spin_lock_irqsave(&frame_state_lock, flags);
		if (!frame_selected_valid) {
			spin_unlock_irqrestore(&frame_state_lock, flags);
			result = -ENODATA;
			break;
		}

		release_index = frame_selected_index;
		frame_ready_mask &= ~(1UL << release_index);
		frame_selected_valid = false;
		msi_operation.msi_flag = 0;
		spin_unlock_irqrestore(&frame_state_lock, flags);

		/* Userspace has cloned the frame; FPGA may now reuse this slot. */
		wmb();
		frame_reg_write(FRAME_RELEASE_OFFSET, 1U << release_index);
		dma_mmio_posted_write_flush(&pci_info);
		break;
	}

	case PCI_PERFORMANCE_START_CMD:
		spin_lock(&performance_config.addr.lock);
		if (copy_from_user(&performance_operation, (PERFORMANCE_OPERATION *)arg,
				   sizeof(PERFORMANCE_OPERATION))) {
			spin_unlock(&performance_config.addr.lock);
			result = -EFAULT;
			break;
		}
		performance_config.addr.data_buf = dma_alloc_coherent(&op_dev->dev,
							      DMA_MAX_PACKET_SIZE * 10 * 2,
							      &performance_config.addr.addr,
							      GFP_KERNEL);
		for (i = 0; i < (DMA_MAX_PACKET_SIZE * 10) / 16; i++) {
			memset(dest_buf, 0, sizeof(dest_buf));
			temp_data = i;
			dest_buf[0] = (unsigned char)temp_data;
			dest_buf[1] = (unsigned char)(temp_data >> 8);
			dest_buf[2] = (unsigned char)(temp_data >> 16);
			dest_buf[3] = (unsigned char)(temp_data >> 24);
			memcpy(performance_config.addr.data_buf + i * 16, dest_buf, 16);
		}

		performance_config.addr.addr_size =
			((performance_config.addr.addr >> 32) > 0) ? 1 : 0;
		performance_config.cmd.data.length = performance_operation.current_len - 1;
		performance_config.cmd.data.addr_type = performance_config.addr.addr_size;
		performance_config.cmd.data.op_type = performance_operation.cmd;
		set_dma_w_r(performance_config.cmd.value, pci_pango);
		set_dma_addr(&performance_config.addr, pci_pango);
		printk("addr_size = %d;     current_len = %d(dw)\n",
		       (performance_config.addr.addr_size) ? 64 : 32,
		       performance_operation.current_len);
		printk("operation cmd = 0x%02x; dma_addr  = 0x%llx \n",
		       performance_operation.cmd, performance_config.addr.addr);
		spin_unlock(&performance_config.addr.lock);
		break;

	case PCI_PERFORMANCE_END_CMD:
		spin_lock(&performance_config.addr.lock);
		if (!memcmp(performance_config.addr.data_buf,
			    performance_config.addr.data_buf + DMA_MAX_PACKET_SIZE * 10,
			    DMA_MAX_PACKET_SIZE * 10))
			performance_operation.cmp_flag = 1;
		else
			performance_operation.cmp_flag = 0;

		if (copy_to_user((PERFORMANCE_OPERATION *)arg, &performance_operation,
				 sizeof(PERFORMANCE_OPERATION))) {
			spin_unlock(&performance_config.addr.lock);
			result = -EFAULT;
			break;
		}
		dma_free_coherent(&op_dev->dev, DMA_MAX_PACKET_SIZE * 10 * 2,
				  performance_config.addr.data_buf,
				  performance_config.addr.addr);
		performance_config.addr.data_buf = NULL;
		spin_unlock(&performance_config.addr.lock);
		break;

	default:
		result = -EINVAL;
		break;
	}

	up(&pci_pango->_sem);
	return result;
}


int pango_cdev_open(struct inode *inode, struct file *filp)
{
	
	return 0;
}

int pango_cdev_release(struct inode *inode, struct file *filp)
{
	LOG("pango_cdev_release.\n");
	
	return 0;
}


static int set_dma_mask(struct pci_dev *pdev)
{
    // if (!dma_set_mask(&pdev->dev, DMA_BIT_MASK(64))) {
    //     dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(64));
    //     printk("Using 64-bit DMA mask\n");
    // } 
	if (!dma_set_mask(&pdev->dev, DMA_BIT_MASK(32))) {
        dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
        printk("Using 32-bit DMA mask\n");
    } else {
        printk("No suitable DMA mask available\n");
        return -EINVAL;
    }
    return 0;
}



int pci_driver_probe(struct pci_dev *dev, const struct pci_device_id *device_id)
{
	int result;
	unsigned long bar_address;

	LOG("pci_driver_probe.\n");
	LOG("dev vendor : 0x%x, device : 0x%x\n", dev->vendor, dev->device);

	result = pci_enable_device(dev);
	if (result)
		return result;

	result = set_dma_mask(dev);
	if (result)
		goto fail_disable_device;

	op_dev = dev;
	pci_set_master(dev);
	ReadConfig(dev);

	result = pci_request_region(dev, pci_info._pango_pci_driver._pci_bar,
				    PCI_DRIVER_DEV_NAME);
	if (result)
		goto fail_clear_master;

	pci_info._pango_pci_driver._pci_io_size =
		pci_resource_len(dev, pci_info._pango_pci_driver._pci_bar);
	bar_address =
		pci_resource_start(dev, pci_info._pango_pci_driver._pci_bar);
	pci_info._pango_pci_driver._pci_io =
		ioremap(bar_address, pci_info._pango_pci_driver._pci_io_size);
	if (!pci_info._pango_pci_driver._pci_io) {
		result = -ENOMEM;
		goto fail_release_region;
	}

	pci_info._pango_pci_driver._pci_io_buff =
		kmalloc(pci_info._pango_pci_driver._pci_io_size, GFP_KERNEL);
	if (!pci_info._pango_pci_driver._pci_io_buff) {
		result = -ENOMEM;
		goto fail_iounmap;
	}

	result = pci_alloc_irq_vectors(dev, 1, 1, PCI_IRQ_MSI);
	if (result < 0) {
		printk(KERN_ERR "hardware MSI is required: %d\n", result);
		goto fail_free_io_buffer;
	}

	pango_msi_irq = pci_irq_vector(dev, 0);
	result = request_irq(pango_msi_irq, pango_frame_msi_handler, 0,
			     PCI_DRIVER_DEV_NAME, dev);
	if (result)
		goto fail_free_irq_vectors;

	pango_msi_enabled = true;
	pci_set_drvdata(dev, &pci_info);
	printk(KERN_INFO "Pango frame DMA MSI vector %d enabled\n",
	       pango_msi_irq);
	return 0;

fail_free_irq_vectors:
	pango_msi_irq = -1;
	pci_free_irq_vectors(dev);
fail_free_io_buffer:
	kfree(pci_info._pango_pci_driver._pci_io_buff);
	pci_info._pango_pci_driver._pci_io_buff = NULL;
fail_iounmap:
	iounmap(pci_info._pango_pci_driver._pci_io);
	pci_info._pango_pci_driver._pci_io = NULL;
fail_release_region:
	pci_release_region(dev, pci_info._pango_pci_driver._pci_bar);
fail_clear_master:
	pci_clear_master(dev);
fail_disable_device:
	pci_disable_device(dev);
	op_dev = NULL;
	return result;
}

void pci_driver_remove(struct pci_dev *dev)
{
	if (pci_info._pango_pci_driver._pci_io)
		frame_dma_stop();

	if (pango_msi_irq >= 0) {
		free_irq(pango_msi_irq, dev);
		pango_msi_irq = -1;
	}
	if (pango_msi_enabled) {
		pci_free_irq_vectors(dev);
		pango_msi_enabled = false;
	}

	free_capture_buffers();
	kfree(pci_info._pango_pci_driver._pci_io_buff);
	pci_info._pango_pci_driver._pci_io_buff = NULL;

	if (pci_info._pango_pci_driver._pci_io) {
		iounmap(pci_info._pango_pci_driver._pci_io);
		pci_info._pango_pci_driver._pci_io = NULL;
	}

	pci_release_region(dev, pci_info._pango_pci_driver._pci_bar);
	pci_clear_master(dev);
	pci_disable_device(dev);
	op_dev = NULL;
	LOG("pci_driver_remove.\n");
}


int init_pango_cdev(struct cdev *pango_cdev)
{
	int result = 0;
	
	LOG("init_pango_cdev.\n");
	
	result = alloc_chrdev_region(&pci_dev_info._dev, pci_dev_info._dev_firstminor, pci_dev_info._dev_count, pci_dev_info._dev_name);
	
	LOG("*********alloc_chrdev_region, result : %d\n", result);
	
	if(result < 0)
	{
		return result;
	}
	
	cdev_init(pango_cdev, &pango_cdev_fops);
	
	pango_cdev->owner = THIS_MODULE;
	pango_cdev->ops   = &pango_cdev_fops;
	
	result = cdev_add(pango_cdev, pci_dev_info._dev, pci_dev_info._dev_count);							
	
	LOG("init pango_cdev result : %d\n", result);
	
	return result;
}

void exit_pango_cdev(struct cdev *pango_cdev)
{
	cdev_del(pango_cdev);
	
	unregister_chrdev_region(pci_dev_info._dev, pci_dev_info._dev_count);
	
	LOG("exit cdev.\n");
}

int init_pango_pci_driver(struct pci_driver *pango_pci_driver)
{
	int result = 0;
	
	LOG("init_pango_pci_driver.\n");
	
	result = pci_register_driver(pango_pci_driver);						
	
	LOG("*********pci_register_driver, result : %d\n", result);
	
	return result;
}

void exit_pango_pci_driver(struct pci_driver *pango_pci_driver)
{
	pci_unregister_driver(pango_pci_driver);
	
	LOG("exit pci pango driver.\n");
}


int init_pango_cdev_class(struct class *pango_cdev_class)
{
	struct device *device;

	pci_info._cdev_class =
		class_create(THIS_MODULE, pci_dev_info._dev_name);
	if (IS_ERR(pci_info._cdev_class)) {
		int result = PTR_ERR(pci_info._cdev_class);
		pci_info._cdev_class = NULL;
		return result;
	}

	device = device_create(pci_info._cdev_class, NULL, pci_dev_info._dev,
			       NULL, "%s", pci_dev_info._dev_name);
	if (IS_ERR(device)) {
		int result = PTR_ERR(device);
		class_destroy(pci_info._cdev_class);
		pci_info._cdev_class = NULL;
		return result;
	}
	return 0;
}

void exit_pango_cdev_class(struct class *pango_cdev_class)
{
	if (!pci_info._cdev_class)
		return;
	device_destroy(pci_info._cdev_class, pci_dev_info._dev);
	class_destroy(pci_info._cdev_class);
	pci_info._cdev_class = NULL;
}

int __init init_pci_pango(void)
{
	int result;

	LOG("init_pci_pango.\n");
	sema_init(&pci_info._sem, 1);
	init_waitqueue_head(&dma_wait_queue);
	spin_lock_init(&frame_state_lock);
	spin_lock_init(&dma_info.addr_r.lock);
	spin_lock_init(&dma_info.addr_w.lock);
	spin_lock_init(&performance_config.addr.lock);
	frame_ready_mask = 0;
	frame_dma_running = false;
	frame_selected_valid = false;
	msi_operation.msi_flag = 0;

	result = init_pango_cdev(&pci_info._cdev);
	if (result)
		return result;

	result = init_pango_pci_driver(
		&pci_info._pango_pci_driver._pci_driver);
	if (result)
		goto fail_cdev;

	result = init_pango_cdev_class(pci_info._cdev_class);
	if (result)
		goto fail_pci_driver;

	return 0;

fail_pci_driver:
	exit_pango_pci_driver(&pci_info._pango_pci_driver._pci_driver);
fail_cdev:
	exit_pango_cdev(&pci_info._cdev);
	return result;
}

void __exit exit_pci_pango(void)
{
	exit_pango_cdev_class(pci_info._cdev_class);
	exit_pango_pci_driver(&pci_info._pango_pci_driver._pci_driver);
	exit_pango_cdev(&pci_info._cdev);
	LOG("exit_pci_pango.\n");
}

module_init(init_pci_pango);
module_exit(exit_pci_pango);

MODULE_AUTHOR("Pango, wzq.");                                                        
MODULE_DESCRIPTION("Pango pci driver");                                               
MODULE_LICENSE("GPL v2");                                                             
MODULE_ALIAS("pango pci driver");                                                    


