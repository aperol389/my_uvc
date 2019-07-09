#include <linux/init.h>
#include <linux/timer.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/spinlock.h>
#include <asm/byteorder.h>
#include <linux/io.h>
//#include <asm/system.h>
#include <asm/unaligned.h>
#include <mach/io.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/video.h>

enum usb_connect_state
{
	USB_STATE_DISCONNECT,
	USB_STATE_CONNECT,
	USB_STATE_CONNECT_RESET,
};

#define USB3_BASE_ADDRESS 			(0x04110000)


void __iomem *reg_base_va = NULL;
#define IO_ADDRESS_VERIFY(x) (reg_base_va + ((x)-(USB3_BASE_ADDRESS)))

#define DCTL 						(IO_ADDRESS_VERIFY(USB3_BASE_ADDRESS  + 0xC704))
//#define DCTL 						(IO_ADDRESS_VERIFY_VERIFY(USB3_BASE_ADDRESS  + 0xC704))
#define RUN_STOP					(1 << 31)
#define SOFT_CORE_RESET				(1 << 30)
#define SFTDISCON					(1 << 1)


#define DEPCMDPAR0 					(0xC808)
#define DEPCMDPAR1 					(0xC804)
#define DEPCMDPAR2 					(0xC800)

#define USB3_DALEPENA  				(IO_ADDRESS_VERIFY(USB3_BASE_ADDRESS  + 0xC720))

#define GUSB2PHYCFGN  				(IO_ADDRESS_VERIFY(USB3_BASE_ADDRESS  + 0xC200))
#define USB2_PHY_SUSPEND_EN			(1 << 6)

#define GUSB3PIPECTLN   			(IO_ADDRESS_VERIFY(USB3_BASE_ADDRESS  + 0xC2C0))
#define USB3_PHY_SUSPEND_EN			(1 << 17)



typedef union dctl_data {
		/** raw register data */
	uint32_t d32;//32位
		/** register bits */
	struct {//低位在前[0]...[32]
		unsigned reserved:1;//长度为1
		unsigned test_control:4;
		unsigned ulstchngraq:4;
		unsigned accept_u1_enable:1;
		unsigned initiate_u1_enable:1;
		unsigned accept_u2_enable:1;
		unsigned initiate_u2_enable:1;
		unsigned reserved2:3;
		unsigned controller_save_state :1;
		unsigned controller_restore_state:1;
		unsigned l1_hibernation_en:1;
		unsigned keepconnect:1;
		unsigned reserved3:3;
		unsigned appl1res:1;
		unsigned hird_threshold:5;
		unsigned reserved4:1;
		unsigned soft_core_reset:1;
		unsigned run_stop:1;
	} b;
} dctl_data_t;

volatile static uint32_t soft_connect_disable_ctrl	= 0;

void dwc3_mdelay(uint32_t msecs)
{
	if (in_interrupt())
		mdelay(msecs);
	else
		msleep(msecs);
}

int hiusb3_init(void)
{
       int ret;

    reg_base_va = ioremap_nocache((unsigned long)USB3_BASE_ADDRESS, (unsigned long)0xD000);
    if (!reg_base_va)
    {   
        printk("Kernel: ioremap ssp base failed!\n");
        return -ENOMEM;
    }   


    return 0;
}

EXPORT_SYMBOL(hiusb3_init);

void set_usb_soft_connect_ctrl(uint32_t value)
{
	soft_connect_disable_ctrl = value; 	
}
EXPORT_SYMBOL(set_usb_soft_connect_ctrl);

uint32_t get_usb_soft_connect_ctrl(void)
{
	return soft_connect_disable_ctrl; 	
}
EXPORT_SYMBOL(get_usb_soft_connect_ctrl);


void usb_soft_connect(void)
{
	unsigned long flags;
	
	if(soft_connect_disable_ctrl)
		return;
	
	local_irq_save(flags);

	dctl_data_t dctl_reg;
	dctl_reg.d32 = hi_readl(DCTL);
	
	//2.如果软件要在软断开或者检测到断开事件后重新启动连接，
	//则需要往此比特写 1 之前将 DCTL[8：5]设置成 5； 
	dctl_reg.b.ulstchngraq = 5;
	hi_writel(dctl_reg.d32, DCTL);
	
	dctl_reg.d32 = hi_readl(DCTL);
	dctl_reg.b.run_stop = 1;
	hi_writel(dctl_reg.d32, DCTL);
	
	dwc3_mdelay(5);	
	//printk("%s +%d %s:usb connect OK\n", __FILE__,__LINE__,__func__);

	local_irq_restore(flags);		
}
EXPORT_SYMBOL(usb_soft_connect);

void usb_soft_disconnect(void)
{
	unsigned long flags;
	
	if(soft_connect_disable_ctrl)
		return;
	
	local_irq_save(flags);
	
	dctl_data_t dctl_reg;
	dctl_reg.d32 = hi_readl(DCTL);
	dctl_reg.b.run_stop = 0;
	hi_writel(dctl_reg.d32, DCTL);
	
	dwc3_mdelay(5);
		
	//printk("%s +%d %s:usb disconnect OK\n", __FILE__,__LINE__,__func__);

	local_irq_restore(flags);
}
EXPORT_SYMBOL(usb_soft_disconnect);

//USB软断开重连
void usb_soft_connect_reset(void)
{
	unsigned long flags;
	
	if(soft_connect_disable_ctrl)
		return;
	
	local_irq_save(flags);
	
	dctl_data_t dctl_reg;
	
	dctl_reg.d32 = hi_readl(DCTL);
	dctl_reg.b.run_stop = 0;
	hi_writel(dctl_reg.d32, DCTL);
	dwc3_mdelay(10);
	
	dctl_reg.d32 = hi_readl(DCTL);
	
	//2.如果软件要在软断开或者检测到断开事件后重新启动连接，
	//则需要往此比特写 1 之前将 DCTL[8：5]设置成 5； 
	dctl_reg.b.ulstchngraq = 5;
	hi_writel(dctl_reg.d32, DCTL);
	
	dctl_reg.d32 = hi_readl(DCTL);
	dctl_reg.b.run_stop = 1;
	hi_writel(dctl_reg.d32, DCTL);
		
	//printk("%s +%d %s:usb connect reset OK\n", __FILE__,__LINE__,__func__);

	local_irq_restore(flags);
}
EXPORT_SYMBOL(usb_soft_connect_reset);

uint32_t get_usb_soft_connect_state(void)
{
	unsigned long flags;
	
	local_irq_save(flags);

	int reg;
	reg = hi_readl(DCTL);
	if(reg & RUN_STOP){
		local_irq_restore(flags);
		return USB_STATE_CONNECT;
	}else{
		local_irq_restore(flags);
		return USB_STATE_DISCONNECT;
	}			
}
EXPORT_SYMBOL(get_usb_soft_connect_state);

#define UE_GET_DIR(a)	((a) & 0x80)
#define UE_DIR_IN	0x80
#define UE_DIR_OUT	0x00
#define UE_ADDR		0x0f
#define UE_GET_ADDR(a)	((a) & UE_ADDR)

int usb3_ep_set_config(const struct usb_endpoint_descriptor *ep_desc, unsigned int usb_speed)
{
#if 1
	int retval;
	int num, dir;
	u16 size;
	u8 eptype;
	unsigned long flags;
	u32 depcfg0, depcfg1, depcfg2 = 0;
	u32 dalepena = 0, depcmd = 0;
	volatile u32 addr;
	
	
	if (ep_desc->bDescriptorType != USB_DT_ENDPOINT) {
		//DWC_WARN("%s, bad ep or descriptor\n", __func__);
		return -EINVAL;
	}

	/* Check FIFO size? */
	if (!ep_desc->wMaxPacketSize) {
		//DWC_WARN("%s, bad maxpacket\n", __func__);
		return -ERANGE;
	}
	
	local_irq_save(flags);
	
	num = UE_GET_ADDR(ep_desc->bEndpointAddress);
	dir = UE_GET_DIR(ep_desc->bEndpointAddress);
	size = le16_to_cpu(ep_desc->wMaxPacketSize) & 0x7FF;
	eptype = ep_desc->bmAttributes & 0x03;
	
	
	dalepena = hi_readl(USB3_DALEPENA);
	if(dir == UE_DIR_OUT)
		dalepena |= 1 << (num * 2);
	else 
		dalepena |= 1 << (num * 2 + 1);
	
	hi_writel(dalepena, USB3_DALEPENA);
	
#if 0	
	addr = USB3_BASE_ADDRESS + DEPCMDPAR2 + num * 0x10;
	hi_writel(depcfg2, IO_ADDRESS(addr));
	
	addr = USB3_BASE_ADDRESS + DEPCMDPAR1 + num * 0x10;
	depcfg1 = hi_readl(IO_ADDRESS(addr));
	depcfg1 = num << DWC_EPCFG1_EP_NUM_SHIFT;
	if (dir == UE_DIR_IN)
		depcfg1 |= DWC_EPCFG1_EP_DIR_BIT;

	depcfg1 |= DWC_EPCFG1_XFER_CMPL_BIT;
	depcfg1 |= DWC_EPCFG1_XFER_IN_PROG_BIT;
	depcfg1 |= DWC_EPCFG1_XFER_NRDY_BIT;

	depcfg1 |= (ep_desc->bInterval - 1) << DWC_EPCFG1_BINTERVAL_SHIFT;

	//depcfg1 |= DWC_EPCFG1_STRM_CAP_BIT;
	
	//depcfg1 |= DWC_EPCFG1_EBC_MODE_BIT;

	hi_writel(depcfg1, IO_ADDRESS(addr));
	
	
	addr = USB3_BASE_ADDRESS + DEPCMDPAR0 + num * 0x10;
	depcfg0 = hi_readl(IO_ADDRESS(addr));
	
	depcfg0 |= eptype << DWC_EPCFG0_EPTYPE_SHIFT;
	depcfg0 |= size << DWC_EPCFG0_MPS_SHIFT;
	
	if (dir == UE_DIR_IN){
		depcfg0 |= 0x01 << DWC_EPCFG0_TXFNUM_SHIFT;	
	}
	
	//depcfg0 |= 0x00 << DWC_EPCFG0_BRSTSIZ_SHIFT;
	
	
	hi_writel(depcfg0, IO_ADDRESS(addr));
	
#endif	
	local_irq_restore(flags);	
	printk("%s +%d %s\n", __FILE__,__LINE__,__func__);
#endif	

	return 0;	
}

EXPORT_SYMBOL(usb3_ep_set_config);

//USB 3.0 USB 2.0 PHY 挂起设置
void usb3_phy_suspend_config(unsigned int usb_speed, unsigned int suspend)
{
	unsigned long flags;
	int reg;
	
	local_irq_save(flags);
	
	if(usb_speed == USB_SPEED_SUPER){
		reg = hi_readl(GUSB3PIPECTLN);
		if(suspend){
			//printk("%s +%d %s:usb3 phy suspend\n", __FILE__,__LINE__,__func__);
			reg |= (USB3_PHY_SUSPEND_EN);
		}	
		else
			reg &= ~(USB3_PHY_SUSPEND_EN);	
		
		hi_writel(reg, GUSB3PIPECTLN);
	}else{
		reg = hi_readl(GUSB2PHYCFGN);
		if(suspend){
			//printk("%s +%d %s:usb2 phy suspend\n", __FILE__,__LINE__,__func__);
			reg |= (USB2_PHY_SUSPEND_EN);
		}
		else
			reg &= ~(USB2_PHY_SUSPEND_EN);	
		
		hi_writel(reg, GUSB2PHYCFGN);
	}

	local_irq_restore(flags);	
}
EXPORT_SYMBOL(usb3_phy_suspend_config);
