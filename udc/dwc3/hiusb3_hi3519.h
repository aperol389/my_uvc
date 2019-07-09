#ifndef __HIUSB3_HI3519_H__
#define __HIUSB3_HI3519_H__


extern void set_usb_soft_connect_ctrl(uint32_t value);
extern uint32_t get_usb_soft_connect_ctrl(void);
extern void usb_soft_connect(void);
extern void usb_soft_disconnect(void);
extern void usb_soft_connect_reset(void);
extern uint32_t get_usb_soft_connect_state(void);
extern int usb3_ep_set_config(const struct usb_endpoint_descriptor *ep_desc, unsigned int usb_speed);
extern void usb3_phy_suspend_config(unsigned int usb_speed, unsigned int suspend_en);

extern int hiusb3_init(void);

#endif
