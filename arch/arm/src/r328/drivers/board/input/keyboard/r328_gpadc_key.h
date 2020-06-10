#ifndef SUNXI_KEYBOARD_H
#define SUNXI_KEYBOARD_H

int gpadc_irq_callback(unsigned int channel, int irq_status, unsigned int key_vol);
int sunxi_gpadc_key_init(void);

#endif
