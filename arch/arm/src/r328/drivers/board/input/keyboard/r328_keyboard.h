#ifndef SUNXI_KEYBOARD_H
#define SUNXI_KEYBOARD_H

int lradc_irq_callback(unsigned int irq_status, unsigned int key_vol);
int sunxi_keyboard_init(void);

#endif
