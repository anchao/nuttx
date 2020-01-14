## ChangeLog

### Nuttx-8.2-r328-0.1 - December 05 2019
**System**
- Added support sdk build env.
- Added support image pack, secure and non-secure.
- Added support prebuilt toolchain.
- Added support Allwinner R328 SOC.
- Added support R328 dual core bringup.
- Added support Nsh， can run to uart console, but should set new line mode.

**Known Issues**
- ostest failed if enable smp
- ostest failed if enable debug options.

**TODO**
- undo os header wrapper.
- undo common arch code modify.
- more r328 driver support.

### Nuttx-8.2-r328-0.3 - December 23 2019
**System**
- Added support melis toolchain, support cortex-a profile
- Added support generic GIC driver.
- Added support ccmu/gpio/dma/spi/lradc/gpadc/audiocodec/sido/i2c/watchdog/pwm driver
- Added support versioned sdk.

**Known Issues**
- system maybe crashed if enable smp
- ostest failed if enable smp.

**TODO**
- fixed smp stability.
- support more bsp drivers.
- support wifi/bt communication.
