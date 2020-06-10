## ChangeLog

### Nuttx-9.0-r328-0.1 - June 10 2020
**System**
- Update nuttx to commit id (124e6ee53d sched/sched/sched_releasetcb.c)
- Update apps to commit id (984b80d7 netlib.h: Fix nxstyle complaints)
- Add mini smp config to debug stablity issues.
- Add opus encode/decode
**TODO**
- SMP stability issues still exist after kernel upgrade.
- Support spinand flash
- Support audio player
- Updated btmesh

### Nuttx-8.2-r328-0.8 - April 03 2020
**System**
- Added io tools rwcheck and rwspeed
- Added breakpoint support
- Updated wifi driver support softap mode
- Updated dma driver fix transport issue
- Updated sdmmc driver set io driving level to 3

**Known Issues**
- ostest run under SMP mode hang at sched/irq/irq_csection.c
- bluetooth avrcp/a2dpsink not work well
- wifi softap mode failed under nuttx-8.2

**TODO**
- Fixed smp stability issues
- Updated BTmesh function
- Support audio player

### Nuttx-8.2-r328-0.7 - March 06 2020
**System**
- Added bluetooth ble stack and xr829 hci driver
- Added bluetooth avrcp/a2dpsink profiles support
- Added xt25f128 spinor flash support
- Added dmic support
- Updated littlefs filesystem patch from xiaomi
- Updated alsa and adb library
- Reverted some patches according to xiao xiang's suggestion

**Known Issues**
- ostest run under SMP mode hang at sched/irq/irq_csection.c
- bluetooth avrcp/a2dpsink not work well
- ls command may return stat failed result

**TODO**
- Fixed smp stability issues
- Support bluetooth mesh stack
- Support audio enc/dec framework
- Support more thirdparty component

### Nuttx-8.2-r328-0.5 - January 14 2020
**System**
- Added CrytoEngine/I2C/Key/AC107/USB/SDMMC/ driver support
- Added neon support for VFP
- Added usb udc driver support and adb library
- Added spinor support and auto mount procfs/littlefs default
- Added XR829 driver and enable network connectivity default
- Added more iobox commands like cd/ls...
- Reverted some modify according to xiao xiang's suggestion
- Updated audio core and alsa library

**Known Issues**
- SMP not work well under cortex-a serial, some stability issues should be fixed.
- PS command show cpu0 idle stack size zero seems a known issue listed in TODO files.

**TODO**
- Fixed smp stability issues
- Support ble communication
- Support more thirdparty components

### Nuttx-8.2-r328-0.3 - December 23 2019
**System**
- Added support melis toolchain, support cortex-a profile
- Added support generic GIC driver.
- Added support ccmu/gpio/dma/spi/lradc/gpadc/audiocodec/sido/i2c/watchdog/pwm driver
- Added support versioned sdk.

**Known Issues**
- System maybe crashed if enable smp
- Ostest failed if enable smp.

**TODO**
- Fixed smp stability issues
- Support more bsp drivers
- Support wifi/bt communication

### Nuttx-8.2-r328-0.1 - December 05 2019
**System**
- Added support sdk build env.
- Added support image pack, secure and non-secure.
- Added support prebuilt toolchain.
- Added support Allwinner R328 SOC.
- Added support R328 dual core bringup.
- Added support Nsh， can run to uart console, but should set new line mode.

**Known Issues**
- Ostest failed if enable smp
- Ostest failed if enable debug options.

**TODO**
- Undo os header wrapper.
- Undo common arch code modify.
- More r328 driver support.