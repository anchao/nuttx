############################################################################
# net/bt/compat/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
############################################################################

# Kernel compat sources
set(BT_COMPAT_KERNEL_SRCS
  queue.c
  timeout.c
  mem_slab.c
)

# Support module sources
set(BT_COMPAT_SUPPORT_SRCS
  fs.c
)

# Iterable section definitions
set(BT_COMPAT_SECTION_SRCS
  defines.c
)

# Compile as part of the net library
set_property(GLOBAL APPEND PROPERTY NET_SRCS
  ${BT_COMPAT_KERNEL_SRCS}
  ${BT_COMPAT_SUPPORT_SRCS}
  ${BT_COMPAT_SECTION_SRCS}
)

# VPATH equivalents for CMake
target_sources(net PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/src/kernel/queue.c
  ${CMAKE_CURRENT_LIST_DIR}/src/kernel/timeout.c
  ${CMAKE_CURRENT_LIST_DIR}/src/kernel/mem_slab.c
  ${CMAKE_CURRENT_LIST_DIR}/src/sections/defines.c
  ${CMAKE_CURRENT_LIST_DIR}/src/subsys/fs/fs.c
  ${CMAKE_CURRENT_LIST_DIR}/src/drivers/bluetooth/hci/hci_driver.c
)
