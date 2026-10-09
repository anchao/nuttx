############################################################################
# net/bt/compat/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
############################################################################

target_sources(net PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/src/kernel/queue.c
  ${CMAKE_CURRENT_LIST_DIR}/src/kernel/timeout.c
  ${CMAKE_CURRENT_LIST_DIR}/src/kernel/mem_slab.c
  ${CMAKE_CURRENT_LIST_DIR}/src/sections/defines.c
  ${CMAKE_CURRENT_LIST_DIR}/src/subsys/fs/fs.c
  ${CMAKE_CURRENT_LIST_DIR}/src/drivers/bluetooth/hci/hci_driver.c
)
