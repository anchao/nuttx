############################################################################
# net/bt/common/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
############################################################################

if(CONFIG_BT)
  set(BT_COMMON_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/addr.c
    ${CMAKE_CURRENT_LIST_DIR}/dummy.c
    ${CMAKE_CURRENT_LIST_DIR}/bt_str.c
    ${CMAKE_CURRENT_LIST_DIR}/hci_pkt.c
    ${CMAKE_CURRENT_LIST_DIR}/hci_lockstep.c
  )

  if(CONFIG_BT_HCI_SET_PUBLIC_ADDR)
    list(APPEND BT_COMMON_SRCS ${CMAKE_CURRENT_LIST_DIR}/hci_public_addr.c)
  endif()

  if(CONFIG_BT_RPA)
    list(APPEND BT_COMMON_SRCS ${CMAKE_CURRENT_LIST_DIR}/rpa.c)
  endif()

  if(CONFIG_BT_PRIVATE_SHELL)
    list(APPEND BT_COMMON_SRCS ${CMAKE_CURRENT_LIST_DIR}/bt_shell_private.c)
  endif()

  target_sources(net PRIVATE ${BT_COMMON_SRCS})
endif()
