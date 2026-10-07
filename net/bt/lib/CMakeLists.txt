############################################################################
# net/bt/lib/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
############################################################################

if(CONFIG_BT)
  set(BT_LIB_SRCS)

  if(CONFIG_BT_EAD)
    list(APPEND BT_LIB_SRCS ${CMAKE_CURRENT_LIST_DIR}/ead.c)
  endif()

  if(CONFIG_LIBSBC)
    list(APPEND BT_LIB_SRCS ${CMAKE_CURRENT_LIST_DIR}/sbc.c)
  endif()

  if(BT_LIB_SRCS)
    target_sources(net PRIVATE ${BT_LIB_SRCS})
  endif()

  target_include_directories(net PRIVATE ${CMAKE_CURRENT_LIST_DIR})
endif()
