############################################################################
# net/bt/crypto/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
############################################################################

if(CONFIG_BT_CRYPTO)
  target_sources(net PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/bt_crypto.c
    ${CMAKE_CURRENT_LIST_DIR}/bt_crypto_psa.c
  )

  target_include_directories(net PRIVATE ${CMAKE_CURRENT_LIST_DIR})
endif()
