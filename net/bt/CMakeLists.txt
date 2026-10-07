############################################################################
# net/bt/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.
#
############################################################################

if(CONFIG_BT)
  add_subdirectory(compat)
  add_subdirectory(common)
  add_subdirectory(crypto)
  add_subdirectory(lib)
  add_subdirectory(host)
  add_subdirectory(audio)
  add_subdirectory(mesh)
  add_subdirectory(services)
endif()
