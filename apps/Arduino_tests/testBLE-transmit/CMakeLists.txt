include_directories(
  ${CMAKE_SOURCE_DIR}/apps/Arduino_tests/testBLE-transmit
  ${CMAKE_SOURCE_DIR}/libs/Arduino_lib/core_libs/inc
  ${CMAKE_SOURCE_DIR}/libs/Arduino_lib/separate_libs/inc
  ${CMAKE_SOURCE_DIR}/libs/Arduino_lib/SX1280LT_lib/inc
  ${CMAKE_SOURCE_DIR}/libs/sis_aes_lib/inc
   .
)

add_application(testBLE-transmit testBLE-transmit.cpp LABELS "arduino" LIBS "sis_aes" "Arduino_SX1280LT")
