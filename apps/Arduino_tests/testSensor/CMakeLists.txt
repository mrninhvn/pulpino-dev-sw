include_directories(
  ${CMAKE_SOURCE_DIR}/libs/Arduino_lib/core_libs/inc
  ${CMAKE_SOURCE_DIR}/libs/Arduino_lib/separate_libs/inc
  ${CMAKE_SOURCE_DIR}/libs/Arduino_lib/Sensor_lib/inc
   .
)

add_application(testSensor testSensor.cpp LABELS "arduino" LIBS "Arduino_sensor")
