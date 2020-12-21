set(SOURCES
    src/sensor.cpp
    src/OneWire.cpp
    )

set(HEADERS
    inc/sensor.h
    inc/OneWire.h
    inc/OneWire_direct_gpio.h
    inc/OneWire_direct_regtype.h
    )

include_directories(inc/)
include_directories(../core_libs/inc)
include_directories(../../sys_lib/inc)

add_library(Arduino_sensor STATIC ${SOURCES} ${HEADERS})


