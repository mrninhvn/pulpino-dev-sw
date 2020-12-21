set(SOURCES
    src/SX128XLT_BLE.cpp
    src/SX128XLT.cpp
    )

set(HEADERS
    inc/SX128XLT_BLE.h
    inc/SX128XLT_Definitions.h
    inc/SX128XLT.h
    )

include_directories(inc/)
include_directories(../core_libs/inc)
include_directories(../../sys_lib/inc)

add_library(Arduino_SX1280LT STATIC ${SOURCES} ${HEADERS})


