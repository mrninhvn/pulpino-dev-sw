set(SOURCES
    src/sis_aes.c
    ${CMAKE_SOURCE_DIR}/apps/ghash/ghash_ctmul32.c
    )

set(HEADERS
    inc/sis_aes.h
    )

include_directories(inc/)
include_directories(../string_lib/inc)
include_directories(${CMAKE_SOURCE_DIR}/apps/ghash)

add_library(sis_aes STATIC ${SOURCES} ${HEADERS})
