include_directories(.)
add_definitions(-DBR_MUL32)
set(ghash_test_SOURCES ghash_ctmul.c ghash_ctmul64.c ghash_ctmul32.c main.c)
add_application(ghash_test "${ghash_test_SOURCES}")
