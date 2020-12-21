include_directories(
  ${CMAKE_SOURCE_DIR}/apps/ghash
   .
)
set(aes_test_SOURCES aes_test.c ${CMAKE_SOURCE_DIR}/apps/ghash/ghash_ctmul32.c)
add_application(aes_test "${aes_test_SOURCES}")
