include_directories(
  ${CMAKE_SOURCE_DIR}/apps/aes_test
  ${CMAKE_SOURCE_DIR}/libs/sis_aes_lib/inc
  ${CMAKE_SOURCE_DIR}/libs/sis_aes_lib/test_vector
   .
)
add_application(sis_aes_test sis_aes_test.c LABEL "sis_aes_test" LIBS "sis_aes")
