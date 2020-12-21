#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sis_aes.h"
#include "aes_test.h"
#include "aes_ofb.h"
#include "aes_cbc.h"
#include "aes_ctr.h"
#include "aes_ccm.h"
#include "aes_gcm.h"

typedef enum {TEST_SUCCESS, TEST_FAILURE} status_t;
void test_aes_ecb();
void test_aes_ofb();
void test_aes_cbc();
void test_aes_ctr();
void test_aes_ccm();
void test_aes_gcm();
void test_decrypt_aes_ccm();
void test_decrypt_aes_gcm();

status_t check_result(const uint8_t *exp, const uint8_t *data, int len);
void dump_data(const uint8_t *data, int len);

int main()
{
  printf("Start AES\n");
  int test_case = 8;
  int test_pass = 0;
  int test_fail = 0;
  switch(test_case){
  case 0: test_aes_ecb(&test_pass, &test_fail); break;
  case 1: test_aes_ofb(&test_pass, &test_fail); break;
  case 2: test_aes_cbc(&test_pass, &test_fail); break;
  case 3: test_aes_ctr(&test_pass, &test_fail); break;
  case 4: test_aes_ccm(&test_pass, &test_fail); break;
  case 5: test_aes_gcm(&test_pass, &test_fail); break;
  case 6: test_decrypt_aes_ccm(&test_pass, &test_fail); break;
  case 7: test_decrypt_aes_gcm(&test_pass, &test_fail); break;
  default:
    test_aes_ecb(&test_pass, &test_fail);
    test_aes_ofb(&test_pass, &test_fail);
    test_aes_cbc(&test_pass, &test_fail);
    test_aes_ctr(&test_pass, &test_fail);
    test_aes_ccm(&test_pass, &test_fail);
    test_aes_gcm(&test_pass, &test_fail);
    test_decrypt_aes_ccm(&test_pass, &test_fail);
    test_decrypt_aes_gcm(&test_pass, &test_fail); break;
  }
  printf("Test results: PASS: %d/%d FAIL: %d/%d\n", test_pass, test_pass+test_fail, test_fail, test_pass+test_fail);
  return 0;
}
void test_decrypt_aes_gcm(int* test_pass, int* test_fail)
{
  for (int i = 0; i < 4; ++i){
    uint32_t start = get_scratchpad_ptr();
    uint8_t *c;
    int output_size = GCM_PLAIN_SIZE[i];
    c = malloc_wrap(output_size);
    memset(c, 0, output_size);
    int ret = aes_aead_gcm_decrypt(c, (uint8_t *) &gcm_key[i],
		 (uint8_t *) gcm_authtext[i], GCM_AUTH_SIZE[i],
		 (uint8_t *) gcm_out[i], output_size + GCM_TAG_SIZE,
		 (uint8_t *) gcm_iv[i], GCM_IV_SIZE[i],
		 GCM_TAG_SIZE,
		 CCM, KEY_128);
    if(check_result((uint8_t *) gcm_plaintext[i], c, GCM_PLAIN_SIZE[i]) != TEST_SUCCESS || ret != AES_OK){
      printf("Test Decrypt %2d GCM failed!\n", i);
      ++(*test_fail);
    }
    else {
      printf("Test Decrypt %2d GCM successful!\n", i);
      ++(*test_pass);
    }
    printf("Expected: ");
    dump_data((uint8_t *) gcm_plaintext[i], GCM_PLAIN_SIZE[i]);
    printf("Received: ");
    dump_data(c, GCM_PLAIN_SIZE[i]);
    free_wrap(c);
  }
}

void test_aes_gcm(int* test_pass, int* test_fail)
{
  for (int i = 0; i < 4; ++i){
    uint32_t start = get_scratchpad_ptr();
    uint8_t *c;
    int output_size = GCM_PLAIN_SIZE[i] + GCM_TAG_SIZE;
    c = malloc_wrap(output_size);
    memset(c, 0, output_size);
    aes_aead_gcm(c, (uint8_t *) &gcm_key[i],
		 (uint8_t *) gcm_authtext[i], GCM_AUTH_SIZE[i],
		 (uint8_t *) gcm_plaintext[i], GCM_PLAIN_SIZE[i],
		 (uint8_t *) gcm_iv[i], GCM_IV_SIZE[i],
		 GCM_TAG_SIZE,
		 CCM, KEY_128);
    if(check_result((uint8_t *) gcm_out[i], c, output_size) != TEST_SUCCESS){
      printf("Test %2d GCM failed!\n", i);
      ++(*test_fail);
    }
    else {
      printf("Test %2d GCM successful!\n", i);
      ++(*test_pass);
    }
    printf("Expected: ");
    dump_data((uint8_t *) gcm_out[i], output_size);
    printf("Received: ");
    dump_data(c, output_size);
    free_wrap(c);
  }
}

void test_aes_ccm(int* test_pass, int* test_fail){

  for (int i = 0; i < CCM_TEST_NUM; ++i){
    uint8_t *c;
    int o_size = tag_size_list[i] + p_size_list[i];
    c = malloc_wrap(o_size);
    memset(c, 0, tag_size_list[i] + p_size_list[i]);

    aes_aead_ccm(c, (uint8_t *) key_p_list[i],
		 (uint8_t *) ad_p_list[i], ad_size_list[i],
		 (uint8_t *) payload_p_list[i], p_size_list[i],
		 (uint8_t *) iv_p_list[i], iv_size_list[i],
		 tag_size_list[i],
		 CCM, KEY_128);
    if(check_result((uint8_t *) cipher_p_list[i], c, tag_size_list[i] + p_size_list[i]) != TEST_SUCCESS){
      printf("Test CCM failed!\n");
      ++(*test_fail);
    }
    else {
      printf("Test CCM successful!\n");
      ++(*test_pass);
    }
    printf("Expected: ");
    dump_data((uint8_t *) cipher_p_list[i], tag_size_list[i] + p_size_list[i]);
    printf("Received: ");
    dump_data(c, tag_size_list[i] + p_size_list[i]);
    free_wrap(c);
  }
}
void test_decrypt_aes_ccm(int* test_pass, int* test_fail){
  for (int i = 0; i < CCM_TEST_NUM; ++i){
    uint8_t *c;
    int o_size = p_size_list[i];

    c = malloc_wrap(o_size);
    memset(c, 0, p_size_list[i]);

    int ret = aes_aead_ccm_decrypt(c, (uint8_t *) key_p_list[i],
		 (uint8_t *) ad_p_list[i], ad_size_list[i],
		 (uint8_t *) cipher_p_list[i], c_size_list[i],
		 (uint8_t *) iv_p_list[i], iv_size_list[i],
		 tag_size_list[i],
		 CCM, KEY_128);

    if(check_result((uint8_t *) payload_p_list[i], c, p_size_list[i]) != TEST_SUCCESS || ret != AES_OK){
      printf("Test Decrypt CCM failed!\n");
      ++(*test_fail);
    }
    else {
      printf("Test Descrypt CCM successful!\n");
      ++(*test_pass);
    }
    printf("Expected: ");
    dump_data((uint8_t *) payload_p_list[i], p_size_list[i]);
    printf("Received: ");
    dump_data(c, p_size_list[i]);
    free_wrap(c);
  }
}

void test_aes_ctr(int* test_pass, int* test_fail)
{
  uint8_t *c;
  
  printf("Test AES ctr mode\n");
  printf("Key size: 128\n");

  c = malloc_wrap(CTR_DATA_SIZE);
  aes_encrypt(c, ctr_key, ctr_plaintext, CTR_DATA_SIZE, ctr_iv, CTR_KEY_SIZE, CTR, KEY_256);
  if(check_result(ctr_ciphertext, c, CTR_DATA_SIZE) != TEST_SUCCESS){
    printf("Test CTR failed!\n");
    ++(*test_fail);
  }
  else {
    printf("Test CTR successful!\n");
    ++(*test_pass);
  }
  printf("Expected: ");
  dump_data(ctr_ciphertext, CTR_DATA_SIZE);
  printf("Actual: ");
  dump_data(c, CTR_DATA_SIZE);
  free_wrap(c);
}

void test_aes_cbc(int* test_pass, int* test_fail)
{
  uint8_t *c;
  
  printf("Test AES cbc mode\n");
  printf("Key size: 128\n");

  c = malloc_wrap(CBC_DATA_SIZE);
  aes_encrypt(c, cbc_key, cbc_plaintext, CBC_DATA_SIZE, cbc_iv, CBC_KEY_SIZE, CBC, KEY_128);
  if(check_result(cbc_ciphertext, c, CBC_DATA_SIZE) != TEST_SUCCESS){
    printf("Test CBC failed!\n");
    ++(*test_fail);
  }
  else {
    printf("Test CBC successful!\n");
    ++(*test_pass);
  }
  printf("Expected: ");
  dump_data(cbc_ciphertext, CBC_DATA_SIZE);
  printf("Actual: ");
  dump_data(c, CBC_DATA_SIZE);
  free_wrap(c);
}
void test_aes_ofb(int* test_pass, int* test_fail){

  uint8_t *c;
  
  printf("Test AES OFB mode\n");
  printf("Key size: 128\n");

  c = malloc_wrap(OFB_DATA_SIZE);
  aes_encrypt(c, ofb_key, ofb_plaintext, OFB_DATA_SIZE, ofb_iv, OFB_KEY_SIZE, OFB, KEY_192);
  if(check_result(ofb_ciphertext, c, OFB_DATA_SIZE) != TEST_SUCCESS){
    printf("Test OFB failed!\n");
    ++(*test_fail);
  }
  else {
    printf("Test OFB successful!\n");
    ++(*test_pass);
  }
  printf("Expected: ");
  dump_data(ofb_ciphertext, OFB_DATA_SIZE);
  printf("Actual: ");
  dump_data(c, OFB_DATA_SIZE);
  free_wrap(c);
}

void test_aes_ecb(int* test_pass, int* test_fail){
  int test_len[6] = {16, 32, 48, 128, 256, 512};
  int test_offset[3] = {0, 128, 16*13};
  uint8_t *c;
  printf("Test AES ECB mode\n");
  printf("Key Size: 128\n");
  for(int i = 0; i < 3; i++)
    for(int j =0; j < 6; j++)
      {
	c = malloc_wrap(test_len[j]);
	aes_encrypt(c, test_key, test_plaintext+test_offset[i], test_len[j], NULL, 0, ECB, KEY_128);
	if(check_result(test_ciphertext+test_offset[i], c, test_len[j]) != TEST_SUCCESS){
	  printf("Test: len: %4d offset: %4d failed!\n", test_len[j], test_offset[i]);
	  printf("Expected: ");
	  dump_data(test_ciphertext+test_offset[i],test_len[j]);
	  printf("Actual: ");
	  dump_data(c,test_len[j]);
	  ++(*test_fail);
	}
	else {
	  printf("Test: len: %4d offset: %4d successful!\n", test_len[j], test_offset[i]);
	  ++(*test_pass);
	}
	free_wrap(c);
      }
}

status_t check_result(const uint8_t *exp, const uint8_t *data, int len)
{
  for(int i = 0; i < len; ++i)
    if(exp[i] != data[i])
      return TEST_FAILURE;
  return TEST_SUCCESS;
}
void dump_data(const uint8_t *data, int len)
{
  for(int i = 0; i < len; i++){
    if (i%16 == 0) printf("\n");
    printf("%02X", data[i]);
  }
  printf("\n");
}
