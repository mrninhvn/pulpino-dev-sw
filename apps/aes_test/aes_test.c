/*
 * Project name   :
 * File name      : helloworld.c
 * Created date   : Fri 26 May 2017 11:37:44 AM +07
 * Author         : Ngoc-Sinh Nguyen
 * Last modified  : Fri 26 May 2017 11:37:44 AM +07
 * Desc           :
 */

#include <stdio.h>
#include <string.h>
#include "aes_test.h"
#include "aes_cbc.h"
#include "aes_ctr.h"
#include "aes_ofb.h"
#include "aes_ccm.h"
#include "aes_gcm.h"
#include "ghash.h"

#define  PULPINO_BASE_INFO     0x1A107010

#define  AES_BASE_ADDR         0x1A120000
#define  AES_REG_STATUS_OFF    0x00
#define  AES_REG_CONF_OFF      0x04
#define  AES_REG_KEYIN         0x08
#define  AES_REG_IVIN          0x10

#define  AES_DMA_SRC_REG       0x24
#define  AES_DMA_DES_REG       0x28
#define  AES_STATUS            0x2c

#define  AES_ENC_TIMES         0x30
#define  AES_ENC_CYCLES        0x34

#define  MAX_TEST_CASE         1000

typedef unsigned long uint32;
typedef unsigned char uint8;

#define AXI_FIXED_BURST 0
#define AXI_INCR_BURST  1
#define AXI_WRAP_BURST  2

#define BIG_MODE   0
#define SMALL_MODE 1
#define LOOP_MODE  2

#define KEY_SIZE_128 0
#define KEY_SIZE_192 1
#define KEY_SIZE_256 2

#define TRUE  1
#define FALSE 0

#define ENCRYPT_ECB   0
#define ENCRYPT_CTR   1
#define ENCRYPT_CBC   2
#define ENCRYPT_CCM   3
#define ENCRYPT_GCM   4
#define ENCRYPT_RAN   5
#define ENCRYPT_OFB   6
#define ENCRYPT_EAX   7

/* Register */
volatile uint32 *pulpino_status = (uint32 *) (PULPINO_BASE_INFO);

volatile uint32 *status_reg = (uint32 *) (AES_BASE_ADDR + AES_STATUS);
volatile uint32 *config_reg = (uint32 *) (AES_BASE_ADDR + AES_REG_CONF_OFF);
volatile uint32 *keyin_reg  = (uint32 *) (AES_BASE_ADDR + AES_REG_KEYIN);
volatile uint32 *ivin_reg   = (uint32 *) (AES_BASE_ADDR + AES_REG_IVIN);
					  
volatile uint32 *dma_src_reg = (uint32 *) (AES_BASE_ADDR + AES_DMA_SRC_REG);
volatile uint32 *dma_des_reg = (uint32 *) (AES_BASE_ADDR + AES_DMA_DES_REG);
volatile uint32 *aes_status  = (uint32 *) (AES_BASE_ADDR + AES_STATUS);

int check_result(uint8 * actual, uint8 *  expected, int data_len) {
  for (int i = 0; i < data_len; ++i) {
    if (expected[i] != actual[i]) {
      printf("\nExpected: ");
      print_data(expected, data_len);
      printf("\nActual  : ");
      print_data(actual, data_len);
      printf("\n");
      return FALSE;      
    }
  }
  return TRUE;
}

int check_ccm_result(uint8 * actual, uint8 * expected, int tag_len, int pay_len) {
  int tag_len_tmp = 16 * ((int)(tag_len / 16) + 1);
  int pay_len_tmp = 16 * ((int)(pay_len / 16) + 1);
  int ccm_data_len = tag_len_tmp + pay_len_tmp;

  for (int i = 0; i < tag_len_tmp; ++i) {
    if (i < tag_len) {
      if (expected[i+tag_len] != actual[i]) {
	printf("\nExpected: ");
	print_data(expected, tag_len);
	printf("\nActual  : ");
	print_data(actual, tag_len);
	printf("\n");
	return FALSE;      
      }
    }
  }
  
  for (int i = 0; i < pay_len_tmp; ++i) {
    if (i < pay_len) {
      if (expected[i] != actual[i + tag_len_tmp]) {
	printf("\nExpected: ");
	print_data(expected, pay_len + tag_len);
	printf("\nActual  : ");
	print_data(actual, pay_len_tmp + tag_len_tmp);
	printf("\n");
	return FALSE;      
      }
    }
  }
  
  return TRUE;
}

// 32 bits: |31 <-- Burst Length (16-bit)    --> 16|
//          |15 <-- NONE                     --> 13|
//          |12 <-- Burst Size (3-bit)       --> 10|
//          |9  <-- Burst Type (2-bit)       -->  8|
//          |7  <== Activate Signals (8-bit) ==>  0|
//          |7  <-- Encryption Mode (3-bit)  -->  5|
//          |4: DMA activate                       |
//          |3  <-- Key Size (2-bit)         -->  2|
//          |1:                                    |
//          |0: Loop Test                          |
// For Example : |0000000000000011|000|000|10|000|1|00|0|0| = 0x00030210
// Burst Length = 0000000000000011 (3)
// Burst Size = 000 (0)
// Burst Type = 10 (WRAPPING)
// Encryption Mode = 000 (ECB Encryption)
// Active DMA = 1 (ACTIVE)
// Key Size = 00 (128-bit)
// Loop Test = 0 (NONE)

uint32 generate_info (int len_data, int size_data, int encrypt_mode,
		      int burst_type, int test_mode) {
  uint32 config_info;

  uint32 active_signal = 0x00000010;

  if (test_mode == LOOP_MODE) {
    config_info = (uint32) (active_signal)
                | (uint32) (encrypt_mode    << 5)
                | (uint32) (active_signal   >> 4)
                | (uint32) (burst_type      << 8)
                | (uint32) ((size_data - 1) << 10)
                | (uint32) ((len_data - 1)  << 16);
  } else {
    config_info = (uint32) (active_signal)
                | (uint32) (encrypt_mode    << 5)
                | (uint32) (burst_type      << 8)
                | (uint32) ((size_data - 1) << 10)
                | (uint32) ((len_data - 1)  << 16);
  }
  return config_info;
}

int wait_ready()
{
    uint32 n = 100;

    for (int i = 0; i < 1000; ++i){
    #ifdef __riscv__ 
        asm volatile ("nop");
    #else
        asm volatile ("l.nop");
    #endif
    }
    return (n > 0);
}


int run_test (uint8 * key, uint8 * iv, uint8 * datain, uint8 * dataout,
	      int data_len, int test_mode, int encrypt_mode) {
  
  for (int i = 0; i < data_len; ++i)
    dataout[i] = 0; // clear output buffer before loading
  
  while (*status_reg != 0);
  uint32 * iv_tmp  = (uint32 *) iv;
  uint32 * key_tmp = (uint32 *) key;
  
  *keyin_reg = key_tmp[0];
  *keyin_reg = key_tmp[1];
  *keyin_reg = key_tmp[2];
  *keyin_reg = key_tmp[3];

  *ivin_reg = iv_tmp[0];
  *ivin_reg = iv_tmp[1];
  *ivin_reg = iv_tmp[2];
  *ivin_reg = iv_tmp[3];

  // Normal test
  while (*status_reg != 0);

  *dma_des_reg = (uint32 *) dataout;
  
  *dma_src_reg = (uint32 *) datain;

  *config_reg = generate_info(data_len/4, 1, encrypt_mode, AXI_INCR_BURST, test_mode);

  if (test_mode == LOOP_MODE) {
    wait_ready();
    *config_reg = 0x00000000;
  } 
    
  while (*status_reg != 0);

  return 0;
}

// NOTE: mod(size_of_X, 16) == 0
uint8 * gcm_hash (uint8 * H, uint8 * X, int x_len) {
  uint8 * Y_tmp = malloc(16);
  memset(Y_tmp, 0, 16);
  
  br_ghash_ctmul32(Y_tmp, H, X, x_len);

  return Y_tmp;
}

int padding_len_ccm_data (int len_adata, int len_payload) {
  int len_ccm_data;

  len_ccm_data = 16 * ((int)(len_payload/16 + 1) + (int)(len_adata/16 + 1) + 1);

  return len_ccm_data;
}

uint8 * padding_data (uint8 * data, int tag_len, int pay_len) {
  int len_data_tmp;
  
  uint8 * data_tmp;
  
  len_data_tmp = 16 * (((int)(tag_len/16) + 1) + ((int)(pay_len/16) + 1));

  data_tmp = (uint8 *) malloc(len_data_tmp);

  for (int i = 0; i < len_data_tmp; ++i)
    data_tmp[i] = (uint8) 0;

  for (int i = 0; i < pay_len; ++i)
    data_tmp[i] = data[i];
  
  return data_tmp;
}

uint8 * padding_gcm_iv_data (uint8 * data, int data_len) {
  int len_data_tmp;
  
  uint8 * data_tmp;

  // Check len(IV) = 96 or not
  if (data_len == 12) {
    len_data_tmp = 16;
  } else {
    len_data_tmp = 16 * ((int)(data_len/16) + 2);
  }

  data_tmp = (uint8 *) malloc(len_data_tmp);

  for (int i = 0; i < len_data_tmp; ++i)
    data_tmp[i] = (uint8) 0;

  for (int i = 0; i < data_len; ++i)
    data_tmp[i] = data[i];

  if (data_len == 12) {
    data_tmp[len_data_tmp-1] = (uint8) 1;
  } else {
    for (int i = 0; i < 4; ++i) {
      data_tmp[len_data_tmp - i - 1] = (uint8) (((data_len * 8) >> i * 8) & 0xff);
    }
  }
  
  return data_tmp;
}

uint8 * padding_gcm_data (uint8 * data, int data_len) {
  int len_data_tmp;

  uint8 * data_tmp;

  len_data_tmp = 16 * (((int)(data_len/16) + 1));

  data_tmp = (uint8 *) malloc(len_data_tmp);

  for (int i = 0; i < len_data_tmp; ++i)
    data_tmp[i] = (uint8) 0;

  for (int i = 0; i < data_len; ++i)
    data_tmp[i] = data[i];
  
  return data_tmp;
}

uint8 * gcm_data_formatting (uint8 * AData, uint8 * Cipher, int len_adata, int len_cipher) {
  uint8 * gcm_data;
  int len_adata_tmp  = 16 * ((int)(len_adata/16) + 1);
  int len_cipher_tmp = 16 * ((int)(len_cipher/16) + 1);
  int len_gcm_data   = len_adata_tmp + len_cipher_tmp + 16;

  gcm_data = (uint8 *) malloc(len_gcm_data);

  for (int i = 0; i < len_gcm_data; ++i)
    gcm_data[i] = (uint8) 0;

  for (int i = 0; i < len_adata; ++i)
    gcm_data[i] = AData[i];

  for (int i = 0; i < len_cipher; ++i)
    gcm_data[i+len_adata_tmp] = Cipher[i];

  int len_cipher_1 = len_cipher * 8;
  int len_adata_1  = len_adata * 8;

  for (int i = 0; i < 4; ++i) {
    gcm_data[len_gcm_data - i - 1] = (uint8) ((len_cipher_1 >> i * 8) & 0xff);
    gcm_data[len_gcm_data - i - 9] = (uint8) ((len_adata_1  >> i * 8) & 0xff);
  }
  
  return gcm_data;
}

uint8 * ctr_data_formatting (uint8 * Nonce, int len_nonce) {
  uint8 * ctr_data;

  ctr_data = (uint8 *) malloc(16);
  
  for (int i = 0; i < 16; ++i)
    ctr_data[i] = (uint8) 0;
  
  int len_q = 15 - len_nonce;
  
  ctr_data[0] = (uint8) (len_q - 1);

  for (int i = 0; i < len_nonce; ++i)
    ctr_data[i+1] = Nonce[i];
  
  return ctr_data;
}

int run_gcm_test (uint8 * key, uint8 * iv, uint8 * authtext, uint8 * plaintext,
		   uint8 * tag, uint8 * ciphertext, int test_mode, int key_len,
		   int iv_len, int plain_len, int auth_len, int * tag_len, int * cipher_len) {
  volatile uint8 aes_ciphertext_1[1600];
  volatile uint8 aes_ciphertext[1600];
	  
  uint8 none_text[16] = {};
	  
  run_test(key, iv, none_text,
	   aes_ciphertext_1, 16, test_mode, ENCRYPT_ECB);

  uint8 * aes_tmp = aes_ciphertext_1;
	  
  int iv_len_tmp, gcm_len;

  if (iv_len == 12) {
    iv_len_tmp = 16;
  } else {
    iv_len_tmp = 16 * ((int)(iv_len/16) + 2);
  }

  uint8 * iv_tmp = padding_gcm_iv_data(iv, iv_len);
  
  uint8 * plain_tmp = padding_gcm_data(plaintext, plain_len);

  int plain_len_tmp = 16 * ((int)(plain_len/16) + 1);
	  
  uint8 * gcm_data;

  uint8 * gcm_cipher;

  uint8 * iv_ghash;
	  
  if (iv_len != 12) {
    print_data(iv_tmp, iv_len_tmp);
    iv_ghash = gcm_hash(aes_tmp, iv_tmp, iv_len_tmp);
    print_data(iv_ghash, 16);
  } else {
    iv_ghash = iv_tmp;
  }
  uint8 iv_tmp_1[16];

  for (int i = 0; i < 16; ++i)
    iv_tmp_1[i] = iv_ghash[i];
	  
  iv_tmp_1[15] = iv_tmp_1[15] + 1;

  run_test(key, iv_tmp_1, plain_tmp, aes_ciphertext,
	   plain_len_tmp, test_mode, ENCRYPT_CTR);

  for (int i = 0; i < plain_len_tmp; ++i) {
    if (i >= plain_len) {
      gcm_cipher[i] = (uint8) 0;
    } else {
      gcm_cipher[i] = (uint8) aes_ciphertext[i];
    }
  }
  gcm_data = gcm_data_formatting(authtext, gcm_cipher, auth_len, plain_len);

  gcm_len = 16 + 16 * ((int)(plain_len/16) + 1) + 16 * ((int)(plain_len/16) + 1);
	  
  uint8 * gcm_tmp = gcm_hash(aes_tmp, gcm_data, gcm_len);

  run_test(key, iv_ghash, gcm_tmp, aes_ciphertext, 16, test_mode, ENCRYPT_CTR);

  if (check_result(aes_ciphertext, tag, 16) == TRUE
      && check_result(gcm_cipher, ciphertext, cipher_len) == TRUE) {
    return TRUE;
  } else {
    return FALSE;
  }
}

uint8 * ccm_data_formatting (uint8 * Nonce, uint8 * AData, uint8 * Payload,
			      int len_nonce, int len_adata, int len_payload, int len_tag) {
  uint8 * ccm_data;
  
  int len_ccm_data = padding_len_ccm_data(len_adata, len_payload);
  int start_payload = 16 * ((int)(len_adata/16 + 1) + 1);
  //  printf("Payload Pos: %d\n", start_payload);
  
  ccm_data = (uint8 *) malloc(len_ccm_data);
  for (int i = 0; i < len_ccm_data; ++i)
    ccm_data[i] = (uint8) 0;
  
  int len_q = 15 - len_nonce;

  if (len_adata > 0) {
    ccm_data[0] = (uint8) (1                 << 6)
                | (uint8) (((len_tag - 2)/2) << 3)
                | (uint8) (len_q - 1);
  } else {
    ccm_data[0] = (uint8) (((len_tag - 2)/2) << 3)
                | (uint8) (len_q - 1);
  }

  for (int i = 1; i <= len_nonce; ++i)
    ccm_data[i] = Nonce[i-1];

  for (int i = 14; i > len_nonce; --i)
    ccm_data[i] = (uint8) 0;

  ccm_data[15] = (uint8) len_tag;

  // FIXME: Only supported 2^16 - 2^8 bit or smaller
  if (len_adata < 65280 && len_adata > 0) {
    ccm_data[16] = (uint8) (len_adata / 256);
    ccm_data[17] = (uint8) (len_adata % 256);
	
    for (int i = 0; i < len_adata; ++i)
      ccm_data[i+18] = AData[i];
  }

  for (int i = 0; i < len_payload; ++i)
    ccm_data[i + start_payload] = Payload[i];

  return ccm_data;
}

void print_data (uint8 * data, int data_len) {
  for (int i = 0; i < data_len; ++i){ 
    if (i % 16  == 0) printf("\n%d  : ", i/16 + 1);
    printf("%02X", data[i]);
  }
}

int main()
{
    printf("Now, Play with AES!!!!!\n");

    int data_len[5] = {16, 32, 48, 128, 1600};

    int test_mode = SMALL_MODE;
    
    volatile uint8 aes_ciphertext[1600];

    int pass_case = 0;
    int fail_case = 0;

    int encrypt_mode = ENCRYPT_GCM;
    
    //    printf("Pulpino status: %08X\n", *pulpino_status);
    if ((*pulpino_status >> 28) == 0) {
      printf("AES_VNU not found!");
    } else {

      int num_test = 4;
      if (test_mode == SMALL_MODE) {
	if (encrypt_mode == ENCRYPT_ECB) {
      
	  for (int i = 0; i < num_test; ++i) {
	    printf("TEST_CASE %d: ", i+1);
	    run_test(test_key, test_iv, test_plaintext,
		     aes_ciphertext, data_len[i], test_mode, encrypt_mode);

	    if(check_result(aes_ciphertext, test_ciphertext, data_len[i])) {
	      pass_case++;
	      printf("OKAY\n");	
	    } else {
	      printf("FALSE\n");
	      fail_case++;
	    }
	  }
    
	} else if (encrypt_mode == ENCRYPT_CBC) {
	  printf("TEST CBC MODE!\n");

	  run_test(cbc_key, cbc_iv, cbc_plaintext,
		   aes_ciphertext, CBC_DATA_SIZE, test_mode, encrypt_mode);
	  
	  if(check_result(aes_ciphertext, cbc_ciphertext, CBC_DATA_SIZE)) {
	    pass_case++;
	    printf("OKAY\n");	
	  } else {
	    fail_case++;
	    printf("FALSE\n");
	  }
	} else if (encrypt_mode == ENCRYPT_CTR) {
	  printf("TEST CTR MODE!\n");

	  run_test(ctr_key, ctr_iv, ctr_plaintext,
		   aes_ciphertext, CTR_DATA_SIZE, test_mode, encrypt_mode);

	  if(check_result(aes_ciphertext, ctr_ciphertext, CTR_DATA_SIZE)) {
	    pass_case++;
	    printf("OKAY\n");
	  } else {
	    fail_case++;
	    printf("FALSE\n");
	  }
	} else if (encrypt_mode == ENCRYPT_OFB) {
	  printf("TEST OFB MODE!\n");

	  run_test(ofb_key, ofb_iv, ofb_plaintext,
		   aes_ciphertext, CTR_DATA_SIZE, test_mode, encrypt_mode);

	  if(check_result(aes_ciphertext, ofb_ciphertext, OFB_DATA_SIZE)) {
	    pass_case++;
	    printf("OKAY\n");
	  } else {
	    fail_case++;
	    printf("FALSE\n");
	  }
	  
	} else if (encrypt_mode == ENCRYPT_CCM) {
	  printf("TEST CCM MODE!\n");

	  int ccm_data_len = padding_len_ccm_data(CCM_ADATA_SIZE, CCM_PAY_SIZE);

	  uint8 * ccm_data = ccm_data_formatting(ccm_nonce, ccm_adata, ccm_pay,
						 CCM_NONCE_SIZE, CCM_ADATA_SIZE,
						 CCM_PAY_SIZE, CCM_TAG_SIZE);
	  uint8 * ccm_iv = ctr_data_formatting(ccm_nonce, CCM_NONCE_SIZE);
	  
	  run_test(ccm_key, ccm_iv, ccm_data, aes_ciphertext,
		   ccm_data_len, test_mode, encrypt_mode);
	  
	  run_test(ccm_key, ccm_iv, padding_data(ccm_pay, CCM_TAG_SIZE, CCM_PAY_SIZE),
		   aes_ciphertext, (CCM_PAY_SIZE + CCM_TAG_SIZE) * 4, test_mode,
		   encrypt_mode);

	  if(check_ccm_result(aes_ciphertext, ccm_cipher, CCM_TAG_SIZE, CCM_PAY_SIZE)) {
	    pass_case++;
	    printf("OKAY\n");
	  } else {
	    fail_case++;
	    printf("FALSE\n");
	  }
	} else if (encrypt_mode == ENCRYPT_GCM) {
	  printf("TEST GCM MODE!\n");
	  for (int i = 0; i < 4; ++i) {
	    if(run_gcm_test(gcm_key[i], gcm_iv[i], gcm_authtext[i], gcm_plaintext[i],
			    gcm_tag[i], gcm_ciphertext[i], test_mode, GCM_KEY_SIZE,
			    GCM_IV_SIZE[i], GCM_PLAIN_SIZE[i], GCM_AUTH_SIZE[i],
			    GCM_TAG_SIZE, GCM_CIPHER_SIZE[i])) {
	      pass_case++;
	      printf("%d : OKAY\n", i+1);
	    } else {
	      fail_case++;
	      printf("%d : FALSE\n", i+1);
	    }
	  }
	}
      } else if (test_mode == BIG_MODE) {
	printf("TEST_CASE_X\n");
	pass_case = 0; 
	run_test(test_key, test_iv, test_plaintext, 
		 aes_ciphertext, data_len[4], test_mode, encrypt_mode);
      
	if(check_result(aes_ciphertext, test_ciphertext, data_len[4])) {
	  pass_case++;
	} else {
	  fail_case++;
	}

	if (pass_case)
	  printf("PASS\n");
	else
	  printf("FALSE\n");
      
      } else if (test_mode == LOOP_MODE) {

	run_test(test_key, test_iv, test_plaintext,
		 aes_ciphertext, data_len[0], test_mode, encrypt_mode);
      }
    }
    printf("\nPASS: %d/%d\n", pass_case, fail_case+pass_case);

    return 0;
}
