#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "sis_aes.h"
#include "ghash.h"

#define  AES_BASE_ADDR         0x1A120000

#define  AES_CONFIG_REG        0x04
#define  AES_KEYIN_REG         0x08
#define  AES_IV_REG            0x10

#define  AES_DMA_SRC_REG       0x24
#define  AES_DMA_DES_REG       0x28
#define  AES_STATUS_REG        0x2c

#define  AES_ENC_TIMES         0x30
#define  AES_ENC_CYCLES        0x34

/* scatch pad buffer for AES */
#define AES_MEM_SIZE 2048
/* #define SIS_AES_DEBUG_MALLOC */

static uint8_t aes_scratch_pad[AES_MEM_SIZE];
static uint32_t aes_current_ptr = (uint32_t) &aes_scratch_pad;
uint32_t get_scratchpad_ptr(){ return aes_current_ptr; }

uint8_t* amalloc(int size){
  int rsize = 0;
  // ensure start address is aligned
  // FIXME: add compiler dirative to insure aes_scratch_pad aligned?
  if (!(aes_current_ptr & 0xf)){
    aes_current_ptr += (16 - (aes_current_ptr & 0xf));
  }

  if ((size & 0x3) == 0) {
    rsize = size;
  }
  else {
    rsize = size + (16 - (size & 0x3));
  }
#ifdef SIS_AES_DEBUG_MALLOC
  printf("Allocate: %08X at %08X\n", rsize, aes_current_ptr);
#endif
  aes_current_ptr += rsize;
  if (aes_current_ptr > ((uint32_t) aes_scratch_pad + AES_MEM_SIZE)){
    printf("ERROR: SIS AES: out of scratch pad memory\n");
    return (uint8_t *) &aes_scratch_pad;
  }
  else {
    return (uint8_t *) (aes_current_ptr - rsize);
  }
}
void afree(uint8_t *ptr){
  uint32_t size = (aes_current_ptr - (uint32_t) ptr);
  
  if (size > 0)
    aes_current_ptr -= size;
  else
    printf("ERROR: SIS AES free unllocated memory\n");
#ifdef SIS_AES_DEBUG_MALLOC
  printf("Free: %08X at %08X\n", size, aes_current_ptr);
#endif
}

/* Register */
volatile uint32_t *aes_info_reg	   = (uint32_t *) AES_BASE_ADDR;
volatile uint32_t *aes_status_reg  = (uint32_t *) (AES_BASE_ADDR + AES_STATUS_REG);
volatile uint32_t *aes_config_reg  = (uint32_t *) (AES_BASE_ADDR + AES_CONFIG_REG);
volatile uint32_t *aes_keyin_reg   = (uint32_t *) (AES_BASE_ADDR + AES_KEYIN_REG);
volatile uint32_t *aes_iv_reg      = (uint32_t *) (AES_BASE_ADDR + AES_IV_REG);
volatile uint32_t *aes_dma_src_reg = (uint32_t *) (AES_BASE_ADDR + AES_DMA_SRC_REG);
volatile uint32_t *aes_dma_des_reg = (uint32_t *) (AES_BASE_ADDR + AES_DMA_DES_REG);

/* return the status register in form of 32-bit unsigned int */
uint32_t aes_get_status(){
  return *aes_status_reg;
}

// 32 bits: |31 <-- Burst Length (16-bit)    --> 16|
//          |15 <-- NONE                     --> 13|
//          |12 <-- Burst Size (3-bit)       --> 10|
//          |9  <-- Burst Type (2-bit)       -->  8|
//          |7  <== Activate Signals (8-bit) ==>  0|
//          |7  <-- Encryption Mode (3-bit)  -->  4|
//          |3: DMA activate                       |
//          |2  <-- Key Size (2-bit)         -->  1|
//          |0: Loop Test                          |
void print_data(uint8_t *d, int len)
{
  for (int i = 0; i < len; ++i){
    if (i % 16 == 0) printf("\n");
    printf("%02X", d[i]);
  }
  printf("\n");
}
#define DEBUG
#ifdef DEBUG
#define print(var) printf("%s: %08X\n", #var, var)
#define dump_data(d,len) print_data(d, len)
#else
#define dump_data(d, len)
#define print(var)
#endif

aes_status_t aes_set_config(key_size_t key_size, uint32_t enable, uint32_t burst_len,
			 enc_mode_t enc_mode, uint32_t loop){
  uint32_t burst_size = 1;
  axi_burst_t burst_mode = INCR;
  if (key_size > KEY_256 || burst_mode > WRAP || enc_mode > OFB
      || burst_size > 8)
    return AES_CFG_ERROR;
  uint32_t cfg = (burst_len-1) << 16;
  cfg |= (burst_size-1) << 10;
  cfg |= (burst_mode << 8);
  cfg |= (enc_mode << 4);
  cfg |= (enable & 0x1) << 3;
  cfg |= (key_size & 0x3) << 1;
  cfg |= loop & 0x1;
  *aes_config_reg = cfg;
  return AES_OK;
}

aes_status_t aes_set_key(const uint8_t *key, key_size_t key_size){
  uint32_t *key_tmp = (uint32_t *) key;
  for(int i = 0; i < 4 + key_size * 2; ++i)
    *aes_keyin_reg = key_tmp[i];
  return AES_OK;
}
aes_status_t aes_set_iv(const uint8_t *iv, uint32_t iv_len)
{
  uint32_t *iv_tmp = (uint32_t *) iv;
  for(int i =0; i < 4; ++i)
    *aes_iv_reg = iv_tmp[i];
  return AES_OK;
}
aes_status_t aes_encrypt(uint8_t *ciphertext, const uint8_t *key, const uint8_t *plaintext,
			 uint32_t data_len, const uint8_t *iv, uint32_t iv_len, enc_mode_t enc_mode, key_size_t key_size)
{
  /* printf("Plaintext:"); */
  /* dump_data(plaintext, data_len); */
  /* printf("IV:"); */
  /* dump_data(iv,iv_len); */
  /* printf("KEY:"); */
  /* dump_data(key,16); */
  while(aes_get_status() != 0);
  // wait until the encryption core is free
  aes_set_key(key, key_size);
  if ( iv != NULL && iv_len != 0 )
    aes_set_iv(iv,iv_len);
  *aes_dma_des_reg = (int) ciphertext;
  *aes_dma_src_reg = (int) plaintext;
  if(aes_set_config(key_size, 1, data_len/4, enc_mode, 0) != AES_OK)
    return AES_CFG_ERROR;
  while(aes_get_status() != 0);
  /* printf("CipherText: "); */
  /* dump_data(ciphertext, data_len); */
  return AES_OK;
}
uint32_t get_size_padding_128(int data_len){
  if (data_len & 0xf)
    return data_len + (16 - (data_len & 0xF));
  else
    return data_len;
}
uint8_t aes_ccm_get_flag(uint32_t ad_len, uint32_t tag_len, uint32_t iv_len){
  /* if (tag_len > 16 || iv_len > 16) */
  /*   perror("Error!"); */
  uint8_t t = (tag_len-2)/2;
  uint8_t q = 15-iv_len -1;
  if (ad_len > 0)
    return (1 << 6) | ((t &0x7) << 3) | (q & 0x7);
  else
    return ((t &0x7) << 3) | (q & 0x7);
}

aes_status_t aes_aead_ccm(uint8_t *aead_out, const uint8_t *key,
			  const uint8_t *ad, uint32_t ad_len,
			  const uint8_t *plaintext, uint32_t plaintext_len,
			  const uint8_t *iv, uint32_t iv_len,
			  uint32_t tag_len,
			  enc_mode_t enc_mode, key_size_t key_size)
{

  uint32_t a_len = get_size_padding_128(2+ad_len);
  uint32_t p_len = get_size_padding_128(plaintext_len);
  uint32_t ccm_data_len = 16 + a_len + p_len;

  uint8_t * ccm_data = malloc_wrap(ccm_data_len+16);
  uint8_t * ccm_data_cbc = malloc_wrap(ccm_data_len);
  uint8_t * iv_tmp = malloc_wrap(16);

  memset(ccm_data_cbc,0, ccm_data_len);
  memset(iv_tmp, 0, 16);
  memcpy(iv_tmp+1, iv, iv_len);
  iv_tmp[0] = (15-iv_len-1) & 0xff;
  memset(ccm_data, 0, ccm_data_len+16);

  uint8_t flag = aes_ccm_get_flag(ad_len, tag_len, iv_len);
  ccm_data[0] = flag;
  memcpy(ccm_data+1, iv, iv_len);

  for (unsigned int i = 15; i > iv_len && i > 11; --i)
    ccm_data[i] = (plaintext_len >> ((15-i)*8)) & 0xff;

  // FIXME: only support ad_len < 2^16-2^8
  ccm_data[16] = ad_len >> 8 & 0xff;
  ccm_data[17] = ad_len & 0xff;

  memcpy(ccm_data + 16+2, ad, ad_len);
  memcpy(ccm_data + 16 + a_len, plaintext, plaintext_len);

  aes_encrypt(ccm_data_cbc, key, ccm_data, ccm_data_len, iv_tmp, 16, CCM, KEY_128);
  aes_encrypt(ccm_data_cbc, key, ccm_data + 16 + a_len, p_len + 16, iv_tmp, 16, CCM, KEY_128);

  memcpy(aead_out, ccm_data_cbc + 16, plaintext_len);
  memcpy(aead_out+plaintext_len, ccm_data_cbc, tag_len);

  // free the memory
  free_wrap(iv_tmp);
  free_wrap(ccm_data_cbc);
  free_wrap(ccm_data);
  /* free(16 + ccm_data_len + ccm_data_len+16); */
  return AES_OK;
}
uint8_t sis_strcmp(uint8_t * a, uint8_t* b, int len)
{
  for (int i = 0; i < len; ++i)
    if ( (a[i] & 0xff) != (b[i] & 0xff) ) return 0;
  return 1;
}
aes_status_t aes_aead_ccm_decrypt(uint8_t *aead_out, const uint8_t *key,
			  const uint8_t *ad, uint32_t ad_len,
			  const uint8_t *ciphertext, uint32_t cipher_len,
			  const uint8_t *iv, uint32_t iv_len,
			  uint32_t tag_len,
			  enc_mode_t enc_mode, key_size_t key_size)
{
  uint32_t p_len = cipher_len - tag_len;
  uint32_t plaintext_len = get_size_padding_128(p_len);
  uint32_t a_len = get_size_padding_128(ad_len + 2);
  uint32_t tag_check = AES_OK;
  uint32_t ccm_data_len = plaintext_len + a_len + 16;
  
  uint8_t * ccm_data = malloc_wrap(ccm_data_len);
  uint8_t * ccm_data_cbc = malloc_wrap(ccm_data_len);
  uint8_t * tag_data = malloc_wrap(get_size_padding_128(tag_len));
  uint8_t * plaintext_data = malloc_wrap(plaintext_len);
  uint8_t * iv_tmp = malloc_wrap(16);

  memcpy(tag_data, (ciphertext + cipher_len - tag_len), tag_len);
  // Formatting of IV
  memset(iv_tmp, 0, 16);
  memcpy(iv_tmp+1, iv, iv_len);
  iv_tmp[0] = (15-iv_len-1) & 0xff;
  iv_tmp[15] = iv_tmp[15] + 1;
  memset(plaintext_data, 0, plaintext_len);
  memset(ccm_data, 0, ccm_data_len);

  if (p_len <= 0) return AES_AUTH_ERROR;

  aes_encrypt(plaintext_data, key, ciphertext, plaintext_len, iv_tmp, 16, CTR, KEY_128);

  iv_tmp[15] -= 1;
  aes_encrypt(tag_data, key, tag_data, 16, iv_tmp, 16, CTR, KEY_128);

  // Formatting of B0. || Flag | Nounce/IV |      Q    ||
  //                   ||  0   |  1...15-q | 16-q...15 ||
  uint8_t flag = aes_ccm_get_flag(ad_len, tag_len, iv_len);
    
  ccm_data[0] =  flag;
  memcpy(ccm_data+1, iv, iv_len);

  for (unsigned int i = 15; i > iv_len && i > 11; --i)
    ccm_data[i] = (p_len >> ((15-i)*8)) & 0xff;
    
  // FIXME: Only support ad_len < 2^16 - 2^8
  ccm_data[16] = ad_len >> 8 & 0xff;
  ccm_data[17] = ad_len & 0xff;
    
  memcpy(ccm_data + 16 + 2, ad, ad_len);
  memcpy(ccm_data + 16 + a_len, plaintext_data, p_len);

  aes_encrypt(iv_tmp, key, ccm_data, 16, iv_tmp, 16, ECB, KEY_128);
  aes_encrypt(ccm_data_cbc+16, key, ccm_data+16, ccm_data_len-16, iv_tmp, 16, CBC, KEY_128);

  uint8_t * p = ccm_data_cbc + ccm_data_len - 16;

  if(sis_strcmp(tag_data, p, tag_len)) {
    memcpy(aead_out, plaintext_data, p_len);
    free_wrap(iv_tmp);
    free_wrap(plaintext_data);  
    free_wrap(tag_data);  
    free_wrap(ccm_data_cbc);  
    free_wrap(ccm_data);
    return AES_OK;
  }
  else {
    free_wrap(iv_tmp);
    free_wrap(plaintext_data);  
    free_wrap(tag_data);  
    free_wrap(ccm_data_cbc);  
    free_wrap(ccm_data);
    return AES_CFG_ERROR;
  }
}

aes_status_t aes_aead_gcm(uint8_t *aead_out, const uint8_t *key,
			  const uint8_t *ad, uint32_t ad_len,
			  const uint8_t *plaintext, uint32_t plaintext_len,
			  const uint8_t *iv, uint32_t iv_len,
			  uint32_t tag_len,
			  enc_mode_t enc_mode, key_size_t key_size)
{
  uint8_t *H = malloc_wrap(16);
  uint8_t *iv_gcm = malloc_wrap(16);
  int iv_gcm_tmp_len = 16 + get_size_padding_128(iv_len);
  uint8_t *iv_gcm_tmp;
  iv_gcm_tmp = malloc_wrap(iv_gcm_tmp_len);
  int pl_len = get_size_padding_128(plaintext_len);
  int a_len = get_size_padding_128(ad_len);
  uint8_t *ciphertext = malloc_wrap(pl_len);
  uint8_t *pl_pad = malloc_wrap(pl_len);
  uint8_t *len_ac = malloc_wrap(16);
  int ghash_data_len = a_len + pl_len + 16;
  uint8_t *ghash_data = malloc_wrap(ghash_data_len); // to store A_pad || c || len(a) || len(c);

  memset(ghash_data, 0, ghash_data_len);
  memset(len_ac, 0, 16);
  memset(pl_pad, 0, pl_len);
  memcpy(pl_pad, plaintext, plaintext_len);
  
  memset(iv_gcm, 0, 16);

  aes_encrypt(H, key, iv_gcm, 16, NULL, 0, ECB, KEY_128);

  // calculate IV
  if (iv_len == 12) {
    memcpy(iv_gcm, iv, iv_len);
    iv_gcm[15] += 1;
  } else {
    memset(iv_gcm_tmp, 0, iv_gcm_tmp_len);
    memcpy(iv_gcm_tmp, iv, iv_len);
    for(int i = 0; i < 4; i++)
      iv_gcm_tmp[iv_gcm_tmp_len-1-i] = ((iv_len<<3) >> (i<<3)) & 0xff;
    memset(iv_gcm, 0,16);
    br_ghash_ctmul32(iv_gcm, H, iv_gcm_tmp, iv_gcm_tmp_len);
  }
  iv_gcm[15] += 1;
  // y0
  aes_encrypt(ciphertext, key, pl_pad, pl_len, iv_gcm, iv_len, CTR, KEY_128);
  memcpy(aead_out, ciphertext, plaintext_len);
  for (int i = 15; i > 11; --i)
    len_ac[i] = ((plaintext_len*8) >> ((15-i)<<3) & 0xff);
  for (int i = 7; i > 3; --i)
    len_ac[i] = ((ad_len*8) >> ((7-i)<<3) & 0xff);

  memcpy(ghash_data, ad, ad_len);
  memcpy(ghash_data+a_len, ciphertext, plaintext_len);
  memcpy(ghash_data+a_len+pl_len, len_ac, 16);

  memset(iv_gcm_tmp, 0, 16);
  br_ghash_ctmul32(iv_gcm_tmp, H, ghash_data, a_len+pl_len+16);

  iv_gcm[15] -= 1;

  aes_encrypt(ciphertext, key, iv_gcm_tmp, 16, iv_gcm, 16, CTR, KEY_128);
  memcpy(aead_out+plaintext_len, ciphertext, tag_len);

  free_wrap(ghash_data);
  free_wrap(len_ac);
  free_wrap(pl_pad);
  free_wrap(ciphertext);
  free_wrap(iv_gcm_tmp);
  free_wrap(iv_gcm);
  free_wrap(H);
  return AES_OK;
}

aes_status_t aes_aead_gcm_decrypt(uint8_t *aead_out, const uint8_t *key,
				 const uint8_t *ad, uint32_t ad_len,
				 const uint8_t *aead_text, uint32_t aead_len,
				 const uint8_t *iv, uint32_t iv_len,
				 uint32_t tag_len,
				 enc_mode_t enc_mode, key_size_t key_size)
{
  uint8_t *H = malloc_wrap(16);
  uint8_t *iv_gcm = malloc_wrap(16);
  int iv_gcm_tmp_len = 16 + get_size_padding_128(iv_len);
  uint8_t *iv_gcm_tmp;
  iv_gcm_tmp = malloc_wrap(iv_gcm_tmp_len);
  int plaintext_len = aead_len - tag_len;
  int pl_len = get_size_padding_128(plaintext_len);
  int a_len = get_size_padding_128(ad_len);
  uint8_t *plaintext = malloc_wrap(pl_len);
  uint8_t *pl_pad = malloc_wrap(pl_len);
  uint8_t *len_ac = malloc_wrap(16);
  int ghash_data_len = a_len + pl_len + 16;
  uint8_t *ghash_data = malloc_wrap(ghash_data_len); // to store A_pad || c || len(a) || len(c);
  uint8_t *tag_data = malloc_wrap(get_size_padding_128(tag_len));

  memset(ghash_data, 0, ghash_data_len);
  memset(len_ac, 0, 16);
  memset(pl_pad, 0, pl_len);
  memcpy(pl_pad, plaintext, plaintext_len);
  
  memset(iv_gcm, 0, 16);

  aes_encrypt(H, key, iv_gcm, 16, NULL, 0, ECB, KEY_128);

  // calculate IV
  if (iv_len == 12) {
    memcpy(iv_gcm, iv, iv_len);
    iv_gcm[15] += 1;
  } else {
    memset(iv_gcm_tmp, 0, iv_gcm_tmp_len);
    memcpy(iv_gcm_tmp, iv, iv_len);
    for(int i = 0; i < 4; i++)
      iv_gcm_tmp[iv_gcm_tmp_len-1-i] = ((iv_len<<3) >> (i<<3)) & 0xff;
    memset(iv_gcm, 0,16);
    br_ghash_ctmul32(iv_gcm, H, iv_gcm_tmp, iv_gcm_tmp_len);
  }
  iv_gcm[15] += 1;
  // y0
  aes_encrypt(plaintext, key, aead_text, pl_len, iv_gcm, iv_len, CTR, KEY_128);
  
  for (int i = 15; i > 11; --i)
    len_ac[i] = ((plaintext_len*8) >> ((15-i)<<3) & 0xff);
  for (int i = 7; i > 3; --i)
    len_ac[i] = ((ad_len*8) >> ((7-i)<<3) & 0xff);

  memcpy(ghash_data, ad, ad_len);
  memcpy(ghash_data+a_len, aead_text, plaintext_len);
  memcpy(ghash_data+a_len+pl_len, len_ac, 16);

  memset(iv_gcm_tmp, 0, 16);
  br_ghash_ctmul32(iv_gcm_tmp, H, ghash_data, a_len+pl_len+16);

  iv_gcm[15] -= 1;

  aes_encrypt(tag_data, key, iv_gcm_tmp, 16, iv_gcm, 16, CTR, KEY_128);
  if(sis_strcmp(tag_data, aead_text + plaintext_len, tag_len)) {
    memcpy(aead_out, plaintext, plaintext_len);
    return AES_OK;
  } else return AES_AUTH_ERROR;

  free_wrap(tag_data);
  free_wrap(ghash_data);
  free_wrap(len_ac);
  free_wrap(pl_pad);
  free_wrap(plaintext);
  free_wrap(iv_gcm_tmp);
  free_wrap(iv_gcm);
  free_wrap(H);
  return AES_OK;
}



aes_status_t aes_aead_encrypt(uint8_t *aead_out, const uint8_t *key,
			      const uint8_t *ad, uint32_t ad_len,
			      const uint8_t *plaintext, uint32_t plaintext_len,
			      const uint8_t *iv, uint32_t iv_len,
			      enc_mode_t enc_mode, key_size_t key_size)
{
  return AES_OK;
}
