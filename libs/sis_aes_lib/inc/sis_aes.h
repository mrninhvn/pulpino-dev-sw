#ifndef _SIS_AES_H_
#define _SIS_AES_H_

#include <stdint.h>
#include <stddef.h>
extern uint8_t* amalloc(int size);
extern void afree(uint8_t * ptr);
extern uint32_t get_scratchpad_ptr();
#define DISABLE_MALLOC

#ifdef DISABLE_MALLOC
#define malloc_wrap amalloc
#define free_wrap afree
#else
#define malloc_wrap malloc
#define free_wrap free
#endif

typedef enum {FIXED=0, INCR, WRAP} axi_burst_t;

typedef enum {KEY_128=0, KEY_192, KEY_256} key_size_t;

typedef enum {ECB=0, CTR, CBC, CCM, GCM, RAN, DE_CTR, DE_CBC, DE_CCM, DE_GCM, OFB, EAX} enc_mode_t;
typedef enum {AES_OK=0, AES_CFG_ERROR, AES_AUTH_ERROR} aes_status_t;
extern uint32_t aes_get_status();
typedef struct {
  uint8_t mode;
  uint8_t key_size;
  uint8_t loop;
} aes_config_t;

extern aes_status_t aes_set_config(key_size_t key_size, uint32_t enable, uint32_t burst_len,
				   enc_mode_t enc_mode, uint32_t loop);

extern aes_status_t aes_encrypt(uint8_t *ciphertext, const uint8_t *key, const uint8_t *plaintext,
				uint32_t data_len, const uint8_t *iv, uint32_t iv_len,
				enc_mode_t enc_mode, key_size_t key_size);

extern aes_status_t aes_aead_encrypt(uint8_t *aead_out, const uint8_t *key,
				     const uint8_t *ad, uint32_t ad_len,
				     const uint8_t *plaintext, uint32_t plaintext_len,
				     const uint8_t *iv, uint32_t iv_len,
				     enc_mode_t enc_mode, key_size_t key_size);

extern aes_status_t aes_aead_ccm(uint8_t *aead_out, const uint8_t *key,
				 const uint8_t *ad, uint32_t ad_len,
				 const uint8_t *plaintext, uint32_t plaintext_len,
				 const uint8_t *iv, uint32_t iv_len,
				 uint32_t tag_len,
				 enc_mode_t enc_mode, key_size_t key_size);
extern aes_status_t aes_aead_gcm(uint8_t *aead_out, const uint8_t *key,
				 const uint8_t *ad, uint32_t ad_len,
				 const uint8_t *plaintext, uint32_t plaintext_len,
				 const uint8_t *iv, uint32_t iv_len,
				 uint32_t tag_len,
				 enc_mode_t enc_mode, key_size_t key_size);
extern aes_status_t aes_aead_gcm_decrypt(uint8_t *aead_out, const uint8_t *key,
				 const uint8_t *ad, uint32_t ad_len,
				 const uint8_t *aead_text, uint32_t aead_len,
				 const uint8_t *iv, uint32_t iv_len,
				 uint32_t tag_len,
				 enc_mode_t enc_mode, key_size_t key_size);
extern aes_status_t aes_aead_ccm_decrypt(uint8_t *aead_out, const uint8_t *key,
			  const uint8_t *ad, uint32_t ad_len,
			  const uint8_t *ciphertext, uint32_t cipher_len,
			  const uint8_t *iv, uint32_t iv_len,
			  uint32_t tag_len,
			  enc_mode_t enc_mode, key_size_t key_size);
#endif
