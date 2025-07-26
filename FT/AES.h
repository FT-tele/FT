#include <Arduino.h>


//-------------------------------------------------------local
static void pad(uint8_t *input, uint8_t input_len, uint8_t padded_len);
static void unpad(uint8_t *input, uint8_t *input_len, uint8_t *output_len);


//-------------------------------------------------------api
//-----------------------------AES
uint8_t aes_cipher(uint8_t *source, uint8_t source_len, uint8_t *send, uint8_t *ShareKey, uint8_t *iv, uint8_t *cipheredLen);
uint8_t aes_decipher(uint8_t *received, uint8_t received_len, uint8_t *decipheredData, uint8_t *ShareKey, uint8_t *iv, uint8_t *decipheredLen);
void GenerateKeyPairs(uint8_t *myPrivate, uint8_t *myPublic);
void GenerateSharedKey(uint8_t *myPrivate, uint8_t *receivedPublic, uint8_t *aesKey);

//-----------------------------RSA
 