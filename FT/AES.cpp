
#include <Curve25519.h>
#include <mbedtls/aes.h>
#include "AES.h"
#include "VariableDefine.h"




//-----------------------------------RSA

#include <Arduino.h>
#include <LittleFS.h>
#include <mbedtls/rsa.h>
#include <mbedtls/pk.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>

//-----------------------------------test

uint8_t myLen = 0;
void randByteArray(uint8_t *byteArray) {

  //uint8_t length = random(230, 240);  // Random length between 10 and 50
  uint8_t length = random(1, 256);  // Random length between 10 and 50
  myLen = length;
  for (uint8_t i = 0; i < length; i++) {
    byteArray[i] = random(0, PKT);  // Random value between 0 and 255
  }
}

//-------------------------------------------------------api

void GenerateKeyPairs(uint8_t *myPrivate, uint8_t *myPublic) {
  Curve25519::dh1(myPrivate, myPublic);
  Serial.println("\nGenerateKeyPairs myPrivate:\t");
  //for (int i = 0; i < 32; i++) //Serial.printf(" %02X", myPrivate[i]);
  //Serial.printf("\nGenerateKeyPairs myPublic:\t");
  //for (int i = 0; i < 32; i++) //Serial.printf(" %02X", myPublic[i]);
};


void GenerateSharedKey(uint8_t *myPrivate, uint8_t *receivedPublic, uint8_t *aesKey) {
  //would clean myPrivate[KEY]
  Curve25519::dh2(myPrivate, receivedPublic);
  //Serial.print("sharedKey generated : \t");
  memcpy(aesKey, myPrivate, KEY);
  //Serial.printf("\GenerateSharedKey aesKey:\t");
  //for (int i = 0; i < 32; i++) //Serial.printf(" %02X", aesKey[i]);
};


/* Apply PKCS#7 padding */
static void pad(byte *input, uint8_t input_len, uint8_t padded_len) {
  if (padded_len <= input_len) {
    Serial.println("Error: padded length must be greater than input length.");
    return;
  }

  uint8_t padding_value = padded_len - input_len;

  for (uint8_t i = input_len; i < padded_len; ++i) {
    input[i] = padding_value;
  }
}

/* Remove PKCS#7 padding */
static void unpad(byte *input, uint8_t *input_len, uint8_t *output_len) {
  if (*input_len == 0) {
    Serial.println("Error: Input length is zero.");
    return;
  }

  uint8_t padding_value = input[*input_len - 1];

  if (padding_value > 0 && padding_value <= *input_len) {
    for (uint8_t i = *input_len - padding_value; i < *input_len; ++i) {
      if (input[i] != padding_value) {
        Serial.println("Error: Invalid padding detected.");
        return;
      }
    }

    *input_len -= padding_value;
    *output_len = *input_len;
    //Serial.printf("DecipherLength is %d, padding_value is %d\n", *output_len, padding_value);
  } else {
    Serial.println("Error: Padding value is invalid. Possibly missing padding.");
    *output_len = *input_len;
  }
}

/* AES Encryption */
uint8_t aes_cipher(byte *source, uint8_t source_len, byte *send, byte *ShareKey, byte *iv, uint8_t *cipheredLen) {

  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);

  uint8_t padded_len = ((source_len + HKEY - 1) / HKEY) * HKEY;
  *cipheredLen = padded_len;

  byte *padded_input = (byte *)malloc(padded_len);
  if (padded_input == NULL) {
    Serial.println("Memory allocation failed");
    return -1;
  }

  memset(padded_input, '\0', padded_len);
  memcpy(padded_input, source, source_len);
  pad(padded_input, source_len, padded_len);

  uint8_t ret = mbedtls_aes_setkey_enc(&aes, ShareKey, PKT);
  if (ret != 0) {
    //Serial.printf("mbedtls_aes_setkey_enc failed with error: %d\n", ret);
  }

  ret = mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, padded_len, iv, padded_input, send);
  if (ret != 0) {
    //Serial.printf("mbedtls_aes_crypt_cbc (encrypt) failed with error: %d\n", ret);
  }

  mbedtls_aes_free(&aes);
  free(padded_input);
  return ret;
}

/* AES Decryption */
uint8_t aes_decipher(byte *received, uint8_t received_len, byte *decipheredData, byte *ShareKey, byte *iv, uint8_t *decipheredLen) {
  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);

  uint8_t ret = mbedtls_aes_setkey_dec(&aes, ShareKey, PKT);
  if (ret != 0) {
    //Serial.printf("mbedtls_aes_setkey_dec failed with error: %d\n", ret);
  }

  ret = mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, received_len, iv, received, decipheredData);
  if (ret != 0) {
    //Serial.printf("mbedtls_aes_crypt_cbc (decrypt) failed with error: %d\n", ret);
  } else {
    //Serial.println("Decryption successful");
    uint8_t deciphered_len = received_len;
    unpad(decipheredData, &deciphered_len, decipheredLen);
  }

  mbedtls_aes_free(&aes);
  return ret;
}




 
 