/**
  ******************************************************************************
  * @file    ECC_KeyGeneration_SignatureVerification/main.c
  * @author  MCD Application Team
  * @version V2.0.6
  * @date    25-June-2013
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "crypto.h"

/** @addtogroup STM32_Crypto_Examples
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*Message to sign*/
const uint8_t InputMessage[] = "abc";

/* DRBG type */
/* Allows selecting automatically the hardware or software random number generator*/
#ifndef USE_HW_RNG
int32_t DRBGtype = C_DRBG_AES128;
#else
int32_t DRBGtype = C_HW_RNG;
#endif

/******************************************************************************/
/******** Parameters for Elliptic Curve P-192 SHA-256 from FIPS 186-3**********/
/******************************************************************************/
const uint8_t P_192_a[] =
  {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC
  };
const uint8_t P_192_b[] =
  {
    0x64, 0x21, 0x05, 0x19, 0xE5, 0x9C, 0x80, 0xE7, 0x0F, 0xA7, 0xE9, 0xAB, 0x72,
    0x24, 0x30, 0x49, 0xFE, 0xB8, 0xDE, 0xEC, 0xC1, 0x46, 0xB9, 0xB1
  };
const uint8_t P_192_p[] =
  {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
  };
const uint8_t P_192_n[] =
  {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x99,
    0xDE, 0xF8, 0x36, 0x14, 0x6B, 0xC9, 0xB1, 0xB4, 0xD2, 0x28, 0x31
  };
const uint8_t P_192_Gx[] =
  {
    0x18, 0x8D, 0xA8, 0x0E, 0xB0, 0x30, 0x90, 0xF6, 0x7C, 0xBF, 0x20, 0xEB, 0x43,
    0xA1, 0x88, 0x00, 0xF4, 0xFF, 0x0A, 0xFD, 0x82, 0xFF, 0x10, 0x12
  };
const uint8_t P_192_Gy[] =
  {
    0x07, 0x19, 0x2B, 0x95, 0xFF, 0xC8, 0xDA, 0x78, 0x63, 0x10, 0x11, 0xED, 0x6B,
    0x24, 0xCD, 0xD5, 0x73, 0xF9, 0x77, 0xA1, 0x1E, 0x79, 0x48, 0x11
  };

/******************************************************************************/
/******** Parameters for Elliptic Curve P-256 SHA-256 from FIPS 186-3**********/
/******************************************************************************/
const uint8_t P_256_a[] =
  {
    0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC
  };
const uint8_t P_256_b[] =
  {
    0x5a, 0xc6, 0x35, 0xd8, 0xaa, 0x3a, 0x93, 0xe7, 0xb3, 0xeb, 0xbd, 0x55, 0x76,
    0x98, 0x86, 0xbc, 0x65, 0x1d, 0x06, 0xb0, 0xcc, 0x53, 0xb0, 0xf6, 0x3b, 0xce,
    0x3c, 0x3e, 0x27, 0xd2, 0x60, 0x4b
  };
const uint8_t P_256_p[] =
  {
    0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
  };
const uint8_t P_256_n[] =
  {
    0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xBC, 0xE6, 0xFA, 0xAD, 0xA7, 0x17, 0x9E, 0x84, 0xF3, 0xB9,
    0xCA, 0xC2, 0xFC, 0x63, 0x25, 0x51
  };
const uint8_t P_256_Gx[] =
  {
    0x6B, 0x17, 0xD1, 0xF2, 0xE1, 0x2C, 0x42, 0x47, 0xF8, 0xBC, 0xE6, 0xE5, 0x63,
    0xA4, 0x40, 0xF2, 0x77, 0x03, 0x7D, 0x81, 0x2D, 0xEB, 0x33, 0xA0, 0xF4, 0xA1,
    0x39, 0x45, 0xD8, 0x98, 0xC2, 0x96
  };
const uint8_t P_256_Gy[] =
  {
    0x4F, 0xE3, 0x42, 0xE2, 0xFE, 0x1A, 0x7F, 0x9B, 0x8E, 0xE7, 0xEB, 0x4A, 0x7C,
    0x0F, 0x9E, 0x16, 0x2B, 0xCE, 0x33, 0x57, 0x6B, 0x31, 0x5E, 0xCE, 0xCB, 0xB6,
    0x40, 0x68, 0x37, 0xBF, 0x51, 0xF5
  };

/******************************************************************************/
/********** Parameters for Elliptic Curve P-384 SHA-256 from FIPS 186-3********/
/******************************************************************************/
const uint8_t P_384_a[] =
  {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFC
  };

const uint8_t P_384_b[] =
  {
    0xb3, 0x31, 0x2f, 0xa7, 0xe2, 0x3e, 0xe7, 0xe4, 0x98, 0x8e, 0x05, 0x6b, 0xe3,
    0xf8, 0x2d, 0x19, 0x18, 0x1d, 0x9c, 0x6e, 0xfe, 0x81, 0x41, 0x12, 0x03, 0x14,
    0x08, 0x8f, 0x50, 0x13, 0x87, 0x5a, 0xc6, 0x56, 0x39, 0x8d, 0x8a, 0x2e, 0xd1,
    0x9d, 0x2a, 0x85, 0xc8, 0xed, 0xd3, 0xec, 0x2a, 0xef
  };

const uint8_t P_384_p[] =
  {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF
  };

const uint8_t P_384_n[] =
  {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC7, 0x63,
    0x4D, 0x81, 0xF4, 0x37, 0x2D, 0xDF, 0x58, 0x1A, 0x0D, 0xB2, 0x48, 0xB0, 0xA7,
    0x7A, 0xEC, 0xEC, 0x19, 0x6A, 0xCC, 0xC5, 0x29, 0x73
  };

const uint8_t P_384_Gx[] =
  {
    0xAA, 0x87, 0xCA, 0x22, 0xBE, 0x8B, 0x05, 0x37, 0x8E, 0xB1, 0xC7, 0x1E, 0xF3,
    0x20, 0xAD, 0x74, 0x6E, 0x1D, 0x3B, 0x62, 0x8B, 0xA7, 0x9B, 0x98, 0x59, 0xF7,
    0x41, 0xE0, 0x82, 0x54, 0x2A, 0x38, 0x55, 0x02, 0xF2, 0x5D, 0xBF, 0x55, 0x29,
    0x6C, 0x3A, 0x54, 0x5E, 0x38, 0x72, 0x76, 0x0A, 0xB7
  };

const uint8_t P_384_Gy[] =
  {
    0x36, 0x17, 0xDE, 0x4A, 0x96, 0x26, 0x2C, 0x6F, 0x5D, 0x9E, 0x98, 0xBF, 0x92,
    0x92, 0xDC, 0x29, 0xF8, 0xF4, 0x1D, 0xBD, 0x28, 0x9A, 0x14, 0x7C, 0xE9, 0xDA,
    0x31, 0x13, 0xB5, 0xF0, 0xB8, 0xC0, 0x0A, 0x60, 0xB1, 0xCE, 0x1D, 0x7E, 0x81,
    0x9D, 0x7A, 0x43, 0x1D, 0x7C, 0x90, 0xEA, 0x0E, 0x5F
  };

/* Private function prototypes -----------------------------------------------*/
int32_t STM32_SHA256_HASH_DigestCompute(uint8_t* InputMessage,
                                  uint32_t InputMessageLength,
                                  uint8_t *MessageDigest,
                                  int32_t* MessageDigestLength);


int32_t STM32_Init_RNG_for_Sign( RNGstate_stt *P_pRNGstate);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c, system_stm32l1xx.c, system_stm32f0xx.c, 
       system_stm32f2xx.c, system_stm32f30x.c, system_stm32f37x.c, or
       system_stm32f4xx.c file depending on device.
     */

  /* size of message to be signed*/
  uint32_t InputLength = NULL;

  /* counter to test different ECC key sizes*/
  __IO int32_t param_ecc_counter = 0x00;

  /* Structure that will keep the random state*/
  RNGstate_stt RNGstate;

  /* Structure that will contain the curve's parameter */
  EC_stt EC_st;

  /* structure to store the preallocated buffer for crypto computation*/
  membuf_stt Crypto_Buffer;

  /* Actual preallocated buffer */
  uint8_t preallocated_buffer[3584];

  /* Structure that will contain the public key, please note that the public key
   * is just a point on the curve, hence the name ECpoint_stt 
   */
  ECpoint_stt *PubKey = NULL;

  /* Private Key Structure */
  ECCprivKey_stt *PrivKey = NULL;

  /* Structure that will contain the ECDSA signature */
  ECDSAsignature_stt *sign = NULL;

  /* Structure context for the signature generation */
  ECDSAsignCtx_stt signCtx;

  /* Structure context used to call the ECDSAverify */
  ECDSAverifyCtx_stt verctx;

  /* Used to check the returned values of the SHA function */
  int32_t status = HASH_SUCCESS;

  /* Message Digest buffer */
  uint8_t MessageDigest[CRL_SHA256_SIZE];

  /* Message Digest length */
  int32_t MessageDigestLength = 0;

  /* DeInitialize STM32 Cryptographic Library */
  Crypto_DeInit();
  
  /* message length */
  InputLength = (sizeof(InputMessage) - 1);

  for (param_ecc_counter = 0; param_ecc_counter < 3; param_ecc_counter++)
  {
    EC_st.pmB = NULL;
    EC_st.mBsize = 0;

    switch (param_ecc_counter)
    {
      case 0:
        /* Initialize the ECC curve structure, with all the parameters */
        EC_st.pmA = P_192_a;
        EC_st.pmP = P_192_p;
        EC_st.pmN = P_192_n;
        EC_st.pmGx = P_192_Gx;
        EC_st.pmGy = P_192_Gy;
        EC_st.mAsize = sizeof(P_192_a);
        EC_st.mNsize = sizeof(P_192_n);
        EC_st.mPsize = sizeof(P_192_p);
        EC_st.mGxsize = sizeof(P_192_Gx);
        EC_st.mGysize = sizeof(P_192_Gy);
        break;

      case 1:
        /* Initialize the ECC curve structure, with all the parameters */
        EC_st.pmA = P_256_a;
        EC_st.pmP = P_256_p;
        EC_st.pmN = P_256_n;
        EC_st.pmGx = P_256_Gx;
        EC_st.pmGy = P_256_Gy;
        EC_st.mAsize = sizeof(P_256_a);
        EC_st.mNsize = sizeof(P_256_n);
        EC_st.mPsize = sizeof(P_256_p);
        EC_st.mGxsize = sizeof(P_256_Gx);
        EC_st.mGysize = sizeof(P_256_Gy);
        break;
      case 2:
        /* Initialize the ECC curve structure, with all the parameters */
        EC_st.pmA = P_384_a;
        EC_st.pmP = P_384_p;
        EC_st.pmN = P_384_n;
        EC_st.pmGx = P_384_Gx;
        EC_st.pmGy = P_384_Gy;
        EC_st.mAsize = sizeof(P_384_a);
        EC_st.mNsize = sizeof(P_384_n);
        EC_st.mPsize = sizeof(P_384_p);
        EC_st.mGxsize = sizeof(P_384_Gx);
        EC_st.mGysize = sizeof(P_384_Gy);
        break;

      default:
        continue;
    }

    /* First hash the message to have a valid digest that will be signed through
     * ECDSA 
     */
    status = STM32_SHA256_HASH_DigestCompute((uint8_t*)InputMessage,
                                       InputLength,
                                       (uint8_t*)MessageDigest,
                                       &MessageDigestLength);
    if (status == HASH_SUCCESS)
    {
      /* We prepare the memory buffer strucure */
      Crypto_Buffer.pmBuf =  preallocated_buffer;
      Crypto_Buffer.mUsed = 0;
      Crypto_Buffer.mSize = sizeof(preallocated_buffer);

      /* Init the Elliptic Curve, passing the required memory */
      /* Note: This is not a temporary buffer, the memory inside EC_stt_buf is linked to EC_st *
       As long as there's need to use EC_st the content of EC_stt_buf should not be altered */
      status = ECCinitEC(&EC_st, &Crypto_Buffer );

      if (status == ECC_SUCCESS)
      {
        /* Init PubKey and Privkey */
        status = ECCinitPoint(&PubKey, &EC_st, &Crypto_Buffer);

        if (status == ECC_SUCCESS)
        {
          status = ECCinitPrivKey(&PrivKey, &EC_st, &Crypto_Buffer);

          if (status == ECC_SUCCESS)
          {
            /* Init the random engine and select hardware random number generator
             * (TRNG) or software random number generator (PRNG) according to the 
             * target platform 
             */
            status = STM32_Init_RNG_for_Sign( &RNGstate);

            if (status == ECC_SUCCESS)
            {
              /* Generate ECC key pair */
              status = ECCkeyGen(PrivKey, PubKey, &RNGstate, &EC_st, &Crypto_Buffer);

              if (status == ECC_SUCCESS)
              {
                /* Initialize an ECDSA signature structure */
                status = ECDSAinitSign(&sign, &EC_st, &Crypto_Buffer);

                if (status == ECC_SUCCESS)
                {
                  /* Prepare the structure for the ECDSA signature verification */
                  signCtx.pmEC = &EC_st;
                  signCtx.pmPrivKey = PrivKey;
                  signCtx.pmRNG = &RNGstate;

                  status = ECDSAsign(MessageDigest, CRL_SHA256_SIZE, sign, &signCtx, &Crypto_Buffer);
                  if (status == ECC_SUCCESS)
                  {
                    /* We now have the signature of the message, we try to verify it */
                    verctx.pmEC = &EC_st;
                    verctx.pmPubKey = PubKey;

                    /* Verify the signature with the digest */
                    status = ECDSAverify(MessageDigest, CRL_SHA256_SIZE, sign, &verctx, &Crypto_Buffer);

                    if (status == SIGNATURE_VALID )
                    {
                      /* Before returning clean the memory by freeing the
                       * signature and the public key 
                       */
                      RNGfree(&RNGstate);
                      ECCfreePoint(&PubKey, &Crypto_Buffer);
                      ECCfreePrivKey(&PrivKey, &Crypto_Buffer);
                      ECDSAfreeSign(&sign, &Crypto_Buffer);
                      ECCfreeEC(&EC_st, &Crypto_Buffer);
                      
                    }
                    else
                    {
                      /* Add application traintment in case of ECC verification is not valid.
                       * Possible return values are: ERR_DYNAMIC_ALLOCATION_FAILED, ECC_ERR_BAD_PARAMETER,
                       * ECC_ERR_BAD_CONTEXT, ECC_ERR_MISSING_EC_PARAMETER, MATH_ERR_BIGNUM_OVERFLOW, 
                       * SIGNATURE_INVALID
                       */

                    }
                  }
                  else
                  {
                    /* Add application traintment in case of ECC signature is not valid.
                     * Possible return values are: ECC_ERR_BAD_PARAMETER, ECC_ERR_BAD_CONTEXT,
                     * ECC_ERR_MISSING_EC_PARAMETER 
                     */

                  }
                }
                else
                {
                  /* Add application traintment in case of Initialize an ECDSA signature
                     * structure is failed. Possible return values are: ECC_ERR_BAD_PARAMETER,
                     * ECC_ERR_BAD_CONTEXT, ERR_DYNAMIC_ALLOCATION_FAILED
                     */

                }
              }
              else
              {
                /* Add application traintment in case of Generate ECC key pair is failed.
                  * Possible return values are : ERR_DYNAMIC_ALLOCATION_FAILED, ECC_ERR_BAD_PARAMETER
                  * RNG_ERR_UNINIT_STATE, ECC_ERR_MISSING_EC_PARAMETER, MATH_ERR_BIGNUM_OVERFLOW 
                  */

              }
            }
            else
            {
              /* Add application traintment in case of Init the random engine is failed.
                * Possible return value is RNG_ERR_UNINIT_STATE
                */

            }
          }
          else
          {
            /* Add application traintment in case of ECC initialize Privkey is failed.
             * Possible return values are : ECC_ERR_BAD_PARAMETER, ECC_ERR_BAD_CONTEXT,
             * ERR_DYNAMIC_ALLOCATION_FAILED 
             */

          }
        }
        else
        {
          /* Add application traintment in case of ECC initialize PubKey is failed.
           * Possible return values are : ECC_ERR_BAD_PARAMETER, ECC_ERR_BAD_CONTEXT
           * ERR_DYNAMIC_ALLOCATION_FAILED 
           */

        }
      }
      else
      {
        /* Add application traintment in case of Init the Elliptic Curve is failed.
         * Possible return values are: ECC_ERR_BAD_PARAMETER, ECC_ERR_BAD_CONTEXT,
         * ERR_DYNAMIC_ALLOCATION_FAILED 
         */

      }
    }
    else
    {
      /* Add application traintment in case of hash not success possible values of status:
       * HASH_ERR_BAD_PARAMETER, HASH_ERR_BAD_CONTEXT, HASH_ERR_BAD_OPERATION
       */

    }
  }
  /* Infinite loop */
  while (1)
  {}


}

/**
  * @brief  SHA256 HASH digest compute example.
  * @param  InputMessage: pointer to input message to be hashed.
  * @param  InputMessageLength: input data message length in byte.
  * @param  MessageDigest: pointer to output parameter that will handle message digest
  * @param  MessageDigestLength: pointer to output digest length.
  * @retval error status: can be HASH_SUCCESS if success or one of
  *         HASH_ERR_BAD_PARAMETER, HASH_ERR_BAD_CONTEXT,
  *         HASH_ERR_BAD_OPERATION if error occured.
  */
int32_t STM32_SHA256_HASH_DigestCompute(uint8_t* InputMessage, uint32_t InputMessageLength,
                                  uint8_t *MessageDigest, int32_t* MessageDigestLength)
{
  SHA256ctx_stt P_pSHA256ctx;
  uint32_t error_status = HASH_SUCCESS;

  /* Set the size of the desired hash digest */
  P_pSHA256ctx.mTagSize = CRL_SHA256_SIZE;

  /* Set flag field to default value */
  P_pSHA256ctx.mFlags = E_HASH_DEFAULT;

  error_status = SHA256_Init(&P_pSHA256ctx);

  /* check for initialization errors */
  if (error_status == HASH_SUCCESS)
  {
    /* Add data to be hashed */
    error_status = SHA256_Append(&P_pSHA256ctx,
                                 InputMessage,
                                 InputMessageLength);

    if (error_status == HASH_SUCCESS)
    {
      /* retrieve */
      error_status = SHA256_Finish(&P_pSHA256ctx, MessageDigest, MessageDigestLength);
    }
  }

  return error_status;
}

/**
  * @brief  initialise the RNG in order to be used during the ECC signature.
  *           need to indicate if it is an hardware or software RNG.
  * @param  P_pRNGstate: pointer to input RNG structure type.
  * @retval error status: can be ECC_SUCCESS if success or RNG_ERR_UNINIT_STATE
  *                        if error occured.
  */
int32_t STM32_Init_RNG_for_Sign(RNGstate_stt *P_pRNGstate)
{

  /* Structure for the parmeters of initialization */
  RNGinitInput_stt RNGinit_st;

  /* Nonce */
  uint8_t nonce[4] = {0, 1, 2, 3};

  uint32_t error_status = RNG_ERR_UNINIT_STATE;

  /* String of entropy */
  uint8_t entropy_data[32] =
    {
      0x9d, 0x20, 0x1a, 0x18, 0x9b, 0x6d, 0x1a, 0xa7, 0x0e, 0x79, 0x57, 0x6f, 0x36,
      0xb6, 0xaa, 0x88, 0x55, 0xfd, 0x4a, 0x7f, 0x97, 0xe9, 0x71, 0x69, 0xb6, 0x60,
      0x88, 0x78, 0xe1, 0x9c, 0x8b, 0xa5
    };

  /* Initialize the RNGinit structure */
  RNGinit_st.pmEntropyData = entropy_data;
  RNGinit_st.mEntropyDataSize = sizeof(entropy_data);
  RNGinit_st.pmNonce = nonce;
  RNGinit_st.mNonceSize = sizeof(nonce);

  /* There is no personalization data in this case */
  RNGinit_st.mPersDataSize = 0;
  RNGinit_st.pmPersData = NULL;

  /* Init the random engine */
  error_status =  RNGinit(&RNGinit_st, DRBGtype,  P_pRNGstate);
  if ( error_status != 0)
  {
    error_status = RNG_ERR_UNINIT_STATE;
  }
  else
  {
    error_status = RNG_SUCCESS;
  }
  return error_status;

}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
