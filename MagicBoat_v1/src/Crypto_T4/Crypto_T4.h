#ifndef Crypto_T4_h
#define Crypto_T4_h

#include "Arduino.h"

// InputOutput  or just Input
#define __IO volatile
#define __I volatile
#define DCP_CH0SEMA_VALUE_MASK (0xFF0000U)
#define DCP_CH0STAT_ERROR_CODE_MASK (0xFF0000U)
#define DCP_HASH_BLOCK_SIZE 128
#define DCP_STAT_OTP_KEY_READY_MASK (0x10000000U)
#define DCP_KEY_INDEX_MASK (0x30U)
#define DCP_KEY_INDEX_SHIFT (4U)
#define DCP_KEY_INDEX(x) (((uint32_t)(((uint32_t)(x)) << DCP_KEY_INDEX_SHIFT)) & DCP_KEY_INDEX_MASK)

class Crypto_T4 {
public:

  enum _generic_status {
    kStatus_Success = 0,
    kStatus_Fail = 1,
    kStatus_ReadOnly = 2,
    kStatus_OutOfRange = 3,
    kStatus_InvalidArgument = 4,
    kStatus_Timeout = 5,
    kStatus_DCP_Again = 6,
  };


  typedef enum _dcp_ch_enable {
    kDCP_chDisable = 0U,    /*!< DCP channel disable */
    kDCP_ch0Enable = 1U,    /*!< DCP channel 0 enable */
    kDCP_ch1Enable = 2U,    /*!< DCP channel 1 enable */
    kDCP_ch2Enable = 4U,    /*!< DCP channel 2 enable */
    kDCP_ch3Enable = 8U,    /*!< DCP channel 3 enable */
    kDCP_chEnableAll = 15U, /*!< DCP channel enable all */
  } _dcp_ch_enable_t;

  typedef enum _dcp_channel {
    kDCP_Channel0 = (1u << 16), /*!< DCP channel 0. */
    kDCP_Channel1 = (1u << 17), /*!< DCP channel 1. */
    kDCP_Channel2 = (1u << 18), /*!< DCP channel 2. */
    kDCP_Channel3 = (1u << 19), /*!< DCP channel 3. */
  } dcp_channel_t;


  typedef enum _dcp_key_slot {
    kDCP_KeySlot0 = 0U,     /*!< DCP key slot 0. */
    kDCP_KeySlot1 = 1U,     /*!< DCP key slot 1. */
    kDCP_KeySlot2 = 2U,     /*!< DCP key slot 2.*/
    kDCP_KeySlot3 = 3U,     /*!< DCP key slot 3. */
    kDCP_OtpKey = 4U,       /*!< DCP OTP key. */
    kDCP_OtpUniqueKey = 5U, /*!< DCP unique OTP key. */
    kDCP_PayloadKey = 6U,   /*!< DCP payload key. */
  } dcp_key_slot_t;

  typedef enum _dcp_swap {
    kDCP_NoSwap = 0x0U,
    kDCP_KeyByteSwap = 0x40000U,
    kDCP_KeyWordSwap = 0x80000U,
    kDCP_InputByteSwap = 0x100000U,
    kDCP_InputWordSwap = 0x200000U,
    kDCP_OutputByteSwap = 0x400000U,
    kDCP_OutputWordSwap = 0x800000U,
  } dcp_swap_t;

  typedef enum _dcp_hash_algo_t {
    kDCP_Sha1,   /*!< SHA_1 */
    kDCP_Sha256, /*!< SHA_256 */
    kDCP_Crc32,  /*!< CRC_32 */
  } dcp_hash_algo_t;

  enum _dcp_hash_digest_len {
    kDCP_OutLenSha1 = 20u,
    kDCP_OutLenSha256 = 32u,
    kDCP_OutLenCrc32 = 4u,
  };

  enum _dcp_work_packet_bit_definitions {
    kDCP_CONTROL0_DECR_SEMAPHOR = 1u << 1, /* DECR_SEMAPHOR */
    kDCP_CONTROL0_ENABLE_HASH = 1u << 6,   /* ENABLE_HASH */
    kDCP_CONTROL0_HASH_INIT = 1u << 12,    /* HASH_INIT */
    kDCP_CONTROL0_HASH_TERM = 1u << 13,    /* HASH_TERM */
    kDCP_CONTROL1_HASH_SELECT_SHA256 = 2u << 16,
    kDCP_CONTROL1_HASH_SELECT_SHA1 = 0u << 16,
    kDCP_CONTROL1_HASH_SELECT_CRC32 = 1u << 16,
  };



  typedef struct _dcp_hash_ctx_t {
    uint32_t x[58];
  } dcp_hash_ctx_t;

  typedef union _dcp_hash_block {
    uint32_t w[DCP_HASH_BLOCK_SIZE / 4]; /*!< array of 32-bit words */
    uint8_t b[DCP_HASH_BLOCK_SIZE];      /*!< byte array */
  } dcp_hash_block_t;

  typedef enum _dcp_hash_algo_state {
    kDCP_StateHashInit = 1u, /*!< Init state. */
    kDCP_StateHashUpdate,    /*!< Update state. */
  } dcp_hash_algo_state_t;

  typedef struct _dcp_handle {
    dcp_channel_t channel;  /*!< Specify DCP channel. */
    dcp_key_slot_t keySlot; /*!< For operations with key (such as AES encryption/decryption), specify DCP key slot. */
    uint32_t swapConfig;    /*!< For configuration of key, input, output byte/word swap options */
    uint32_t keyWord[4];
    uint32_t iv[4];
  } dcp_handle_t;

  typedef struct _dcp_hash_ctx_internal {
    dcp_hash_block_t blk;        /*!< memory buffer. only full blocks are written to DCP during hash updates */
    size_t blksz;                /*!< number of valid bytes in memory buffer */
    dcp_hash_algo_t algo;        /*!< selected algorithm from the set of supported algorithms */
    dcp_hash_algo_state_t state; /*!< finite machine state of the hash software process */
    uint32_t fullMessageSize;    /*!< track message size */
    uint32_t ctrl0;              /*!< HASH_INIT and HASH_TERM flags */
    uint32_t runningHash[9];     /*!< running hash. up to SHA-256 plus size, that is 36 bytes. */
    dcp_handle_t *handle;
  } dcp_hash_ctx_internal_t;

  typedef struct _dcp_work_packet {
    uint32_t nextCmdAddress;
    uint32_t control0;
    uint32_t control1;
    uint32_t sourceBufferAddress;
    uint32_t destinationBufferAddress;
    uint32_t bufferSize;
    uint32_t payloadPointer;
    uint32_t status;
  } dcp_work_packet_t;


  /** DCP - Register Layout Typedef */
  typedef struct {
    __IO uint32_t CTRL; /**< DCP control register 0, offset: 0x0 */
    uint8_t RESERVED_0[12];
    __IO uint32_t STAT; /**< DCP status register, offset: 0x10 */
    uint8_t RESERVED_1[12];
    __IO uint32_t CHANNELCTRL; /**< DCP channel control register, offset: 0x20 */
    uint8_t RESERVED_2[12];
    __IO uint32_t CAPABILITY0; /**< DCP capability 0 register, offset: 0x30 */
    uint8_t RESERVED_3[12];
    __I uint32_t CAPABILITY1; /**< DCP capability 1 register, offset: 0x40 */
    uint8_t RESERVED_4[12];
    __IO uint32_t CONTEXT; /**< DCP context buffer pointer, offset: 0x50 */
    uint8_t RESERVED_5[12];
    __IO uint32_t KEY; /**< DCP key index, offset: 0x60 */
    uint8_t RESERVED_6[12];
    __IO uint32_t KEYDATA; /**< DCP key data, offset: 0x70 */
    uint8_t RESERVED_7[12];
    __I uint32_t PACKET0; /**< DCP work packet 0 status register, offset: 0x80 */
    uint8_t RESERVED_8[12];
    __I uint32_t PACKET1; /**< DCP work packet 1 status register, offset: 0x90 */
    uint8_t RESERVED_9[12];
    __I uint32_t PACKET2; /**< DCP work packet 2 status register, offset: 0xA0 */
    uint8_t RESERVED_10[12];
    __I uint32_t PACKET3; /**< DCP work packet 3 status register, offset: 0xB0 */
    uint8_t RESERVED_11[12];
    __I uint32_t PACKET4; /**< DCP work packet 4 status register, offset: 0xC0 */
    uint8_t RESERVED_12[12];
    __I uint32_t PACKET5; /**< DCP work packet 5 status register, offset: 0xD0 */
    uint8_t RESERVED_13[12];
    __I uint32_t PACKET6; /**< DCP work packet 6 status register, offset: 0xE0 */
    uint8_t RESERVED_14[28];
    __IO uint32_t CH0CMDPTR; /**< DCP channel 0 command pointer address register, offset: 0x100 */
    uint8_t RESERVED_15[12];
    __IO uint32_t CH0SEMA; /**< DCP channel 0 semaphore register, offset: 0x110 */
    uint8_t RESERVED_16[12];
    __IO uint32_t CH0STAT; /**< DCP channel 0 status register, offset: 0x120 */
    uint8_t RESERVED_17[12];
    __IO uint32_t CH0OPTS; /**< DCP channel 0 options register, offset: 0x130 */
    uint8_t RESERVED_18[12];
    __IO uint32_t CH1CMDPTR; /**< DCP channel 1 command pointer address register, offset: 0x140 */
    uint8_t RESERVED_19[12];
    __IO uint32_t CH1SEMA; /**< DCP channel 1 semaphore register, offset: 0x150 */
    uint8_t RESERVED_20[12];
    __IO uint32_t CH1STAT; /**< DCP channel 1 status register, offset: 0x160 */
    uint8_t RESERVED_21[12];
    __IO uint32_t CH1OPTS; /**< DCP channel 1 options register, offset: 0x170 */
    uint8_t RESERVED_22[12];
    __IO uint32_t CH2CMDPTR; /**< DCP channel 2 command pointer address register, offset: 0x180 */
    uint8_t RESERVED_23[12];
    __IO uint32_t CH2SEMA; /**< DCP channel 2 semaphore register, offset: 0x190 */
    uint8_t RESERVED_24[12];
    __IO uint32_t CH2STAT; /**< DCP channel 2 status register, offset: 0x1A0 */
    uint8_t RESERVED_25[12];
    __IO uint32_t CH2OPTS; /**< DCP channel 2 options register, offset: 0x1B0 */
    uint8_t RESERVED_26[12];
    __IO uint32_t CH3CMDPTR; /**< DCP channel 3 command pointer address register, offset: 0x1C0 */
    uint8_t RESERVED_27[12];
    __IO uint32_t CH3SEMA; /**< DCP channel 3 semaphore register, offset: 0x1D0 */
    uint8_t RESERVED_28[12];
    __IO uint32_t CH3STAT; /**< DCP channel 3 status register, offset: 0x1E0 */
    uint8_t RESERVED_29[12];
    __IO uint32_t CH3OPTS; /**< DCP channel 3 options register, offset: 0x1F0 */
    uint8_t RESERVED_30[524];
    __IO uint32_t DBGSELECT; /**< DCP debug select register, offset: 0x400 */
    uint8_t RESERVED_31[12];
    __I uint32_t DBGDATA; /**< DCP debug data register, offset: 0x410 */
    uint8_t RESERVED_32[12];
    __IO uint32_t PAGETABLE; /**< DCP page table register, offset: 0x420 */
    uint8_t RESERVED_33[12];
    __I uint32_t VERSION; /**< DCP version register, offset: 0x430 */
  } DCP_Type;

#define DCP ((DCP_Type *)0x402FC000)

  void dcp_reverse_and_copy(uint8_t *src, uint8_t *dest, size_t src_len);
  uint32_t dcp_get_channel_status(dcp_channel_t channel);
  void dcp_clear_status();
  void dcp_clear_channel_status(uint32_t mask);
  uint32_t DCP_WaitForChannelComplete(dcp_handle_t *handle);
  uint32_t dcp_schedule_work(dcp_handle_t *handle, dcp_work_packet_t *dcpPacket);
  uint32_t dcp_hash_update_non_blocking(dcp_hash_ctx_internal_t *ctxInternal, dcp_work_packet_t *dcpPacket, const uint8_t *msg, size_t size);
  void dcp_hash_update(dcp_hash_ctx_internal_t *ctxInternal, const uint8_t *msg, size_t size);
  void dcp_hash_process_message_data(dcp_hash_ctx_internal_t *ctxInternal, const uint8_t *message, size_t messageSize);
  void DCP_HASH_Init(dcp_handle_t *handle, dcp_hash_ctx_t *ctx, dcp_hash_algo_t algo);
  void DCP_HASH_Update(dcp_hash_ctx_t *ctx, const uint8_t *input, size_t inputSize);
  void DCP_HASH_Finish(dcp_hash_ctx_t *ctx, uint8_t *output);
  void dcp_init();
  void prhash(unsigned char *h, int n);
  void do_sha256();
  void do_crc32();
  uint32_t dcp_aes_set_sram_based_key(dcp_handle_t *handle, const uint8_t *key);
  uint32_t DCP_AES_EncryptCbcNonBlocking(dcp_handle_t *handle, dcp_work_packet_t *dcpPacket, const uint8_t *plaintext, uint8_t *ciphertext, size_t size, const uint8_t *iv);
  uint32_t DCP_AES_DecryptCbcNonBlocking(dcp_handle_t *handle, dcp_work_packet_t *dcpPacket, const uint8_t *ciphertext, uint8_t *plaintext, size_t size, const uint8_t *iv);
  uint32_t DCP_AES_SetKey(dcp_handle_t *handle, const uint8_t *key, size_t keySize);
  void DCP_AES_EncryptCbc(dcp_handle_t *handle, const uint8_t *plaintext, uint8_t *ciphertext, size_t size, const uint8_t iv[16]);
  void DCP_AES_DecryptCbc(dcp_handle_t *handle, const uint8_t *ciphertext, uint8_t *plaintext, size_t size, const uint8_t iv[16]);
  void encryptAESMess(uint8_t keyAes128[], uint8_t ive[], uint8_t plainTextIn[], uint8_t *cipherTextOut);
  void decryptAESMess(uint8_t keyAes128[], uint8_t ive[], uint8_t cipherTextIn[], uint8_t *plainTextOut);
};

#endif