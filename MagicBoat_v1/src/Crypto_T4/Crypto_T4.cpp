#include "Crypto_T4.h"

void Crypto_T4::dcp_reverse_and_copy(uint8_t *src, uint8_t *dest, size_t src_len)
{
  for (uint32_t i = 0; i < src_len; i++)
  {
    dest[i] = src[src_len - 1 - i];
  }
}


uint32_t Crypto_T4::dcp_get_channel_status( dcp_channel_t channel)
{
  uint32_t statReg = 0;
  uint32_t semaReg = 0;
  uint32_t status = kStatus_Fail;

  switch (channel)
  {
    case kDCP_Channel0:
      statReg = DCP->CH0STAT;
      semaReg = DCP->CH0SEMA;
      break;

    case kDCP_Channel1:
      statReg = DCP->CH1STAT;
      semaReg = DCP->CH1SEMA;
      break;

    case kDCP_Channel2:
      statReg = DCP->CH2STAT;
      semaReg = DCP->CH2SEMA;
      break;

    case kDCP_Channel3:
      statReg = DCP->CH3STAT;
      semaReg = DCP->CH3SEMA;
      break;

    default:
      break;
  }

  if (!((semaReg & DCP_CH0SEMA_VALUE_MASK) || (statReg & DCP_CH0STAT_ERROR_CODE_MASK)))
  {
    status = kStatus_Success;
  }

  return status;
}

void Crypto_T4::dcp_clear_status()
{
  volatile uint32_t *dcpStatClrPtr = &DCP->STAT + 2u;
  *dcpStatClrPtr = 0xFFu;
}

void Crypto_T4::dcp_clear_channel_status( uint32_t mask)
{
  volatile uint32_t *chStatClrPtr;

  if (mask & kDCP_Channel0)
  {
    chStatClrPtr = &DCP->CH0STAT + 2u;
    *chStatClrPtr = 0xFFu;
  }
  if (mask & kDCP_Channel1)
  {
    chStatClrPtr = &DCP->CH1STAT + 2u;
    *chStatClrPtr = 0xFFu;
  }
  if (mask & kDCP_Channel2)
  {
    chStatClrPtr = &DCP->CH2STAT + 2u;
    *chStatClrPtr = 0xFFu;
  }
  if (mask & kDCP_Channel3)
  {
    chStatClrPtr = &DCP->CH3STAT + 2u;
    *chStatClrPtr = 0xFFu;
  }
}


uint32_t Crypto_T4::DCP_WaitForChannelComplete( dcp_handle_t *handle)
{
  /* wait if our channel is still active */
  while ((DCP->STAT & (uint32_t)handle->channel) == handle->channel)
  {
  }

  if (dcp_get_channel_status(handle->channel) != kStatus_Success)
  {
    dcp_clear_status();
    dcp_clear_channel_status(handle->channel);
    return kStatus_Fail;
  }

  return kStatus_Success;
}


uint32_t Crypto_T4::dcp_schedule_work( dcp_handle_t *handle, dcp_work_packet_t *dcpPacket)
{
  uint32_t status;

  /* check if our channel is active */
  if ((DCP->STAT & (uint32_t)handle->channel) != handle->channel)
  {
    noInterrupts();

    /* re-check if our channel is still available */
    if ((DCP->STAT & (uint32_t)handle->channel) == 0)
    {
      volatile uint32_t *cmdptr = NULL;
      volatile uint32_t *chsema = NULL;

      switch (handle->channel)
      {
        case kDCP_Channel0:
          cmdptr = &DCP->CH0CMDPTR;
          chsema = &DCP->CH0SEMA;
          break;

        case kDCP_Channel1:
          cmdptr = &DCP->CH1CMDPTR;
          chsema = &DCP->CH1SEMA;
          break;

        case kDCP_Channel2:
          cmdptr = &DCP->CH2CMDPTR;
          chsema = &DCP->CH2SEMA;
          break;

        case kDCP_Channel3:
          cmdptr = &DCP->CH3CMDPTR;
          chsema = &DCP->CH3SEMA;
          break;

        default:
          break;
      }

      if (cmdptr && chsema)
      {
        /* set out packet to DCP CMDPTR */
        *cmdptr = (uint32_t)dcpPacket;

        /* set the channel semaphore */
        *chsema = 1u;
      }
      status = kStatus_Success;
    }

    else
    {
      status = kStatus_DCP_Again;
    }
    interrupts();
  }

  else
  {
    return kStatus_DCP_Again;
  }

  return status;
}

uint32_t Crypto_T4::dcp_hash_update_non_blocking(
  dcp_hash_ctx_internal_t *ctxInternal, dcp_work_packet_t *dcpPacket, const uint8_t *msg, size_t size)
{
  dcpPacket->control0 = ctxInternal->ctrl0 | (ctxInternal->handle->swapConfig & 0xFC0000u) |
                        kDCP_CONTROL0_ENABLE_HASH | kDCP_CONTROL0_DECR_SEMAPHOR;
  if (ctxInternal->algo == kDCP_Sha256)
  {
    dcpPacket->control1 = kDCP_CONTROL1_HASH_SELECT_SHA256;
  }
  else if (ctxInternal->algo == kDCP_Sha1)
  {
    dcpPacket->control1 = kDCP_CONTROL1_HASH_SELECT_SHA1;
  }
  else if (ctxInternal->algo == kDCP_Crc32)
  {
    dcpPacket->control1 = kDCP_CONTROL1_HASH_SELECT_CRC32;
  }
  else
  {
    return 1;
  }
  dcpPacket->sourceBufferAddress = (uint32_t)msg;
  dcpPacket->destinationBufferAddress = 0;
  dcpPacket->bufferSize = size;
  dcpPacket->payloadPointer = (uint32_t)ctxInternal->runningHash;

  return dcp_schedule_work( ctxInternal->handle, dcpPacket);
}

void Crypto_T4::dcp_hash_update(dcp_hash_ctx_internal_t *ctxInternal, const uint8_t *msg, size_t size)
{
  uint32_t completionStatus;
  dcp_work_packet_t dcpWork = {0};

  do
  {
    completionStatus = dcp_hash_update_non_blocking( ctxInternal, &dcpWork, msg, size);
  } while (completionStatus == kStatus_DCP_Again);

  completionStatus = DCP_WaitForChannelComplete(ctxInternal->handle);

  ctxInternal->ctrl0 = 0; /* clear kDCP_CONTROL0_HASH_INIT and kDCP_CONTROL0_HASH_TERM flags */
  return; // (completionStatus);
}

void Crypto_T4::dcp_hash_process_message_data(
  dcp_hash_ctx_internal_t *ctxInternal,
  const uint8_t *message,
  size_t messageSize)
{
  /* if there is partially filled internal buffer, fill it to full block */
  if (ctxInternal->blksz > 0)
  {
    size_t toCopy = DCP_HASH_BLOCK_SIZE - ctxInternal->blksz;
    memcpy(&ctxInternal->blk.b[ctxInternal->blksz], message, toCopy);
    message += toCopy;
    messageSize -= toCopy;

    /* process full internal block */
    dcp_hash_update(ctxInternal, &ctxInternal->blk.b[0], DCP_HASH_BLOCK_SIZE);
  }

  /* process all full blocks in message[] */
  uint32_t fullBlocksSize = ((messageSize >> 6) << 6); /* (X / 64) * 64 */
  if (fullBlocksSize > 0)
  {
    dcp_hash_update( ctxInternal, message, fullBlocksSize);
    message += fullBlocksSize;
    messageSize -= fullBlocksSize;
  }

  /* copy last incomplete message bytes into internal block */
  memcpy(&ctxInternal->blk.b[0], message, messageSize);
  ctxInternal->blksz = messageSize;

}


void Crypto_T4::DCP_HASH_Init(dcp_handle_t *handle, dcp_hash_ctx_t *ctx, dcp_hash_algo_t algo)
{
  dcp_hash_ctx_internal_t *ctxInternal;
  ctxInternal = (dcp_hash_ctx_internal_t *)ctx;
  ctxInternal->algo = algo;
  ctxInternal->blksz = 0u;
  for (uint32_t i = 0; i < sizeof(ctxInternal->blk.w) / sizeof(ctxInternal->blk.w[0]); i++)
  {
    ctxInternal->blk.w[i] = 0u;  // bug was 0
  }
  ctxInternal->state = kDCP_StateHashInit;
  ctxInternal->fullMessageSize = 0;
  ctxInternal->handle = handle;

}

void Crypto_T4::DCP_HASH_Update(dcp_hash_ctx_t *ctx, const uint8_t *input, size_t inputSize)
{
  bool isUpdateState;
  dcp_hash_ctx_internal_t *ctxInternal;
  size_t blockSize = DCP_HASH_BLOCK_SIZE;

  ctxInternal = (dcp_hash_ctx_internal_t *)ctx;
  ctxInternal->fullMessageSize += inputSize;
  /* if we are still less than DCP_HASH_BLOCK_SIZE bytes, keep only in context */
  if ((ctxInternal->blksz + inputSize) <= blockSize)
  {
    memcpy((&ctxInternal->blk.b[0]) + ctxInternal->blksz, input, inputSize);
    ctxInternal->blksz += inputSize;
    return ;
  }
  else
  {
    isUpdateState = ctxInternal->state == kDCP_StateHashUpdate;
    if (!isUpdateState)
    {
      /* start NEW hash */
      ctxInternal->ctrl0 = kDCP_CONTROL0_HASH_INIT;
      ctxInternal->state = kDCP_StateHashUpdate;
    }
    else
    {
      //      dcp_hash_restore_running_hash(ctxInternal);  // context switch
    }
  }

  /* process input data */
  dcp_hash_process_message_data(ctxInternal, input, inputSize);
  //  dcp_hash_save_running_hash(ctxInternal);  // context

}

void Crypto_T4::DCP_HASH_Finish(dcp_hash_ctx_t *ctx, uint8_t *output)
{
  size_t algOutSize = 0;
  dcp_hash_ctx_internal_t *ctxInternal;

  ctxInternal = (dcp_hash_ctx_internal_t *)ctx;

  if (ctxInternal->state == kDCP_StateHashInit)
  {
    ctxInternal->ctrl0 = kDCP_CONTROL0_HASH_INIT;//dcp_hash_engine_init( ctxInternal);

  }
  else
  {
    // dcp_hash_restore_running_hash(ctxInternal);  // context
  }

  size_t outSize = 0u;

  /* compute algorithm output length */
  switch (ctxInternal->algo)
  {
    case kDCP_Sha256:
      outSize = kDCP_OutLenSha256;
      break;
    case kDCP_Sha1:
      outSize = kDCP_OutLenSha1;
      break;
    case kDCP_Crc32:
      outSize = kDCP_OutLenCrc32;
      break;
    default:
      break;
  }
  algOutSize = outSize;

  /* flush message last incomplete block, if there is any, and add padding bits */
  //dcp_hash_finalize( ctxInternal);
  ctxInternal->ctrl0 |= kDCP_CONTROL0_HASH_TERM;
  dcp_hash_update(ctxInternal, &ctxInternal->blk.b[0], ctxInternal->blksz);

  /* Reverse and copy result to output[] */
  dcp_reverse_and_copy((uint8_t *)ctxInternal->runningHash, &output[0], algOutSize);

  memset(ctx, 0, sizeof(dcp_hash_ctx_t));

}

void Crypto_T4::dcp_init() {
  //volatile uint32_t *p;
  CCM_CCGR0 |= CCM_CCGR0_DCP(CCM_CCGR_ON);  // DCP on

  DCP->CTRL = 0xF0800000u; /* reset value */
  DCP->CTRL = 0x30800000u; /* default value */

  dcp_clear_status();
  // clear channel status
  dcp_clear_channel_status(kDCP_Channel0 | kDCP_Channel1 | kDCP_Channel2 | kDCP_Channel3);

  DCP->CTRL = 0x00C00000;  // enable caching  writes
  DCP->CHANNELCTRL = kDCP_ch0Enable;
}

void Crypto_T4::prhash(unsigned char *h, int n) {
  int i;

  for (i = 0; i < n; i++) {
    Serial.printf("%02x", h[i]);
    if (i % 4 == 3) Serial.printf(" ");
  }
  Serial.printf("\n");
}

void Crypto_T4::do_sha256() {
  dcp_handle_t m_handle;
  dcp_hash_ctx_t hashCtx;
  uint8_t msg[16 * 1024], hash[32];
  static const uint8_t message[] = "hello";  // hash 2cf24dba ...

  m_handle.channel = kDCP_Channel0;
  m_handle.keySlot = kDCP_KeySlot0;
  m_handle.swapConfig = kDCP_NoSwap;

  DCP_HASH_Init( &m_handle, &hashCtx, kDCP_Sha256);
  DCP_HASH_Update( &hashCtx, message, 5);
  DCP_HASH_Finish( &hashCtx, hash);
  prhash(hash, 32);

  uint32_t t = micros();
  DCP_HASH_Init( &m_handle, &hashCtx, kDCP_Sha256);
  DCP_HASH_Update( &hashCtx, msg, sizeof(msg));
  DCP_HASH_Finish( &hashCtx, hash);
  t = micros() - t;
  Serial.printf("SHA256 %d bytes %d us  %.3f MBs\n", sizeof(msg), t, (float)sizeof(msg) / t);
  prhash(hash, 32);
}

void Crypto_T4::do_crc32() {
  dcp_handle_t m_handle;
  dcp_hash_ctx_t hashCtx;
  uint8_t msg[16 * 1024], hash[4];
  static const uint8_t message[] = "abcdbcdecdefdefgefghfghighijhijk";
  static const unsigned char crc32[] = {0x7f, 0x04, 0x6a, 0xdd};  // crc

  m_handle.channel = kDCP_Channel0;
  m_handle.keySlot = kDCP_KeySlot0;
  m_handle.swapConfig = kDCP_NoSwap;

  DCP_HASH_Init( &m_handle, &hashCtx, kDCP_Crc32);
  DCP_HASH_Update( &hashCtx, message, sizeof(message) - 1);
  DCP_HASH_Finish( &hashCtx, hash);
  Serial.printf("memcmp %d\n", memcmp(hash, crc32, 4));
  prhash(hash, 4);

  uint32_t t = micros();
  DCP_HASH_Init( &m_handle, &hashCtx, kDCP_Crc32);
  DCP_HASH_Update( &hashCtx, msg, sizeof(msg));
  DCP_HASH_Finish( &hashCtx, hash);
  t = micros() - t;
  Serial.printf("CRC32 %d bytes %d us  %.3f MBs\n", sizeof(msg), t, (float)sizeof(msg) / t);
  prhash(hash, 4);
}

uint32_t Crypto_T4::dcp_aes_set_sram_based_key( dcp_handle_t *handle, const uint8_t *key)
{
  DCP->KEY = DCP_KEY_INDEX(handle->keySlot);
  /* move the key by 32-bit words */
  int i = 0;
  size_t keySize = 16u;
  while (keySize)
  {
    keySize -= sizeof(uint32_t);
    DCP->KEYDATA = ((uint32_t *)(uintptr_t)key)[i];
    i++;
  }
  return kStatus_Success;
}

uint32_t Crypto_T4::DCP_AES_EncryptCbcNonBlocking(
  dcp_handle_t *handle,
  dcp_work_packet_t *dcpPacket,
  const uint8_t *plaintext,
  uint8_t *ciphertext,
  size_t size,
  const uint8_t *iv)
{
  /* Size must be 16-byte multiple */
  if ((size < 16u) || (size % 16u))
  {
    return kStatus_InvalidArgument;
  }

  dcpPacket->control0 =
    0x322u | (handle->swapConfig & 0xFC0000u); /* CIPHER_INIT | CIPHER_ENCRYPT | ENABLE_CIPHER | DECR_SEMAPHORE */
  dcpPacket->control1 = 0x10u;                   /* CBC */
  dcpPacket->sourceBufferAddress = (uint32_t)plaintext;
  dcpPacket->destinationBufferAddress = (uint32_t)ciphertext;
  dcpPacket->bufferSize = (uint32_t)size;

  if (handle->keySlot == kDCP_OtpKey)
  {
    dcpPacket->payloadPointer = (uint32_t)iv;
    dcpPacket->control0 |= (1u << 10);   /* OTP_KEY */
    dcpPacket->control1 |= (0xFFu << 8); /* KEY_SELECT = OTP_KEY */
  }
  else if (handle->keySlot == kDCP_OtpUniqueKey)
  {
    dcpPacket->payloadPointer = (uint32_t)iv;
    dcpPacket->control0 |= (1u << 10);   /* OTP_KEY */
    dcpPacket->control1 |= (0xFEu << 8); /* KEY_SELECT = UNIQUE_KEY */
  }
  else if (handle->keySlot == kDCP_PayloadKey)
  {
    /* In this case payload must contain key & iv in one array. */
    /* Copy iv into handle right behind the keyWord[] so we can point payload to keyWord[]. */
    memcpy(handle->iv, iv, 16);
    dcpPacket->payloadPointer = (uint32_t)&handle->keyWord[0];
    dcpPacket->control0 |= (1u << 11); /* PAYLOAD_KEY */
  }
  else
  {
    dcpPacket->payloadPointer = (uint32_t)iv;
    dcpPacket->control1 |= ((uint32_t)handle->keySlot << 8); /* KEY_SELECT = keySlot */
  }

  return dcp_schedule_work( handle, dcpPacket);
}

uint32_t Crypto_T4::DCP_AES_DecryptCbcNonBlocking(
  dcp_handle_t *handle,
  dcp_work_packet_t *dcpPacket,
  const uint8_t *ciphertext,
  uint8_t *plaintext,
  size_t size,
  const uint8_t *iv)
{
  /* Size must be 16-byte multiple */
  if ((size < 16u) || (size % 16u))
  {
    return kStatus_InvalidArgument;
  }

  dcpPacket->control0 = 0x222u | (handle->swapConfig & 0xFC0000u); /* CIPHER_INIT | ENABLE_CIPHER | DECR_SEMAPHORE */
  dcpPacket->control1 = 0x10u;                                     /* CBC */
  dcpPacket->sourceBufferAddress = (uint32_t)ciphertext;
  dcpPacket->destinationBufferAddress = (uint32_t)plaintext;
  dcpPacket->bufferSize = (uint32_t)size;

  if (handle->keySlot == kDCP_OtpKey)
  {
    dcpPacket->payloadPointer = (uint32_t)iv;
    dcpPacket->control0 |= (1u << 10);   /* OTP_KEY */
    dcpPacket->control1 |= (0xFFu << 8); /* OTP_KEY */
  }
  else if (handle->keySlot == kDCP_OtpUniqueKey)
  {
    dcpPacket->payloadPointer = (uint32_t)iv;
    dcpPacket->control0 |= (1u << 10);   /* OTP_KEY */
    dcpPacket->control1 |= (0xFEu << 8); /* UNIQUE_KEY */
  }
  else if (handle->keySlot == kDCP_PayloadKey)
  {
    /* in this case payload must contain KEY + IV together */
    /* copy iv into handle struct so we can point payload directly to keyWord[]. */
    memcpy(handle->iv, iv, 16);
    dcpPacket->payloadPointer = (uint32_t)&handle->keyWord[0];
    dcpPacket->control0 |= (1u << 11); /* PAYLOAD_KEY */
  }
  else
  {
    dcpPacket->payloadPointer = (uint32_t)iv;
    dcpPacket->control1 |= ((uint32_t)handle->keySlot << 8); /* KEY_SELECT */
  }

  return dcp_schedule_work( handle, dcpPacket);
}

uint32_t Crypto_T4::DCP_AES_SetKey( dcp_handle_t *handle, const uint8_t *key, size_t keySize)
{
  uint32_t status = kStatus_Fail;

  if ((kDCP_OtpKey == handle->keySlot) || (kDCP_OtpUniqueKey == handle->keySlot))
  {
    /* for AES OTP and unique key, check and return read from fuses status */
    if ((DCP->STAT & DCP_STAT_OTP_KEY_READY_MASK) == DCP_STAT_OTP_KEY_READY_MASK)
    {
      status = kStatus_Success;
    }
  }
  else
  {
    /* only work with aligned key[] */
    if (0x3U & (uintptr_t)key)
    {
      return kStatus_InvalidArgument;
    }

    /* keySize must be 16. */
    if (keySize != 16U)
    {
      return kStatus_InvalidArgument;
    }

    /* move the key by 32-bit words */
    int i = 0;
    while (keySize)
    {
      keySize -= sizeof(uint32_t);
      handle->keyWord[i] = ((uint32_t *)(uintptr_t)key)[i];
      i++;
    }

    if (kDCP_PayloadKey != handle->keySlot)
    {
      /* move the key by 32-bit words to DCP SRAM-based key storage */
      status = dcp_aes_set_sram_based_key(handle, key);
    }
    else
    {
      /* for PAYLOAD_KEY, just return Ok status now */
      status = kStatus_Success;
    }
  }
  return status;
}

void Crypto_T4::DCP_AES_EncryptCbc(
  dcp_handle_t *handle,
  const uint8_t *plaintext,
  uint8_t *ciphertext,
  size_t size,
  const uint8_t iv[16])
{
  uint32_t completionStatus = kStatus_Fail;
  dcp_work_packet_t dcpWork = {0};

  do
  {
    completionStatus = DCP_AES_EncryptCbcNonBlocking(handle, &dcpWork, plaintext, ciphertext, size, iv);
  } while (completionStatus == kStatus_DCP_Again);

  if (completionStatus != kStatus_Success)
  {
    return ; //completionStatus;
  }

  DCP_WaitForChannelComplete( handle);
}

void Crypto_T4::DCP_AES_DecryptCbc(
  dcp_handle_t *handle,
  const uint8_t *ciphertext,
  uint8_t *plaintext,
  size_t size,
  const uint8_t iv[16])
{
  uint32_t completionStatus = kStatus_Fail;
  dcp_work_packet_t dcpWork = {0};

  do
  {
    completionStatus = DCP_AES_DecryptCbcNonBlocking( handle, &dcpWork, ciphertext, plaintext, size, iv);
  } while (completionStatus == kStatus_DCP_Again);

  if (completionStatus != kStatus_Success)
  {
    return; // completionStatus;
  }

  DCP_WaitForChannelComplete( handle);
}


void Crypto_T4::encryptAESMess(uint8_t keyAes128[], uint8_t ive[], uint8_t plainTextIn[], uint8_t *cipherTextOut)
{
  dcp_handle_t m_handle;
  m_handle.channel = kDCP_Channel0;
  m_handle.swapConfig = kDCP_NoSwap;
  m_handle.keySlot = kDCP_KeySlot0;    // could use OTP key
  DCP_AES_SetKey(&m_handle, keyAes128, 16);
  DCP_AES_EncryptCbc(&m_handle, plainTextIn, cipherTextOut, 16, ive);
}

void Crypto_T4::decryptAESMess(uint8_t keyAes128[], uint8_t ive[], uint8_t cipherTextIn[], uint8_t *plainTextOut)
{
  dcp_handle_t m_handle;
  m_handle.channel = kDCP_Channel0;
  m_handle.swapConfig = kDCP_NoSwap;
  m_handle.keySlot = kDCP_KeySlot0;    // could use OTP key
  DCP_AES_SetKey(&m_handle, keyAes128, 16);
  DCP_AES_DecryptCbc(&m_handle, cipherTextIn, plainTextOut, 16, ive);
}