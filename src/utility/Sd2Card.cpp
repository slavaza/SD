/* Arduino Sd2Card Library
 * Copyright (C) 2009 by William Greiman
 * ©Copyright 2001-2017 SD Card Association
 * ©Copyright 2009-2016 Atmel Corporation.
 * ⃝Ↄopywrong 2017 Simplest System Solutions (canceled)
 * ⃝Ↄopywrong 2017-2020 Vyacheslav Azarov
 *
 * This file is part of the Arduino Sd2Card Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino Sd2Card Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
#define TRACE_ENABLE

#define USE_SPI_LIB
#include <Arduino.h>
#include "Sd2Card.h"
//------------------------------------------------------------------------------
#ifndef SOFTWARE_SPI
#ifdef USE_SPI_LIB
#include <SPI.h>
static SPISettings settings;
#endif
// functions for hardware SPI
/** Send a byte to the card */
static void spiSend(uint8_t b) {
#ifndef USE_SPI_LIB
  SPDR = b;
  while (!(SPSR & (1 << SPIF)))
    ;
#else
#ifdef ESP8266
  SPI.write(b);
#else
  SPI.transfer(b);
#endif
#endif
}
/** Receive a byte from the card */
static  uint8_t spiRec(void) {
#ifndef USE_SPI_LIB
  spiSend(0XFF);
  return SPDR;
#else
  return SPI.transfer(0xFF);
#endif
}
#else  // SOFTWARE_SPI
//------------------------------------------------------------------------------
/** nop to tune soft SPI timing */
#define nop asm volatile ("nop\n\t")
//------------------------------------------------------------------------------
/** Soft SPI receive */
uint8_t spiRec(void) {
  uint8_t data = 0;
  // no interrupts during byte receive - about 8 us
  cli();
  // output pin high - like sending 0XFF
  fastDigitalWrite(SPI_MOSI_PIN, HIGH);

  for (uint8_t i = 0; i < 8; i++) {
    fastDigitalWrite(SPI_SCK_PIN, HIGH);

    // adjust so SCK is nice
    nop;
    nop;

    data <<= 1;

    if (fastDigitalRead(SPI_MISO_PIN)) data |= 1;

    fastDigitalWrite(SPI_SCK_PIN, LOW);
  }
  // enable interrupts
  sei();
  return data;
}
//------------------------------------------------------------------------------
/** Soft SPI send */
void spiSend(uint8_t data) {
  // no interrupts during byte send - about 8 us
  cli();
  for (uint8_t i = 0; i < 8; i++) {
    fastDigitalWrite(SPI_SCK_PIN, LOW);

    fastDigitalWrite(SPI_MOSI_PIN, data & 0X80);

    data <<= 1;

    fastDigitalWrite(SPI_SCK_PIN, HIGH);
  }
  // hold SCK high for a few ns
  nop;
  nop;
  nop;
  nop;

  fastDigitalWrite(SPI_SCK_PIN, LOW);
  // enable interrupts
  sei();
}
#endif  // SOFTWARE_SPI
//------------------------------------------------------------------------------
// send command and return error code.  Return zero for OK
uint8_t Sd2Card::cardCommand(uint8_t cmd, uint32_t arg) {
  // end read if in partialBlockRead mode
  readEnd();

  // select card
  chipSelectLow();

  // wait up to 300 ms if busy
  waitNotBusy(300);
  
  // write dummy byte
  spiSend(0xFF);

  // send command
  spiSend(cmd | 0x40);

#ifdef ESP8266
  // send argument
  SPI.write32(arg, true);
#else
  // send argument
  for (int8_t s = 24; s >= 0; s -= 8) spiSend(arg >> s);
#endif


  // send CRC
  uint8_t crc = 0xFF;
  if (cmd == CMD0) crc = 0x95;  // correct crc for CMD0 with arg 0
  if (cmd == CMD8) crc = 0x87;  // correct crc for CMD8 with arg 0X1AA
  spiSend(crc);

  // wait for response
  for (uint8_t i = 0; ((status_ = spiRec()) & 0x80) && i != 0xFF; i++)
    ;
  #ifdef ESP8266
  optimistic_yield(10000);
  #endif
  return status_;
}
//------------------------------------------------------------------------------
/**
 * Determine the size of an SD flash memory card.
 *
 * \return The number of 512 byte data blocks in the card
 *         or zero if an error occurs.
 */
uint32_t Sd2Card::cardSize(void) {
  csd_t csd;
  if (!readCSD(&csd)) return 0;
  if (csd.v1.csd_ver == 0) {
    uint8_t read_bl_len = csd.v1.read_bl_len;
    uint16_t c_size = (csd.v1.c_size_high << 10)
                      | (csd.v1.c_size_mid << 2) | csd.v1.c_size_low;
    uint8_t c_size_mult = (csd.v1.c_size_mult_high << 1)
                          | csd.v1.c_size_mult_low;
    return (uint32_t)(c_size + 1) << (c_size_mult + read_bl_len - 7);
  } else if (csd.v2.csd_ver == 1) {
    uint32_t c_size = ((uint32_t)csd.v2.c_size_high << 16)
                      | (csd.v2.c_size_mid << 8) | csd.v2.c_size_low;
    return (c_size + 1) << 10;
  } else {
    error(SD_CARD_ERROR_BAD_CSD);
    return 0;
  }
}
//------------------------------------------------------------------------------
static uint8_t chip_select_asserted = 0;

void Sd2Card::chipSelectHigh(void) {
  digitalWrite(chipSelectPin_, HIGH);
#ifdef USE_SPI_LIB
  if (chip_select_asserted) {
    chip_select_asserted = 0;
    SPI.endTransaction();
  }
#endif
}
//------------------------------------------------------------------------------
void Sd2Card::chipSelectLow(void) {
#ifdef USE_SPI_LIB
  if (!chip_select_asserted) {
    chip_select_asserted = 1;
    SPI.beginTransaction(settings);
  }
#endif
  digitalWrite(chipSelectPin_, LOW);
}
//------------------------------------------------------------------------------
/** Erase a range of blocks.
 *
 * \param[in] firstBlock The address of the first block in the range.
 * \param[in] lastBlock The address of the last block in the range.
 *
 * \note This function requests the SD card to do a flash erase for a
 * range of blocks.  The data on the card after an erase operation is
 * either 0 or 1, depends on the card vendor.  The card must support
 * single block erase.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::erase(uint32_t firstBlock, uint32_t lastBlock) {
  if (!eraseSingleBlockEnable()) {
    error(SD_CARD_ERROR_ERASE_SINGLE_BLOCK);
    goto fail;
  }
  if (type_ != SD_CARD_TYPE_SDHC) {
    firstBlock <<= 9;
    lastBlock <<= 9;
  }
  if (cardCommand(CMD32, firstBlock)
    || cardCommand(CMD33, lastBlock)
    || cardCommand(CMD38, 0)) {
      error(SD_CARD_ERROR_ERASE);
      goto fail;
  }
  if (!waitNotBusy(SD_ERASE_TIMEOUT)) {
    error(SD_CARD_ERROR_ERASE_TIMEOUT);
    goto fail;
  }
  chipSelectHigh();
  return true;

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/** Determine if card supports single block erase.
 *
 * \return The value one, true, is returned if single block erase is supported.
 * The value zero, false, is returned if single block erase is not supported.
 */
uint8_t Sd2Card::eraseSingleBlockEnable(void) {
  csd_t csd;
  return readCSD(&csd) ? csd.v1.erase_blk_en : 0;
}
//------------------------------------------------------------------------------
/**
 * Initialize an SD flash memory card.
 *
 * \param[in] sckRateID SPI clock rate selector. See setSckRate().
 * \param[in] chipSelectPin SD chip select pin number.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.  The reason for failure
 * can be determined by calling errorCode() and errorData().
 */
#ifdef ESP8266
uint8_t Sd2Card::init(uint32_t sckRateID, uint8_t chipSelectPin) {
#else
uint8_t Sd2Card::init(uint8_t sckRateID, uint8_t chipSelectPin) {
#endif
  errorCode_ = inBlock_ = partialBlockRead_ = type_ = 0;
  chipSelectPin_ = chipSelectPin;
  clkRate_ = sckRateID;
  
  chip_select_asserted = 0;
  
  // 16-bit init start time allows over a minute
  uint16_t t0 = (uint16_t)millis();
  uint32_t arg;
  
  // set pin modes
  pinMode(chipSelectPin_, OUTPUT);
  digitalWrite(chipSelectPin_, HIGH);
#ifndef USE_SPI_LIB
  pinMode(SPI_MISO_PIN, INPUT);
  pinMode(SPI_MOSI_PIN, OUTPUT);
  pinMode(SPI_SCK_PIN, OUTPUT);
#endif

#ifndef SOFTWARE_SPI
#ifndef USE_SPI_LIB
  // SS must be in output mode even it is not chip select
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH); // disable any SPI device using hardware SS pin
  // Enable SPI, Master, clock rate f_osc/128
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
  // clear double speed
  SPSR &= ~(1 << SPI2X);
#else // USE_SPI_LIB
  SPI.begin();
  settings = SPISettings(250000, MSBFIRST, SPI_MODE0);
#endif // USE_SPI_LIB
#endif // SOFTWARE_SPI

  // must supply min of 74 clock cycles with CS high.
#ifdef USE_SPI_LIB
  SPI.beginTransaction(settings);
#endif
  for (uint8_t i = 0; i < 10; i++) spiSend(0XFF);
#ifdef USE_SPI_LIB
  SPI.endTransaction();
#endif

  chipSelectLow();

  // command to go idle in SPI mode
  while ((status_ = cardCommand(CMD0, 0)) != R1_IDLE_STATE) {
    if (((uint16_t)(millis() - t0)) > SD_INIT_TIMEOUT) {
      error(SD_CARD_ERROR_CMD0);
      goto fail;
    }
  }
  // check SD version
  if ((cardCommand(CMD8, 0x1AA) & R1_ILLEGAL_COMMAND)) {
    type(SD_CARD_TYPE_SD1);
  } else {
    // only need last byte of r7 response
    for (uint8_t i = 0; i < 4; i++) status_ = spiRec();
    if (status_ != 0XAA) {
      error(SD_CARD_ERROR_CMD8);
      goto fail;
    }
    type(SD_CARD_TYPE_SD2);
  }
  // initialize card and send host supports SDHC if SD2
  arg = type() == SD_CARD_TYPE_SD2 ? 0X40000000 : 0;

  while ((status_ = cardAcmd(ACMD41, arg)) != R1_READY_STATE) {
    // check for timeout
    if (((uint16_t)(millis() - t0)) > SD_INIT_TIMEOUT) {
      error(SD_CARD_ERROR_ACMD41);
      goto fail;
    }
  }
  // if SD2 read OCR register to check for SDHC card
  if (type() == SD_CARD_TYPE_SD2) {
    if (cardCommand(CMD58, 0)) {
      error(SD_CARD_ERROR_CMD58);
      goto fail;
    }
    if ((spiRec() & 0XC0) == 0XC0) type(SD_CARD_TYPE_SDHC);
    // discard rest of ocr - contains allowed voltage range
    for (uint8_t i = 0; i < 3; i++) spiRec();
  }
  chipSelectHigh();

#ifndef SOFTWARE_SPI
  return setSckRate(sckRateID);
#else  // SOFTWARE_SPI
  return true;
#endif  // SOFTWARE_SPI

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/**
 * Enable or disable partial block reads.
 *
 * Enabling partial block reads improves performance by allowing a block
 * to be read over the SPI bus as several sub-blocks.  Errors may occur
 * if the time between reads is too long since the SD card may timeout.
 * The SPI SS line will be held low until the entire block is read or
 * readEnd() is called.
 *
 * Use this for applications like the Adafruit Wave Shield.
 *
 * \param[in] value The value TRUE (non-zero) or FALSE (zero).)
 */
void Sd2Card::partialBlockRead(uint8_t value) {
  readEnd();
  partialBlockRead_ = value;
}
//------------------------------------------------------------------------------
/**
 * Read a 512 byte block from an SD card device.
 *
 * \param[in] block Logical block to be read.
 * \param[out] dst Pointer to the location that will receive the data.

 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::readBlock(uint32_t block, uint8_t* dst) {
  return readData(block, 0, 512, dst);
}
//------------------------------------------------------------------------------
/**
 * Read part of a 512 byte block from an SD card.
 *
 * \param[in] block Logical block to be read.
 * \param[in] offset Number of bytes to skip at start of block
 * \param[out] dst Pointer to the location that will receive the data.
 * \param[in] count Number of bytes to read
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::readData(uint32_t block,
        uint16_t offset, uint16_t count, uint8_t* dst) {

  if (count == 0) return true;
  if ((count + offset) > 512) {
    goto fail;
  }
  if (!inBlock_ || block != block_ || offset < offset_) {
    block_ = block;
    // use address if not SDHC card
    if (type()!= SD_CARD_TYPE_SDHC) block <<= 9;
    if (cardCommand(CMD17, block)) {
      error(SD_CARD_ERROR_CMD17);
      goto fail;
    }
    if (!waitStartBlock()) {
      goto fail;
    }
    offset_ = 0;
    inBlock_ = 1;
  }

#ifdef OPTIMIZE_HARDWARE_SPI
  // start first spi transfer
  SPDR = 0XFF;

  // skip data before offset
  for (;offset_ < offset; offset_++) {
    while (!(SPSR & (1 << SPIF)))
      ;
    SPDR = 0XFF;
  }
  // transfer data
  {   
	uint16_t n; n = count - 1;
  // read	
	for (uint16_t i = 0; i < n; i++) {
	while (!(SPSR & (1 << SPIF)))
		;
	dst[i] = SPDR;
	SPDR = 0XFF;
	}
  // wait for last byte
	while (!(SPSR & (1 << SPIF)))
		;
	dst[n] = SPDR;
  }

#else  // OPTIMIZE_HARDWARE_SPI
#ifdef ESP8266
  // skip data before offset
  SPI.transferBytes(NULL, NULL, offset_);

  // transfer data
  SPI.transferBytes(NULL, dst, count);

#else
  // skip data before offset
  for (;offset_ < offset; offset_++) {
    spiRec();
  }
  // transfer data
  for (uint16_t i = 0; i < count; i++) {
    dst[i] = spiRec();
  }
#endif
#endif  // OPTIMIZE_HARDWARE_SPI

  offset_ += count;
  if (!partialBlockRead_ || offset_ >= 512) {
    // read rest of data, checksum and set chip select high
    readEnd();
  }
  return true;

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/** Skip remaining data in a block when in partial block read mode. */
void Sd2Card::readEnd(void) {
  if (inBlock_) {
      // skip data and crc
#ifdef OPTIMIZE_HARDWARE_SPI
    // optimize skip for hardware
    SPDR = 0XFF;
    while (offset_++ < 513) {
      while (!(SPSR & (1 << SPIF)))
        ;
      SPDR = 0XFF;
    }
    // wait for last crc byte
    while (!(SPSR & (1 << SPIF)))
      ;
#else  // OPTIMIZE_HARDWARE_SPI
#ifdef ESP8266
    SPI.transferBytes(NULL, NULL, (514-offset_));
#else
    while (offset_++ < 514) spiRec();
#endif
#endif  // OPTIMIZE_HARDWARE_SPI
    chipSelectHigh();
    inBlock_ = 0;
  }
}
//------------------------------------------------------------------------------
/** read CID or CSR register */
uint8_t Sd2Card::readRegister(uint8_t cmd, void* buf) {
  uint8_t* dst = reinterpret_cast<uint8_t*>(buf);
  if (cardCommand(cmd, 0)) {
    error(SD_CARD_ERROR_READ_REG);
    goto fail;
  }
  if (!waitStartBlock()) goto fail;
  // transfer data
#ifdef ESP8266
  SPI.transferBytes(NULL, dst, 16);
#else
  for (uint16_t i = 0; i < 16; i++) dst[i] = spiRec();
#endif
  spiRec();  // get first crc byte
  spiRec();  // get second crc byte
  chipSelectHigh();
  return true;

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/**
 * Set the SPI clock rate.
 *
 * \param[in] sckRateID A value in the range [0, 6].
 *
 * The SPI clock will be set to F_CPU/pow(2, 1 + sckRateID). The maximum
 * SPI rate is F_CPU/2 for \a sckRateID = 0 and the minimum rate is F_CPU/128
 * for \a scsRateID = 6.
 *
 * \return The value one, true, is returned for success and the value zero,
 * false, is returned for an invalid value of \a sckRateID.
 */
#ifdef ESP8266
uint8_t Sd2Card::setSckRate(uint32_t sckRateID) {
#else
uint8_t Sd2Card::setSckRate(uint8_t sckRateID) {
  if (sckRateID > 6) {
    error(SD_CARD_ERROR_SCK_RATE);
    return false;
  }
#endif
#ifndef USE_SPI_LIB
  // see avr processor datasheet for SPI register bit definitions
  if ((sckRateID & 1) || sckRateID == 6) {
    SPSR &= ~(1 << SPI2X);
  } else {
    SPSR |= (1 << SPI2X);
  }
  SPCR &= ~((1 <<SPR1) | (1 << SPR0));
  SPCR |= (sckRateID & 4 ? (1 << SPR1) : 0)
    | (sckRateID & 2 ? (1 << SPR0) : 0);
#else // USE_SPI_LIB
  #ifdef ESP8266
  settings = SPISettings(sckRateID, MSBFIRST, SPI_MODE0);
  #else
  switch (sckRateID) {
    case 0:  settings = SPISettings(25000000, MSBFIRST, SPI_MODE0); break;
    case 1:  settings = SPISettings(4000000, MSBFIRST, SPI_MODE0); break;
    case 2:  settings = SPISettings(2000000, MSBFIRST, SPI_MODE0); break;
    case 3:  settings = SPISettings(1000000, MSBFIRST, SPI_MODE0); break;
    case 4:  settings = SPISettings(500000, MSBFIRST, SPI_MODE0); break;
    case 5:  settings = SPISettings(250000, MSBFIRST, SPI_MODE0); break;
    default: settings = SPISettings(125000, MSBFIRST, SPI_MODE0);
  }
  #endif
#endif // USE_SPI_LIB
  return true;
}
//------------------------------------------------------------------------------
// wait for card to go not busy
uint8_t Sd2Card::waitNotBusy(uint16_t timeoutMillis) {
  uint16_t t0 = millis();
  do {
    #ifdef ESP8266
    optimistic_yield(10000);
    #endif
    if (spiRec() == 0XFF) return true;
  }
  while (((uint16_t)millis() - t0) < timeoutMillis);
  return false;
}
//------------------------------------------------------------------------------
/** Wait for start block token */
uint8_t Sd2Card::waitStartBlock(void) {
  uint16_t t0 = millis();
  while ((status_ = spiRec()) == 0XFF) {
    #ifdef ESP8266
    optimistic_yield(10000);
    #endif
    if (((uint16_t)millis() - t0) > SD_READ_TIMEOUT) {
      error(SD_CARD_ERROR_READ_TIMEOUT);
      goto fail;
    }
  }
  if (status_ != DATA_START_BLOCK) {
    error(SD_CARD_ERROR_READ);
    goto fail;
  }
  return true;

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/**
 * Writes a 512 byte block to an SD card.
 *
 * \param[in] blockNumber Logical block to be written.
 * \param[in] src Pointer to the location of the data to be written.
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::writeBlock(uint32_t blockNumber, const uint8_t* src) {
#if SD_PROTECT_BLOCK_ZERO
  // don't allow write to first block
  if (blockNumber == 0) {
    error(SD_CARD_ERROR_WRITE_BLOCK_ZERO);
    goto fail;
  }
#endif  // SD_PROTECT_BLOCK_ZERO

  // use address if not SDHC card
  if (type() != SD_CARD_TYPE_SDHC) blockNumber <<= 9;
  if (cardCommand(CMD24, blockNumber)) {
    error(SD_CARD_ERROR_CMD24);
    goto fail;
  }
  if (!writeData(DATA_START_BLOCK, src)) goto fail;

  // wait for flash programming to complete
  if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
    error(SD_CARD_ERROR_WRITE_TIMEOUT);
    goto fail;
  }
  // response is r2 so get and check two bytes for nonzero
  if (cardCommand(CMD13, 0) || spiRec()) {
    error(SD_CARD_ERROR_WRITE_PROGRAMMING);
    goto fail;
  }
  chipSelectHigh();
  return true;

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/** Write one data block in a multiple block write sequence */
uint8_t Sd2Card::writeData(const uint8_t* src) {
  // wait for previous write to finish
  if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
    error(SD_CARD_ERROR_WRITE_MULTIPLE);
    chipSelectHigh();
    return false;
  }
  return writeData(WRITE_MULTIPLE_TOKEN, src);
}
//------------------------------------------------------------------------------
// send one block of data for write block or write multiple blocks
uint8_t Sd2Card::writeData(uint8_t token, const uint8_t* src) {
#ifdef OPTIMIZE_HARDWARE_SPI

  // send data - optimized loop
  SPDR = token;

  // send two byte per iteration
  for (uint16_t i = 0; i < 512; i += 2) {
    while (!(SPSR & (1 << SPIF)))
      ;
    SPDR = src[i];
    while (!(SPSR & (1 << SPIF)))
      ;
    SPDR = src[i+1];
  }

  // wait for last data byte
  while (!(SPSR & (1 << SPIF)))
    ;

#else  // OPTIMIZE_HARDWARE_SPI
  spiSend(token);
#ifdef ESP8266
  // send argument
  SPI.writeBytes((uint8_t *)src, 512);
#else
  for (uint16_t i = 0; i < 512; i++) {
    spiSend(src[i]);
  }
#endif
#endif  // OPTIMIZE_HARDWARE_SPI
#ifdef ESP8266
  SPI.write16(0xFFFF, true);
#else
  spiSend(0xff);  // dummy crc
  spiSend(0xff);  // dummy crc
#endif
  status_ = spiRec();
  if ((status_ & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
    error(SD_CARD_ERROR_WRITE);
    chipSelectHigh();
    return false;
  }
  return true;
}
//------------------------------------------------------------------------------
/** Start a write multiple blocks sequence.
 *
 * \param[in] blockNumber Address of first block in sequence.
 * \param[in] eraseCount The number of blocks to be pre-erased.
 *
 * \note This function is used with writeData() and writeStop()
 * for optimized multiple block writes.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::writeStart(uint32_t blockNumber, uint32_t eraseCount) {
#if SD_PROTECT_BLOCK_ZERO
  // don't allow write to first block
  if (blockNumber == 0) {
    error(SD_CARD_ERROR_WRITE_BLOCK_ZERO);
    goto fail;
  }
#endif  // SD_PROTECT_BLOCK_ZERO
  // send pre-erase count
  if (cardAcmd(ACMD23, eraseCount)) {
    error(SD_CARD_ERROR_ACMD23);
    goto fail;
  }
  // use address if not SDHC card
  if (type() != SD_CARD_TYPE_SDHC) blockNumber <<= 9;
  if (cardCommand(CMD25, blockNumber)) {
    error(SD_CARD_ERROR_CMD25);
    goto fail;
  }
  return true;

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/** End a write multiple blocks sequence.
 *
* \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::writeStop(void) {
  if (!waitNotBusy(SD_WRITE_TIMEOUT)) goto fail;
  spiSend(STOP_TRAN_TOKEN);
  if (!waitNotBusy(SD_WRITE_TIMEOUT)) goto fail;
  chipSelectHigh();
  return true;

 fail:
  error(SD_CARD_ERROR_STOP_TRAN);
  chipSelectHigh();
  return false;
}

///////////////////////////////////////////////////////////////////////////////
// Entire SD card security operations based on
// Physical Layer Simplified Specification Version 6.00
// ©Copyright 2001-2017 SD Card Association
// ©Copyright 2009-2016 Atmel Corporation
// ⃝Ↄopywrong 2017 Simplest System Solutions 

uint8_t const   CMD16 = 16;
uint8_t const   CMD42 = 42;

uint16_t Sd2Card::getStatusR2(void)
{
    uint32_t result = cardCommand(CMD13,0) | spiRec() << 8;
    
    chipSelectHigh(); 
    
    return result;
}

bool Sd2Card::isProtected()
{
    csd_t csd;
    return (Sd2Card::readCSD(&csd))? 
    (csd.v1.csd_ver == 0)? csd.v1.tmp_write_protect | csd.v1.perm_write_protect:
    (csd.v2.csd_ver == 1)? csd.v2.tmp_write_protect | csd.v2.perm_write_protect: 
    false: true;
}

bool Sd2Card::isLocked()
{
    csd_t csd;
    if (type() == 0 && readCSD(&csd) && 
       (((((uint8_t*)&csd)[0]) >> 2) & 0x0F) < 2) return false;
    
// new cards
    return (getStatusR2() & 0x0100) != 0;
}

uint16_t Sd2Card::cancelCommand(uint16_t result)
{
     chipSelectHigh(); return result;
}

uint16_t Sd2Card::lockUnlock(uint8_t action, uint8_t * passwords, uint8_t pwds_len)
{
    uint16_t result, error;
    
// constrains check
    if (((action == CHANGE_PASSWORD) && pwds_len > 32) ||
        ((action != CHANGE_PASSWORD) && pwds_len > 16) ||
        ((action == FORCED_ERASE) && (pwds_len != 0))  ||
        ((action == ENABLE_COP) && (pwds_len != 0))    ||
	(!waitNotBusy(5000)))  
				return 0xFFFF;

#ifdef TRACE_ENABLE
    Serial.print("A: "); Serial.print(action, BIN); Serial.print(" CMD16 ");
#endif

    if ((action == FORCED_ERASE) || (action == ENABLE_COP))
	error = cardCommand(CMD16, 1);
    else
	error = cardCommand(CMD16, pwds_len+2);
    
    chipSelectHigh();
    
// dummy
    spiSend(0xff); spiSend(0xff); spiSend(0xff);
    
    if (error) return error;  
    
#ifdef TRACE_ENABLE
      Serial.print("CMD42 ");
#endif
// prepare frame       
    if (result = cardCommand(CMD42,0)) return cancelCommand(result);

#ifdef TRACE_ENABLE
    Serial.print(pwds_len); Serial.print(' '); 
    Serial.println((char*)passwords);
    Serial.println("Command accepted");
#endif

// give clock again to end transaction
    spiSend(0xFF);

// send data start token
    spiSend(MMC_STARTBLOCK_WRITE);
  
// header    
    spiSend(action);
    
// no password
    if ((action != FORCED_ERASE) && (action != ENABLE_COP))
    {
	spiSend(pwds_len);
    
// send password
#ifdef ESP8266
	SPI.writeBytes(passwords, pwds_len);
#else
	for (uint8_t = 0; i < pwds_len; i++) spiSend(passwords[i])
#endif
    }
// dummy
    spiSend(0xff); spiSend(0xff);
 
// check data response token
    status_ = spiRec();
    if ((status_ & DATA_RES_MASK) != DATA_RES_ACCEPTED) 
	    return cancelCommand(0xFFFF);
// dummy
    spiSend(0xFF);
    chipSelectHigh();

// wait termination
    if (!waitNotBusy(action == FORCED_ERASE ? 5000: 100)) return 0xFFFF;

// get and check status of the operation
    result = getStatusR2();

// set original block length
    error = cardCommand(CMD16, 512);
    if (error) return cancelCommand(error);
	
#ifdef TRACE_ENABLE     
     Serial.println("Processing done");
#endif
     
    return cancelCommand(result); 
}


bool Sd2Card::lockSupported(void)
{
   csd_t csd;
   if (!readCSD(&csd)) return false;
   else if (csd.v1.csd_ver == 0) return (csd.v1.ccc_high & 0x08) != 0;
   else if (csd.v2.csd_ver == 1) return (csd.v1.ccc_high & 0x08) != 0;
   else return false;
}


//-----------------------------------------------------------------------------

static const char * csi[] =
    {"R1_IN_IDLE_STATE",    "R1_ERASE_RESET",
     "R1_ILLEGAL_COMMAND",  "R1_RCOM_CRC_ERROR",
     "R1_ERASE_SEQ_ERROR",  "R1_ADDRESS_ERROR",
     "R1_PARAMETER_ERROR",  "R1_MUST_BE_ZERO",
     "R2_CARD_IS_LOCKED",   "R2_ACCESS_CONFLICT",
     "R2_ERROR",            "R2_CC_ERROR",
     "R2_CARD_ECC_FAILED",  "R2_VP_VIOLATION",
     "R2_ERASE_PARAM",      "R2_OUT_OF_RANGE"
    };

static String _cmdline = "";
static bool   _crlf    = false;

    
void  Sd2Card::doTerminal(Stream &console)
{
    while (console.available())
    {
        char c = console.read(); console.print(c); // echo
        if (c != '\n' && c != '\f') 
        {
            _crlf = false; _cmdline += c;
        }
        else if (!_crlf)
                {
                    _crlf = true;
#ifdef TRACE_ENABLE
                    console.println("\nstart ...");
#endif
                    interpretCommand(console); 
                    _cmdline = "";
                }
    }
}

static uint8_t hex2num(char h)
{
	return (uint8_t)((h >= '0' && h <= '9') ? h - '0': 
	                 (h >= 'a') ? h - 'a' + 10: ~0);
}

uint16_t Sd2Card::hexLockUnlock(const uint8_t action, String &password)
{
	uint8_t buffer[16]; uint8_t n, size = 0;

// normalising
	password.toLowerCase();
	if (password.length() % 2) password += '0';
	size = password.length() / 2;
	
// conversion
	if (size > 16) return 0xFFFF; 
	else
		for (uint8_t i = 0; i < size; i++)
		{
// first nible
			n = hex2num(password[i*2]);
			if (n > 15) return 0xFFFF;  // error
			buffer[i]  = n << 4;
// second nible
			n = hex2num(password[i*2+1]);
			if (n > 15) return 0xFFFF;  // error
			buffer[i] |= n;
		}
	
// execution
	return lockUnlock(action, &buffer[0], size);
}

static void printStatus(Stream &io, uint16_t r)
{
    for (uint8_t i = 0; i < 16; i++)
    {
       io.print((r >> i & 1)? "[1] ":"[0] "); 
       io.println(csi[i]);
    } 
}

void Sd2Card::interpretCommand(Stream &io)
 {   
    _cmdline.trim();
    uint8_t pos = _cmdline.indexOf(' ');
    String  cmd = _cmdline.substring(0, pos);
	    cmd.toLowerCase();
    _cmdline = _cmdline.substring(pos);
    _cmdline.trim();
    _cmdline.replace(" ","");
//     
    if (cmd == "r") 
    {
       if (!init(clkRate_, chipSelectPin_)) 
       {
            io.println("initialization failed. Things to check:");
       }
       else
       {
        io.println("Wiring is correct and a card is present.");
        io.print("\nCard type: ");
        switch (type()) 
         {
            case SD_CARD_TYPE_SD1:  io.println("SD1");  break;
            case SD_CARD_TYPE_SD2:  io.println("SD2");  break;
            case SD_CARD_TYPE_SDHC: io.println("SDHC"); break;
            default: io.println("Unknown");
        }
       }
    }
    else if (cmd == "i") 
    {
       cid_t cid; 
       
       if (!readCID(&cid))
                      io.println("Can not read card");
       else 
       {
        io.println("CID content");
        io.print("Man ID: "); io.println(cid.mid); 
        io.print("OEM ID: "); io.write(cid.oid,2);
        io.print("\nName: "); io.write(cid.pnm,4); 
        io.print("\nVers: "); io.print(cid.prv_n); 
            io.print("."); io.println(cid.prv_m); 
        io.print("S/n: "); io.println(cid.psn);
        io.println("Year: "); io.print(cid.mdt_year_high << 4 | cid.mdt_year_low); 
        io.println(lockSupported()? "Lock is supported": "Lock do not supported");
      }
    }
    else if (cmd == "pc")
          printStatus(io, lockUnlock(Sd2Card::CHANGE_PASSWORD, 
                      (uint8_t*)_cmdline.c_str(), _cmdline.length()));
    else if (cmd == "psf")
          printStatus(io, lockUnlock(Sd2Card::SET_FEP_PASSWORD, 
                      (uint8_t*)_cmdline.c_str(), _cmdline.length()));
    else if (cmd == "prf")
          printStatus(io, lockUnlock(Sd2Card::RESET_FEP_PASSWORD, 
                      (uint8_t*)_cmdline.c_str(), _cmdline.length()));
    else if (cmd == "pfe")
          printStatus(io, lockUnlock(Sd2Card::FEP_FORCED_ERASE, 
                      (uint8_t*)_cmdline.c_str(), _cmdline.length()));
    else if (cmd == "ps")
          printStatus(io, lockUnlock(Sd2Card::SET_PASSWORD, 
                      (uint8_t*)_cmdline.c_str(), _cmdline.length()));
    else if (cmd == "pr")
          printStatus(io, lockUnlock(Sd2Card::RESET_PASSWORD, 
                      (uint8_t*)_cmdline.c_str(), _cmdline.length()));
    else if (cmd == "l")
          printStatus(io, lockUnlock(Sd2Card::LOCK, 
                      (uint8_t*)_cmdline.c_str(), _cmdline.length()));
    else if (cmd == "u")
         printStatus(io, lockUnlock(Sd2Card::UNLOCK, 
                      (uint8_t*)_cmdline.c_str(), _cmdline.length()));
    else if (cmd == "uc")
         printStatus(io, lockUnlock(Sd2Card::UNLOCK_COP, 
                      (uint8_t*)_cmdline.c_str(), _cmdline.length()));
    else if (cmd == "pch")
         printStatus(io, hexLockUnlock(Sd2Card::CHANGE_PASSWORD, _cmdline));
    else if (cmd == "psh")
         printStatus(io, hexLockUnlock(Sd2Card::SET_PASSWORD, _cmdline));
    else if (cmd == "prh")
         printStatus(io, hexLockUnlock(Sd2Card::RESET_PASSWORD, _cmdline));
    else if (cmd == "uh")
         printStatus(io, hexLockUnlock(Sd2Card::UNLOCK, _cmdline));
    else if (cmd == "lh")
         printStatus(io, hexLockUnlock(Sd2Card::LOCK, _cmdline));
    else if (cmd == "uch")
         printStatus(io, hexLockUnlock(Sd2Card::UNLOCK_COP, _cmdline));
    else if (cmd == "psfh")
         printStatus(io, hexLockUnlock(Sd2Card::SET_FEP_PASSWORD, _cmdline));
    else if (cmd == "prfh")
         printStatus(io, hexLockUnlock(Sd2Card::RESET_FEP_PASSWORD, _cmdline));
    else if (cmd == "pfeh")
         printStatus(io, hexLockUnlock(Sd2Card::FEP_FORCED_ERASE, _cmdline));
    else if (cmd == "ec") 
         printStatus(io, lockUnlock(Sd2Card::ENABLE_COP, NULL, 0));
    else if (cmd == "fe")
         printStatus(io, lockUnlock(Sd2Card::FORCED_ERASE, NULL, 0));
    else if (cmd == "?")
         io.println(isLocked()? "Locked": "Unlocked");
    else if (cmd == "p") 
         io.println(isProtected()? "Protecteed": "Accessible");
    else if (cmd == "s")
         printStatus(io ,getStatusR2());
    else if (cmd == "h")
         io.println(
          "CMD42 functons (Lock/Unlock and Code Ownership Protection)\n"
          "h - this help, i - info, s - status, p - is it protected?\n"
          "? - is it locked card? pc <oldp> <newp> - password change,\n"
	  "ps <pwd> - password set, pr <pwd> - password reset,\n"
	  "u <pwd> - unlock, l <pwd> - lock, uc <pwd> - unlock COP,\n"
          "psf <pwd> - FEP password set, prf <pwd> - FEP password reset,\n"
          "pfe <pwd> - FEP force erase, eс - enable COP functions,\n"
	  "pch, psh, prh, uh, lh, uch, psfh, prfh, pfeh - is identical\n"
	  "versions of the commands with hexadecimal password string, \n"
	  "to string what have odd length, will be added '0' into tail,"
	  "<pwd> = letter string without spaces, r - reset card."
         );
    else io.println("Unknown command. Type h for help.");
//        
    io.println("\nESP8266...");
 }

