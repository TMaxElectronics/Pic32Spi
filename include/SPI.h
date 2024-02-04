#include <xc.h>
#include <stdint.h>


#ifndef SPI_INC
#define SPI_INC

#include "FreeRTOS.h"
#include "semphr.h"
#include "DMA.h"

#define SPI_maxDummybufferSize 2048

typedef struct __SPIHandle_t__ SPIHandle_t;
typedef struct __SPIDeviceHandle_t__ SPIDeviceHandle_t;

void SPI_setCustomPinConfig(SPIHandle_t * handle, uint32_t SDIEnabled, uint32_t SDOEnabled);
SPIHandle_t * SPI_init(uint32_t module, volatile uint32_t* SDOPin, uint8_t SDIPin, uint8_t spiMode, uint32_t clkFreq);
uint32_t SPI_setDMAEnabled(SPIHandle_t * handle, uint32_t ena);
void SPI_setBufferConfig(SPIHandle_t * handle, uint32_t eBufferEna);
void SPI_setIRQConfig(SPIHandle_t * handle, uint32_t txIRQMode, uint32_t rxIRQMode);
uint8_t SPI_send(SPIHandle_t * handle, uint8_t data);
void SPI_continueDMARead(SPIHandle_t * handle, uint8_t * data, uint32_t length, unsigned WE, unsigned dummyEnable);
void SPI_flush(SPIHandle_t * handle);
void SPI_sendBytes(SPIHandle_t * handle, uint8_t * data, uint32_t length, unsigned WE, unsigned dummyEnable, DMAIRQHandler_t customIRQHandlerFunction, void * customIRQHandlerData);
void SPI_readBytes(SPIHandle_t * handle, uint8_t * data, uint16_t length);
void SPI_setCLKFreq(SPIHandle_t * handle, uint32_t freq);
void SPI_setCLKDiv(SPIHandle_t * handle, uint32_t div);
uint32_t SPI_getCLKDiv(SPIHandle_t * handle);

//device related commands TODO add all hardware related options as a per-device setting
SPIDeviceHandle_t   *   SPIDevice_create(SPIHandle_t * handle, volatile uint32_t * csRegister, uint32_t * csMask, uint32_t csActiveLevel);
void                    SPIDevice_delete(SPIDeviceHandle_t * handle);

void                    SPIDevice_setDMAEnabled(SPIDeviceHandle_t * handle, uint32_t ena);
void                    SPIDevice_setClockspeed(SPIDeviceHandle_t * handle, uint32_t speed);

uint32_t                SPIDevice_select(SPIDeviceHandle_t * handle, uint32_t timeout);
uint32_t                SPIDevice_deselect(SPIDeviceHandle_t * handle);

#define SPIDevice_sendBytes(deviceHandle, data, length, WE, dummyEnable, customIRQHandlerFunction, customIRQHandlerData)    SPI_sendBytes(deviceHandle->spiHandle, data, length, WE, dummyEnable, customIRQHandlerFunction, customIRQHandlerData)
#define SPIDevice_readBytes(deviceHandle, data, length)                                                                     SPI_readBytes(deviceHandle->spiHandle, data, length)
#define SPIDevice_send(deviceHandle, data)                                                                                  SPI_send(deviceHandle->spiHandle, data)

#define SPI_isDeviceSelected() handle->spiHandle->currentDevice == handle
#define SPI_setCS(HANDLE) *(handle->csRegister) |= (handle->csMask)
#define SPI_clearCS(HANDLE) *(handle->csRegister) &= ~(handle->csMask)

                
typedef union {
  struct {
    uint32_t SRXISEL:2;
    uint32_t STXISEL:2;
    uint32_t DISSDI:1;
    uint32_t MSTEN:1;
    uint32_t CKP:1;
    uint32_t SSEN:1;
    uint32_t CKE:1;
    uint32_t SMP:1;
    uint32_t MODE16:1;
    uint32_t MODE32:1;
    uint32_t DISSDO:1;
    uint32_t SIDL:1;
    uint32_t :1;
    uint32_t ON:1;
    uint32_t ENHBUF:1;
    uint32_t SPIFE:1;
    uint32_t :5;
    uint32_t MCLKSEL:1;
    uint32_t FRMCNT:3;
    uint32_t FRMSYPW:1;
    uint32_t MSSEN:1;
    uint32_t FRMPOL:1;
    uint32_t FRMSYNC:1;
    uint32_t FRMEN:1;
  };
  struct {
    uint32_t w:32;
  };
} CONBITS_t;

typedef union {
  struct {
    uint32_t AUDMOD:2;
    uint32_t :1;
    uint32_t AUDMONO:1;
    uint32_t :3;
    uint32_t AUDEN:1;
    uint32_t IGNTUR:1;
    uint32_t IGNROV:1;
    uint32_t SPITUREN:1;
    uint32_t SPIROVEN:1;
    uint32_t FRMERREN:1;
    uint32_t :2;
    uint32_t SPISGNEXT:1;
  };
  struct {
    uint32_t AUDMOD0:1;
    uint32_t AUDMOD1:1;
  };
  struct {
    uint32_t w:32;
  };
} CON2BITS_t;

typedef union {
  struct {
    uint32_t SPIRBF:1;
    uint32_t SPITBF:1;
    uint32_t :1;
    uint32_t SPITBE:1;
    uint32_t :1;
    uint32_t SPIRBE:1;
    uint32_t SPIROV:1;
    uint32_t SRMT:1;
    uint32_t SPITUR:1;
    uint32_t :2;
    uint32_t SPIBUSY:1;
    uint32_t FRMERR:1;
    uint32_t :3;
    uint32_t TXBUFELM:5;
    uint32_t :3;
    uint32_t RXBUFELM:5;
  };
  struct {
    uint32_t w:32;
  };
} STATBITS_t;

struct __SPIHandle_t__{
    volatile CONBITS_t * CON;
    volatile CON2BITS_t * CON2;
    volatile uint32_t * BRG;
    volatile uint32_t * BUF;
    volatile uint32_t * STAT;
    volatile uint32_t * SDIR;
    volatile uint8_t    pinVal;
    
    SemaphoreHandle_t   semaphore;
    SemaphoreHandle_t   dmaSemaphore;
    
    uint32_t            rxIRQ;
    uint32_t            txIRQ;
    uint32_t            fltIRQ;
    
    DMA_HANDLE_t      * txDMA;
    DMA_HANDLE_t      * rxDMA;
    uint32_t            dmaEnabled;
    
    SPIDeviceHandle_t * currentDevice;
};

struct __SPIDeviceHandle_t__{
    SPIHandle_t         *   spiHandle;
    
    volatile uint32_t   *   csRegister;
    uint32_t                csMask;
    
    int32_t                 deviceClockspeed;
    uint32_t                dmaEnabled;
    uint32_t                csActiveLevel;
};


#endif