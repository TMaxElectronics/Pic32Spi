#include <stdint.h>

#include "SPI.h"
#include "UART.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portable.h"
#include "semphr.h"
#include "System.h"
#include "TTerm.h"

#define SPICON handle->CON->w
#define SPICONbits (*handle->CON)
#define SPICON2 handle->CON2->w
#define SPICON2bits (*handle->CON2)
#define SPIBRG (*handle->BRG)
#define SPISTAT *handle->STAT
#define SPIBUF *handle->BUF

const uint8_t SPI_dummyData[SPI_maxDummybufferSize] = {[0 ... (SPI_maxDummybufferSize-1)] = 0xff};
static uint32_t SPI_populateHandle(SPIHandle_t * ret, uint32_t module);

void SPI_setCustomPinConfig(SPIHandle_t * handle, uint32_t SDIEnabled, uint32_t SDOEnabled){
    SPICONbits.DISSDI = !SDIEnabled;
    SPICONbits.DISSDO = !SDOEnabled;
}

SPIHandle_t * SPI_init(uint32_t module, volatile uint32_t* SDOPin, uint8_t SDIPin, uint8_t spiMode, uint32_t clkFreq){
    SPIHandle_t * handle = pvPortMalloc(sizeof(SPIHandle_t));
    if(handle == NULL) return NULL;
    
    //try to populate the handle
    if(!SPI_populateHandle(handle, module)){
        //failed! likely due to an invalid module ID being selected. Just free the memory again and return NULL
        vPortFree(handle);
        return NULL;
    }
            
    SPICONbits.FRMEN = 0;
    SPICONbits.MSSEN = 0;
    SPICONbits.MCLKSEL = 0;
    SPICONbits.ENHBUF = 0;
    SPICONbits.SIDL = 0;
    SPICONbits.MODE16 = 0;
    SPICONbits.MODE32 = 0;
    
    handle->semaphore = xSemaphoreCreateBinary();
    handle->dmaSemaphore = xSemaphoreCreateBinary();
    
    xSemaphoreGive(handle->semaphore);
    xSemaphoreGive(handle->dmaSemaphore);
    
    SPICONbits.DISSDI = 0; //all data pins active, CS controlled seperately
    SPICONbits.DISSDO = 0;
    SPICONbits.SSEN = 0;
    
    switch(spiMode){
        case 0:
            SPICONbits.CKP = 0;    //set up for spi mode 0
            SPICONbits.CKE = 1;
            SPICONbits.SMP = 0;
            break;
            
        case 1:
            SPICONbits.CKP = 0;    //set up for spi mode 0
            SPICONbits.CKE = 0;
            SPICONbits.SMP = 0;
            break;
            
        case 2:
            SPICONbits.CKP = 1;    //set up for spi mode 0
            SPICONbits.CKE = 0;
            SPICONbits.SMP = 0;
            break;
            
        case 3:
            SPICONbits.CKP = 1;    //set up for spi mode 0
            SPICONbits.CKE = 1;
            SPICONbits.SMP = 0;
            break;
            
        default:    //mode 0 if any others are selected
            SPICONbits.CKP = 0;
            SPICONbits.CKE = 1;
            SPICONbits.SMP = 1;
            break;
    }
    
    SPICONbits.MODE32 = 0; //8 bit communication, master mode
    SPICONbits.MODE16 = 0;
    SPICONbits.MSTEN = 1;
    
    SPICONbits.STXISEL = 0;
    SPICONbits.SRXISEL = 0;
    
    SPICONbits.ON = 1;
    
    SPICON2 = 0;   //no audio stuff
    
    *SDOPin = handle->pinVal;
    (*handle->SDIR) = SDIPin;
    
    SPIBRG = (configPERIPHERAL_CLOCK_HZ/(2*clkFreq)) - 1;
    
    return handle;
}

static void SPI_DMAISR(uint32_t evt, void * data){
    SPIHandle_t * handle = (SPIHandle_t *) data;
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
        /* Unblock the task by releasing the semaphore. */
    xSemaphoreGiveFromISR(handle->dmaSemaphore, &xHigherPriorityTaskWoken);
    
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

uint32_t SPI_setDMAEnabled(SPIHandle_t * handle, uint32_t ena){
    //do we need to change anything?
    if(ena && handle->dmaEnabled) return 1;
    if(!ena && !handle->dmaEnabled) return 1;
    
    if(ena){
        //aquire two dma channels
        //do we need RX DMA?
        if(!SPICONbits.DISSDI) handle->rxDMA = DMA_allocateChannel();
        if(!SPICONbits.DISSDO) handle->txDMA = DMA_allocateChannel();

        //configure SPI module IRQs (tx when at least one item can be written, rx when at least one can be read)
        SPI_setBufferConfig(handle, 1);
        SPI_setIRQConfig(handle, (handle->txDMA != 0) ? 0b11 : 0, (handle->rxDMA != NULL) ? 0b01 : 0);

        if(handle->rxDMA != 0){
            //rx dma setup, transfer one byte every rx isr
            DMA_setChannelAttributes(handle->rxDMA, 0, 0, 0, 0, 3);
            DMA_setSrcConfig(handle->rxDMA, handle->BUF, 1);
            DMA_setTransferAttributes(handle->rxDMA, 1, handle->rxIRQ, -1);
        }

        if(handle->txDMA != 0){
            DMA_setChannelAttributes(handle->txDMA, 0, 0, 0, 0, 2);
            DMA_setDestConfig(handle->txDMA, handle->BUF, 1);
            DMA_setTransferAttributes(handle->txDMA, 1, handle->txIRQ, -1);
        }
    }else{
        DMA_freeChannel(handle->rxDMA);
        DMA_freeChannel(handle->txDMA);
        handle->rxDMA = NULL;
        handle->txDMA = NULL;
        SPI_setIRQConfig(handle, 0, 0);
        SPI_setBufferConfig(handle, 0);
    }
    
    handle->dmaEnabled = ena;
}

void SPI_setBufferConfig(SPIHandle_t * handle, uint32_t eBufferEna){
    SPICONbits.ENHBUF = eBufferEna;
}

void SPI_setIRQConfig(SPIHandle_t * handle, uint32_t txIRQMode, uint32_t rxIRQMode){
    SPICON &= ~(_SPI1CON_STXISEL_MASK | _SPI1CON_SRXISEL_MASK);
    SPICON |= txIRQMode << _SPI1CON_STXISEL_POSITION;
    SPICON |= rxIRQMode << _SPI1CON_SRXISEL_POSITION;
}

uint8_t SPI_send(SPIHandle_t * handle, uint8_t data){
    SPIBUF = data;

    while(SPISTAT & _SPI2STAT_SPIBUSY_MASK);// UART_print("return 0x%08x\r\n", SPISTAT);
    uint8_t ret = SPIBUF;
    return ret;
}

void SPI_flush(SPIHandle_t * handle){
    while(!(SPISTAT & _SPI1STAT_SPIRBE_MASK));
}

void SPI_continueDMARead(SPIHandle_t * handle, uint8_t * data, uint32_t length, unsigned WE, unsigned dummyEnable){
    
    //reset pointers and irq sources
    DMA_abortTransfer(handle->rxDMA);
    DMA_abortTransfer(handle->txDMA);
    
    DMA_setDestConfig(handle->rxDMA, data, length);
    DMA_setSrcConfig(handle->txDMA, dummyEnable ? SPI_dummyData : data, length);
    
    SPI_flush(handle);
    
    DMA_setInterruptConfig(handle->rxDMA, 0, 0, 0, 0, WE, 0, 1, 1);
    DMA_setInterruptConfig(handle->txDMA, 0, 0, 0, 0, !WE, 0, 1, 1);
    DMA_clearIF(handle->rxDMA, DMA_ALL_IF);
    DMA_clearIF(handle->txDMA, DMA_ALL_IF);
    DMA_clearGloablIF(handle->rxDMA);
    DMA_clearGloablIF(handle->txDMA);
    
    //arm rx channel if reading is desired
    DMA_setEnabled(handle->rxDMA, WE);

    //start transfer (tx isr is active as long as we can write something)
    DMA_setEnabled(handle->txDMA, 1);
}

#pragma GCC push_options
#pragma GCC optimize ("Os")
void SPI_sendBytes(SPIHandle_t * handle, uint8_t * data, uint32_t length, unsigned WE, unsigned dummyEnable, DMAIRQHandler_t customIRQHandlerFunction, void * customIRQHandlerData){
    //will we use DMA for the transfer?
    if(handle->dmaEnabled){
        //no point in running a send operation if there is no send dma TODO: evaluate this... might still be useful in some circumstance
        if(handle->txDMA == NULL) return;
        
        //reset pointers and irq sources
        if(handle->rxDMA != NULL) DMA_abortTransfer(handle->rxDMA);
        if(handle->txDMA != NULL) DMA_abortTransfer(handle->txDMA);
        
        if(handle->rxDMA != NULL) DMA_setDestConfig(handle->rxDMA, data, length);
        DMA_setSrcConfig(handle->txDMA, dummyEnable ? SPI_dummyData : data, length);
        
        SPI_flush(handle);
        
        //TODO ignore receiver overflow in tx mode
        
        if(WE){
            DMA_setIRQHandler(handle->rxDMA, (customIRQHandlerFunction == NULL) ? SPI_DMAISR : customIRQHandlerFunction, (customIRQHandlerData == NULL) ? handle : customIRQHandlerData);
            DMA_setIRQHandler(handle->txDMA, NULL, NULL);
        }else{
            DMA_setIRQHandler(handle->rxDMA, NULL, NULL);
            DMA_setIRQHandler(handle->txDMA, (customIRQHandlerFunction == NULL) ? SPI_DMAISR : customIRQHandlerFunction, (customIRQHandlerData == NULL) ? handle : customIRQHandlerData);
        }
        
        DMA_setInterruptConfig(handle->rxDMA, 0, 0, 0, 0, WE, 0, 1, 1);
        DMA_setInterruptConfig(handle->txDMA, 0, 0, 0, 0, !WE, 0, 1, 1);
        
        if(handle->rxDMA != NULL) DMA_clearIF(handle->rxDMA, DMA_ALL_IF);
        DMA_clearIF(handle->txDMA, DMA_ALL_IF);
        if(handle->rxDMA != NULL) DMA_clearGloablIF(handle->rxDMA);
        DMA_clearGloablIF(handle->txDMA);
        
        //arm rx channel if reading is desired
        if(handle->rxDMA != NULL) DMA_setEnabled(handle->rxDMA, WE);
        
        //start transfer (tx isr is active as long as we can write something)
        DMA_setEnabled(handle->txDMA, 1);
        
        if(!customIRQHandlerFunction){
            if(!xSemaphoreTake(handle->dmaSemaphore, 1000)){ 
                xSemaphoreGive(handle->dmaSemaphore);
                return;
            }
        }
        //wait for the buffer to be emptied
        while(SPISTAT & _SPI2STAT_SPIBUSY_MASK);
    }else{
        uint8_t trash = 0;
        if(dummyEnable){
            for(uint32_t i = 0;i < length; i++){
                SPIBUF = 0xff;
                while(SPISTAT & _SPI2STAT_SPIBUSY_MASK);// UART_print("return 0x%08x\r\n", SPISTAT);
                if(WE) data[i] = SPIBUF;
            }
        }else{
            for(uint32_t i = 0;i < length; i++){
                SPIBUF = data[i];
                while(SPISTAT & _SPI2STAT_SPIBUSY_MASK);// UART_print("return 0x%08x\r\n", SPISTAT);
                if(WE) data[i] = SPIBUF;
            }
        }
    }
}
#pragma GCC pop_options

void SPI_readBytes(SPIHandle_t * handle, uint8_t * data, uint16_t length){
    uint16_t i = 0;
    for(;i < length; i++) data[i] = SPI_send(handle, 0xff);
}

void SPI_setCLKFreq(SPIHandle_t * handle, uint32_t freq){
    if(freq > configPERIPHERAL_CLOCK_HZ/2) freq = configPERIPHERAL_CLOCK_HZ/2;
    SPIBRG = (configPERIPHERAL_CLOCK_HZ/(2*freq)) - 1;
}

void SPI_setCLKDiv(SPIHandle_t * handle, uint32_t div){
    SPIBRG = div;
}

uint32_t SPI_getCLKDiv(SPIHandle_t * handle){
    return SPIBRG;
}

SPIDeviceHandle_t * SPIDevice_create(SPIHandle_t * handle, volatile uint32_t * csRegister, uint32_t * csMask, uint32_t csActiveLevel){
    //allocate memory for the device struct
    SPIDeviceHandle_t * ret = pvPortMalloc(sizeof(SPIDeviceHandle_t));
    
    //did we actually get some memory?
    if(ret == NULL) return NULL;
    
    memset(ret, 0, sizeof(SPIDeviceHandle_t));
    
    //assign values
    ret->spiHandle      = handle;
    ret->csRegister     = csRegister;
    ret->csMask         = csMask;
    ret->csActiveLevel  = csActiveLevel;
    
    return ret;
}

void SPIDevice_delete(SPIDeviceHandle_t * handle){
    if(handle == NULL) return;
    
    //is the device we are removing currently selected?
    if(SPI_isDeviceSelected()){
        //yes, somebody was careless :P assert in debug mode, otherwise just deselect
        configASSERT(0);
        SPIDevice_deselect(handle);
    }
    
    //free memory
    vPortFree(handle);
}

void SPIDevice_setDMAEnabled(SPIDeviceHandle_t * handle, uint32_t ena){
    if(handle == NULL) return;
    handle->dmaEnabled = ena;
    
    //is the device currently in use? If so make sure to enable dma immediately
    if(SPI_isDeviceSelected()){
        SPI_setDMAEnabled(handle->spiHandle, ena);
    }
}

void SPIDevice_setClockspeed(SPIDeviceHandle_t * handle, uint32_t speed){
    if(handle == NULL) return;
    handle->deviceClockspeed = speed;
    
    //is the device currently in use? If so make sure to change the clockspeed immediately
    if(SPI_isDeviceSelected()){
        SPI_setCLKFreq(handle->spiHandle, speed);
    }
}

uint32_t SPIDevice_quickSelect(SPIDeviceHandle_t * handle){
    //just assert the cs pin, do nothing else. This function requires call of SPIDevice_select first
    if(handle->csActiveLevel){
        SPI_setCS(handle);
    }else{
        SPI_clearCS(handle);
    }
}

uint32_t SPIDevice_quickDeSelect(SPIDeviceHandle_t * handle){
    //just de-assert the cs pin, do nothing else. This function requires call of SPIDevice_select for proper ending of a transfer
    if(handle->csActiveLevel){        
        SPI_clearCS(handle);
    }else{
        SPI_setCS(handle);
    }
}

uint32_t SPIDevice_select(SPIDeviceHandle_t * handle, uint32_t timeout){
    if(handle == NULL) return;
    
    //is the device already selected?
    if(SPI_isDeviceSelected()){
        //yep, just return as there is nothing more to do
        return 1;
    }
    
    //no, device isn't selected right now. Try to get the semaphore, if it times out return a fail
    if(!xSemaphoreTake(handle->spiHandle->semaphore, timeout)) return 0;
    
    //we are now in total control of the spi module (evil smiley). Set it up the way the device wants it
    SPI_setDMAEnabled(handle->spiHandle, handle->dmaEnabled);
    if(handle->deviceClockspeed != 0) SPI_setCLKFreq(handle->spiHandle, handle->deviceClockspeed);
    
    //and finally assert the chipselect
    if(handle->csActiveLevel){
        SPI_setCS(handle);
    }else{
        SPI_clearCS(handle);
    }
    return 1;
}

uint32_t SPIDevice_deselect(SPIDeviceHandle_t * handle){
    //is the device even selected?
    if(!SPI_isDeviceSelected()){
        //no, just return as there is nothing more to do
        return 1;
    }
    
    //yes it is, make sure we return the semaphore
    xSemaphoreGive(handle->spiHandle->semaphore);
    
    //and de-assert the chipselect
    if(handle->csActiveLevel){        
        SPI_clearCS(handle);
    }else{
        SPI_setCS(handle);
    }
    return 1;
}

static uint32_t SPI_populateHandle(SPIHandle_t * ret, uint32_t module){
    switch(module){
#ifdef SPI1CON
        case 1:
            ret->CON = (CONBITS_t *) &SPI1CON;
            ret->CON2 = (CON2BITS_t *) &SPI1CON2;
            ret->STAT = &SPI1STAT;
            ret->BRG = &SPI1BRG;
            ret->BUF = &SPI1BUF;
            ret->pinVal = 0b0011;
            ret->SDIR = &SDI1R;
    
            ret->rxIRQ = _SPI1_RX_IRQ;
            ret->txIRQ = _SPI1_TX_IRQ;
            ret->fltIRQ = _SPI1_ERR_IRQ;
            return 1;
#endif
            
#ifdef SPI2CON
        case 2:
            ret->CON = (CONBITS_t *) &SPI2CON;
            ret->CON2 = (CON2BITS_t *) &SPI2CON2;
            ret->STAT = &SPI2STAT;
            ret->BRG = &SPI2BRG;
            ret->BUF = &SPI2BUF;
            ret->pinVal = 0b0100;
            ret->SDIR = &SDI2R;
    
            ret->rxIRQ = _SPI2_RX_IRQ;
            ret->txIRQ = _SPI2_TX_IRQ;
            ret->fltIRQ = _SPI2_ERR_IRQ;
            return 1;
#endif
            
#ifdef SPI3CON
        case 3:
            ret->CON = (CONBITS_t *) &SPI3CON;
            ret->CON2 = (CON2BITS_t *) &SPI3CON2;
            ret->STAT = &SPI3STAT;
            ret->BRG = &SPI3BRG;
            ret->BUF = &SPI3BUF;
            ret->pinVal = 0b0111;
            ret->SDIR = &SDI3R;
    
            ret->rxIRQ = _SPI3_RX_VECTOR;
            ret->txIRQ = _SPI3_TX_VECTOR;
            ret->fltIRQ = _SPI3_FAULT_VECTOR;
            return 1;
#endif
            
#ifdef SPI4CON
        case 4:
            ret->CON = (CONBITS_t *) &SPI4CON;
            ret->CON2 = (CON2BITS_t *) &SPI4CON2;
            ret->STAT = &SPI4STAT;
            ret->BRG = &SPI4BRG;
            ret->BUF = &SPI4BUF;
            ret->pinVal = 0b1000;
            ret->SDIR = &SDI4R;
    
            ret->rxIRQ = _SPI4_RX_VECTOR;
            ret->txIRQ = _SPI4_TX_VECTOR;
            ret->fltIRQ = _SPI4_FAULT_VECTOR;
            return 1;
#endif
            
#ifdef SPI5CON
        case 5:
            ret->CON = (CONBITS_t *) &SPI5CON;
            ret->CON2 = (CON2BITS_t *) &SPI5CON2;
            ret->STAT = &SPI5STAT;
            ret->BRG = &SPI5BRG;
            ret->BUF = &SPI5BUF;
            ret->pinVal = 0b1001;
            ret->SDIR = &SDI5R;
    
            ret->rxIRQ = _SPI5_RX_VECTOR;
            ret->txIRQ = _SPI5_TX_VECTOR;
            ret->fltIRQ = _SPI5_FAULT_VECTOR;
            return 1;
#endif
            
#ifdef SPI6CON
        case 6:
            ret->CON = (CONBITS_t *) &SPI6CON;
            ret->CON2 = (CON2BITS_t *) &SPI6CON2;
            ret->STAT = &SPI6STAT;
            ret->BRG = &SPI6BRG;
            ret->BUF = &SPI6BUF;
            ret->pinVal = 0b1010;
            ret->SDIR = &SDI6R;
    
            ret->rxIRQ = _SPI6_RX_VECTOR;
            ret->txIRQ = _SPI6_TX_VECTOR;
            ret->fltIRQ = _SPI6_FAULT_VECTOR;
            return 1;
#endif
        default:
            return 0;
    }
}