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

SPI_HANDLE * SPI_createHandle(uint8_t module){
    SPI_HANDLE * ret = 0;
    switch(module){
#ifdef SPI1CON
        case 1:
            ret = pvPortMalloc(sizeof(SPI_HANDLE));
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
            return ret;
#endif
            
#ifdef SPI2CON
        case 2:
            ret = pvPortMalloc(sizeof(SPI_HANDLE));
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
            return ret;
#endif
            
#ifdef SPI3CON
        case 3:
            ret = pvPortMalloc(sizeof(SPI_HANDLE));
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
            return ret;
#endif
            
#ifdef SPI4CON
        case 4:
            ret = pvPortMalloc(sizeof(SPI_HANDLE));
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
            return ret;
#endif
            
#ifdef SPI5CON
        case 5:
            ret = pvPortMalloc(sizeof(SPI_HANDLE));
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
            return ret;
#endif
            
#ifdef SPI6CON
        case 6:
            ret = pvPortMalloc(sizeof(SPI_HANDLE));
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
            return ret;
#endif
    }
    return 0;
}

void SPI_setCustomPinConfig(SPI_HANDLE * handle, uint32_t SDIEnabled, uint32_t SDOEnabled){
    SPICONbits.DISSDI = !SDIEnabled;
    SPICONbits.DISSDO = !SDOEnabled;
}

void SPI_init(SPI_HANDLE * handle, volatile uint32_t* SDOPin, uint8_t SDIPin, uint8_t spiMode, uint32_t clkFreq){
    SPICONbits.FRMEN = 0;
    SPICONbits.MSSEN = 0;
    SPICONbits.MCLKSEL = 0;
    SPICONbits.ENHBUF = 0;
    SPICONbits.SIDL = 0;
    SPICONbits.MODE16 = 0;
    SPICONbits.MODE32 = 0;
    
    handle->semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(handle->semaphore);
    
    SPICONbits.DISSDI = 0; //all data pins active, CS controlled seperately
    SPICONbits.DISSDO = 0;
    SPICONbits.SSEN = 0;
    
    switch(spiMode){
        case 0:
            SPICONbits.CKP = 0;    //set up for spi mode 0
            SPICONbits.CKE = 1;
            SPICONbits.SMP = 0;
            
        case 1:
            SPICONbits.CKP = 0;    //set up for spi mode 0
            SPICONbits.CKE = 0;
            SPICONbits.SMP = 0;
            
        case 2:
            SPICONbits.CKP = 1;    //set up for spi mode 0
            SPICONbits.CKE = 0;
            SPICONbits.SMP = 0;
            
        case 3:
            SPICONbits.CKP = 1;    //set up for spi mode 0
            SPICONbits.CKE = 1;
            SPICONbits.SMP = 0;
            
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
}

static void SPI_DMAISR(uint32_t evt, void * data){
    SPI_HANDLE * handle = (SPI_HANDLE *) data;
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
        /* Unblock the task by releasing the semaphore. */
    xSemaphoreGiveFromISR(handle->semaphore, &xHigherPriorityTaskWoken);
}

uint32_t SPI_setDMAEnabled(SPI_HANDLE * handle, uint32_t ena){
    //do we need to change anything?
    if(ena && handle->dmaEnabled) return 1;
    if(!ena && !handle->dmaEnabled) return 1;
    
    //aquire SPI semaphore
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

void SPI_setBufferConfig(SPI_HANDLE * handle, uint32_t eBufferEna){
    SPICONbits.ENHBUF = eBufferEna;
}

void SPI_setIRQConfig(SPI_HANDLE * handle, uint32_t txIRQMode, uint32_t rxIRQMode){
    SPICON &= ~(_SPI1CON_STXISEL_MASK | _SPI1CON_SRXISEL_MASK);
    SPICON |= txIRQMode << _SPI1CON_STXISEL_POSITION;
    SPICON |= rxIRQMode << _SPI1CON_SRXISEL_POSITION;
}

uint8_t SPI_send(SPI_HANDLE * handle, uint8_t data){
    SPIBUF = data;

    while(SPISTAT & _SPI2STAT_SPIBUSY_MASK);// UART_print("return 0x%08x\r\n", SPISTAT);
    uint8_t ret = SPIBUF;
    return ret;
}

void SPI_flush(SPI_HANDLE * handle){
    while(!(SPISTAT & _SPI1STAT_SPIRBE_MASK));
}

void SPI_continueDMARead(SPI_HANDLE * handle, uint8_t * data, uint32_t length, unsigned WE, unsigned dummyEnable){
    
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

void SPI_sendBytes(SPI_HANDLE * handle, uint8_t * data, uint32_t length, unsigned WE, unsigned dummyEnable, DMAIRQHandler_t customIRQHandlerFunction, void * customIRQHandlerData){
    //if(!xSemaphoreTake(handle->semaphore, 1000)) return 0;
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
            if(!xSemaphoreTake(handle->semaphore, 1000)){ 
                xSemaphoreGive(handle->semaphore);
                return;
            }
        }
        //wait for the buffer to be emptied
        while(SPISTAT & _SPI2STAT_SPIBUSY_MASK);
    }else{
        for(uint32_t i = 0;i < length; i++){
            uint8_t trash = SPI_send(handle, dummyEnable ? 0xff : data[i]);
            if(WE) data[i] = trash;
        }
    }
    //xSemaphoreGive(handle->semaphore);
}

void SPI_readBytes(SPI_HANDLE * handle, uint8_t * data, uint16_t length){
    uint16_t i = 0;
    for(;i < length; i++) data[i] = SPI_send(handle, 0xff);
}

void SPI_setCLKFreq(SPI_HANDLE * handle, uint32_t freq){
    if(freq > configPERIPHERAL_CLOCK_HZ/2) freq = configPERIPHERAL_CLOCK_HZ/2;
    SPIBRG = (configPERIPHERAL_CLOCK_HZ/(2*freq)) - 1;
}

void SPI_setCLKDiv(SPI_HANDLE * handle, uint32_t div){
    SPIBRG = div;
}

uint32_t SPI_getCLKDiv(SPI_HANDLE * handle){
    return SPIBRG;
}