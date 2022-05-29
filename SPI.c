#include "SPI.h"
#include "UART.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portable.h"

#define SPICON handle->CON->w
#define SPICONbits (*handle->CON)
#define SPICON2 handle->CON2->w
#define SPICON2bits (*handle->CON2)
#define SPIBRG (*handle->BRG)
#define SPISTAT *handle->STAT
#define SPIBUF *handle->BUF

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
            ret->pinVal = 0b0101;
            ret->SDIR = &SDI1R;
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
            ret->pinVal = 0b0110;
            ret->SDIR = &SDI2R;
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
            return ret;
#endif
    }
    return 0;
}

void SPI_init(SPI_HANDLE * handle, volatile uint32_t* SDOPin, uint8_t SDIPin, uint8_t spiMode, uint32_t clkFreq){
    SPICONbits.FRMEN = 0;
    SPICONbits.MSSEN = 0;
    SPICONbits.MCLKSEL = 0;
    SPICONbits.ENHBUF = 0;
    SPICONbits.SIDL = 0;
    
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

uint8_t SPI_send(SPI_HANDLE * handle, uint8_t data){
    SPIBUF = data;

    while(SPISTAT & _SPI2STAT_SPIBUSY_MASK);// UART_print("return 0x%08x\r\n", SPISTAT);
    uint8_t ret = SPIBUF;
    return ret;
}

void SPI_sendBytes(SPI_HANDLE * handle, uint8_t * data, uint8_t length, unsigned WE){
    uint16_t i = 0;
    for(;i < length; i++){
        uint8_t trash = SPI_send(handle, data[i]);
        if(WE) data[i] = trash;
    }
}

void SPI_readBytes(SPI_HANDLE * handle, uint8_t * data, uint16_t length){
    uint16_t i = 0;
    for(;i < length; i++) data[i] = SPI_send(handle, 0xff);
}

void SPI_setCLKFreq(SPI_HANDLE * handle, uint32_t freq){
    SPIBRG = (configPERIPHERAL_CLOCK_HZ/(2*freq)) - 1;
}

void SPI_setCLKDiv(SPI_HANDLE * handle, uint32_t div){
    SPIBRG = div;
}

uint32_t SPI_getCLKDiv(SPI_HANDLE * handle){
    return SPIBRG;
}