//************************************************************************************************
//
// File:            DCCHardware-stm32h7xx.inc
// Purpose:         DCC Waveform Generator for STM32H7 processors
// Copyright 2026:  R.A. van Hoeijen
// History:         2026-003-04/ R.A. van Hoeijen - First version
//
//
//  STM32h7 series processor DMA/Timer based DCC generator ( Overview)
//
// This module defines a DCC Hardware Layer based upon DMA/Timer handling
// for STM32h7 based processors. This result in accurate usec timings for
// DCC streams and minimal processor loads. The routines in this file are hardware dependent
// and are mostly initialization routines to setup the hardware.
//
// To generate the DCC stream, two buffers are used together with A DMA
// channel to provide a timer with the next compare value of a halfbit time
// for transmission on the DCC output pin.
// 
// In the Interrupt Service Routine from the DMA channel the buffers shall be 
// switched and coupled to the dma channel . When the first buffer is handled by DMA 
// the second buffer will be filled with new values.
// A combination of DMA channel and a Timer in toggle compare mode is used  to minimize
// processor load. The DMA Channel feeds the timer compare register CCR1 , when the timer needs a new 
// compare value it signals the DMA channel which in turn transfers the next value.
// In order to realize a a coherent DCC stream the DMA buffers contain the next half bit compare value.
// this means that a new value is the current compare value added with the  half bittime. 
// the Timer is a 16 bit counter and by using the next compare value, the value can continously added because
// the overflow is taken into account.
//
// Synchronization with the application:
// -------------------------------------
// The public interface is synchronous and flag-based. The application calls
// newPacket() only when canAcceptPacket is true. Internally, user data is
// first copied into a staging buffer and later transferred to one of the two
// DMA buffers. This prevents the application from modifying memory that DMA
// is still using.
//
// GPIO pins
//--------------------------------------
// Due to the nature of gpio pin assignement within the STM32F4xx nucleo boards the Pins
// are fixed. This is a design decision  within the code.
// We have choosen to use 16 bit Timer3 and are in toggle mode using timer3-channel1. This means that
// the DCC stream pin is fixed to PA6. For the dccRailAuxPin you can  choose in setup.

//************************************************************************************************
#include "stm32h7xx.h"

//******************************************************************************
// 4. Setup the Interrupt timer and configure the output pins
//******************************************************************************


/* =========================================================
 * TIM4 @ 1 mhz overflow mode for railcom cutout timing
 * ========================================================= */
static void TIM4_Init(void)
{
   // enable Timer 4 clock
    __HAL_RCC_TIM4_CLK_ENABLE();   // <-- FIX

    // initialize new timer object
    RailComCutoutTimer = new HardwareTimer(TIM4);

    // set prescale factor to value for 1 usec ticks
    uint32_t timerClk = RailComCutoutTimer->getTimerClkFreq();
    uint32_t factor = timerClk / 1000000;
    RailComCutoutTimer->setPrescaleFactor(factor);

    // set initial value for overflow .
    RailComCutoutTimer->setOverflow(RAILCOMPREDELAY);

    RailComCutoutTimer->attachInterrupt(RailComCutoutIRQ);
    /* Enable NVIC BEFORE enabling stream */
    NVIC_SetPriority(TIM4_IRQn, 1);   // hogere prioriteit
    NVIC_EnableIRQ(TIM4_IRQn);
}

/* =========================================================
 * TIM3 @ 1 MHz, toggle on compare, DMA on CC1
 * ========================================================= */
static void TIM3_Init(void)
{
    //RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
	(void)RCC->APB1LENR;   // dummy read (aanbevolen)


     // the timer to run on 1 usec pulses
     // Get APB1 prescaler
// D2APB1 prescaler
// H7 + Arduino core
uint32_t ppre1 = (RCC->D2CFGR >> 8) & 0x7;  // D2PPRE1 bits
uint32_t apb1_div;
switch(ppre1) {
    case 0: case 1: case 2: case 3: apb1_div = 1; break;
    case 4: apb1_div = 2; break;
    case 5: apb1_div = 4; break;
    case 6: apb1_div = 8; break;
    case 7: apb1_div = 16; break;
    default: apb1_div = 1; break;
}

// Timer klok
uint32_t apb1clk = SystemCoreClock / apb1_div;
uint32_t timclk  = apb1clk;//(apb1_div == 1) ? apb1clk : apb1clk * 2;

TIM3->PSC = (timclk / 1000000) - 1;
(void)TIM3->PSC;
    TIM3->ARR = 0xffff;      // startwaarde
    TIM3->CCR1 = 100;

    TIM3->CCMR1 |= (0b011 << TIM_CCMR1_OC1M_Pos);  // Toggle

    TIM3->CCMR1 &= ~TIM_CCMR1_OC1PE;

    TIM3->CR1 &= ~TIM_CR1_ARPE;

    TIM3->EGR   |= TIM_EGR_UG;       // Force update

    TIM3->EGR |= TIM_EGR_UG;         // force update

    TIM3->CCER  |= TIM_CCER_CC1E;                  // Enable output
    TIM3->CR1   |= TIM_CR1_CEN;                    // Start timer

    TIM3->DIER |= TIM_DIER_CC1DE;
    
  
}



/* =========================================================
 * DMA1 Stream4, TIM3_CH1
 * ========================================================= */

static void DMA_Init(void)
{
    /* Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
   
    RCC->AHB4ENR |= (1U << 1);
    /* Disable stream */
    DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream4->CR & DMA_SxCR_EN);

    /* Clear flags */
    DMA1->HIFCR =
        DMA_HIFCR_CTCIF4 |
        DMA_HIFCR_CHTIF4 |
        DMA_HIFCR_CTEIF4 |
        DMA_HIFCR_CDMEIF4 |
        DMA_HIFCR_CFEIF4;

    /* Configure peripheral & memory */
    DMA1_Stream4->PAR  = (uint32_t)&TIM3->CCR1;
    DMA1_Stream4->M0AR = (uint32_t)dccISR.DMAMesg_Buf1.data;
    DMA1_Stream4->NDTR = dccISR.DMAMesg_Buf1.length;

    /* Direct mode */
    DMA1_Stream4->FCR &= ~DMA_SxFCR_DMDIS;

    /* Configure stream (NO CHANNEL SELECT HERE!) */
    DMA1_Stream4->CR =
          DMA_SxCR_DIR_0        |   // MEM -> Periph
          DMA_SxCR_MINC         |
          DMA_SxCR_PSIZE_0      |   // 16-bit
          DMA_SxCR_MSIZE_0      |   // 16-bit
          DMA_SxCR_TCIE;

    /* Configure DMAMUX */
    DMAMUX1_Channel4->CCR = 23;//DMA_REQUEST_TIM3_CH1;;   // TIM3_CH1 request ID

    /* Enable NVIC */
    NVIC_SetPriority(DMA1_Stream4_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    /* 2. Clear alle relevante flags */
       	              DMA1->HIFCR =
       	                  DMA_HIFCR_CTCIF4 |
       	                  DMA_HIFCR_CHTIF4 |
       	                  DMA_HIFCR_CTEIF4 |
       	                  DMA_HIFCR_CDMEIF4 |
       	                  DMA_HIFCR_CFEIF4;
    /* Enable stream */
     SCB_CleanDCache_by_Addr((uint32_t*)dccISR.DMAMesg_Buf1.data, 88 * 2);
    DMA1_Stream4->CR |= DMA_SxCR_EN;
}


/* =========================================================
 * INIT API
 * ========================================================= */


void DccPacketEngine::setupWaveformGenerator(void)
{
  
  if (dccISR.is_initialized) return;
   initIsr();
   // initialize the GPIO pint used for the method
  // thes e gpio pins are fixed in order to connect them to
  // the timer-channel1
   initDccPin(dccAuxPin, dccRailAuxPin);
   initDccPinAF(dccrailPin, PA6, 2);
   initDccPin(DebugPin_1, PE9);
   initDccPin(DebugPin_2, PA7);
   // set the first trigger point of the timer
	dccISR.PreviousTime = 100;
  // start to fill the dma buffers for continously 
  // oiperation
	dccISR.Buf_in_Use=2; // start to fill buffer 1
	fill_Buffer(idle_pkt, 3);
	dccISR.Buf_in_Use = 1; // start to fill mess_buf 2
	fill_Buffer(idle_pkt, 3);

  
  
  // initialize TIM 3 to handle DCC stream
  TIM3_Init();
  TIM4_Init();
  //configure DMA channel to service need of compare
  // register timer 3
  DMA_Init();

    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0); // laagste prioriteit

		


		TIM3->CR1 |= TIM_CR1_CEN;
		

    DMA1_Stream4->CR |= DMA_SxCR_EN;
    dccISR.is_initialized = true;
    // start with low level on auxpin
    DCCAUX_LOW;
   
}

  





