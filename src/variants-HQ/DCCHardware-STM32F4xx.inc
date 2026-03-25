//************************************************************************************************
//
// File:            DCCHardware-stm32f4xx.inc
// Purpose:         DCC Waveform Generator for STM32F4 processors
// Copyright 2026:  R.A. van Hoeijen
// History:         2026-003-04/ R.A> van Hoeijen - First version
//
//
//  STM32F4 series processor DMA/Timer based DCC generator ( Overview)
//
// This module defines a DCC Hardware Layer based upon DMA/Timer handling
// for STM32F4 based processors. This result in accurate usec timings for
// DCC streams and minimal processor loads.
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
// the DCC stream pin is fixed to GPIO pin PA6. For the dccRailAuxPin these can be set to a choosen value in 
// the setup sequence

//************************************************************************************************
#include "stm32f4xx.h"

//******************************************************************************
// 4. Setup the Interrupt timer and the railcom timer
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
    RailComCutoutTimer->setOverflow(RAILCOMPREDELAY, TICK_FORMAT);

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

   // enable timer 3 clock  no timer without it
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

   
    // determine used clock so we can configure
    // the timer to run on 1 usec pulses
     uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7;
     uint32_t apb1_div;
    switch (ppre1)
    {
        case 0: apb1_div = 1; break;
        case 4: apb1_div = 2; break;
        case 5: apb1_div = 4; break;
        case 6: apb1_div = 8; break;
        case 7: apb1_div = 16; break;
        default: apb1_div = 1; break;
    }

    uint32_t apb1 = SystemCoreClock / apb1_div;
    uint32_t timclk = (apb1_div == 1) ? apb1 : apb1 * 2;
    TIM3->PSC = (timclk / 1000000) - 1;   // exact 1 MHz
    // set the overflow count to highest possible value
    // will not be the main trigger for the timer
    TIM3->ARR = 0xFFFF;
    // set first compare point on 100 usec.
    TIM3->CCR1 = 100;
    // initilaize timer count start value to zero
    TIM3->CNT  = 0;
    // Timer 3 will be in compare toggle mode 
    TIM3->CCMR1 &= ~(7 << TIM_CCMR1_OC1M_Pos);
    TIM3->CCMR1 |=  (3 << TIM_CCMR1_OC1M_Pos);  // Toggle mode
    // confirm new value to timer registers
    TIM3->EGR = TIM_EGR_UG;

    TIM3->CCER |= TIM_CCER_CC1E;
    TIM3->DIER |= TIM_DIER_CC1DE;
    // start timer
    TIM3->CR1 |= TIM_CR1_CEN;
}



/* =========================================================
 * DMA1 Stream4, TIM3_CH1
 * ========================================================= */

static void DMA_Init(void)
{
	/* Clock */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	/* Disable  DMA Stream an wait for confirmation */
	DMA1_Stream4->CR &= ~DMA_SxCR_EN;
	while (DMA1_Stream4->CR & DMA_SxCR_EN);

	/* Clear ALL flags BEFORE config */
	DMA1->HIFCR =
	    DMA_HIFCR_CTCIF4 |
	    DMA_HIFCR_CHTIF4 |
	    DMA_HIFCR_CTEIF4 |
	    DMA_HIFCR_CDMEIF4 |
	    DMA_HIFCR_CFEIF4;

	/* Configure */
  // DMA Channel destination will be CCR1 register from timer 3 channel-1
	DMA1_Stream4->PAR  = (uint32_t)&TIM3->CCR1;
  // source for the dma channel data to be transfered
	DMA1_Stream4->M0AR = (uint32_t)dccISR.DMAMesg_Buf1.data;
  // number of transfers for the dma cahnnel
	DMA1_Stream4->NDTR = dccISR.nr_of_bytes;
	DMA1_Stream4->FCR &= ~DMA_SxFCR_DMDIS;   // Direct mode enable (FIFO uit)

	DMA1_Stream4->CR =
	      (5 << DMA_SxCR_CHSEL_Pos) | // TIM3_CH1
	      DMA_SxCR_DIR_0 |	// MEM-> Periph
	      DMA_SxCR_MINC |
	      DMA_SxCR_PSIZE_0 | // 16 bit transfers
	      DMA_SxCR_MSIZE_0 | // 16 Bits transfer
	      DMA_SxCR_TCIE; // full interrupts

	/* Enable NVIC BEFORE enabling stream */
	NVIC_SetPriority(DMA1_Stream4_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Stream4_IRQn);

	/* Enable stream LAST */
	DMA1_Stream4->CR |= DMA_SxCR_EN;

}




/* =========================================================
 * INIT API
 * ========================================================= */


void DccPacketEngine::setupWaveformGenerator(void)
{
  
  if (dccISR.is_initialized) return;
   initIsr();
   initDccPin(dccAuxPin, dccRailAuxPin);
   initDccPinAF(dccrailPin,PA6, 2);
   //initDccPin(DebugPin_1, PE9);
   // set the first trigger point of the timer
	dccISR.PreviousTime = 100;
  // start to fill the dma buffers for continously 
  // oiperation
	dccISR.Buf_in_Use=2; // start to fill buffer 1
	fill_Buffer(idle_pkt, 3);
	dccISR.Buf_in_Use = 1; // start to fill mess_buf 2
	fill_Buffer(idle_pkt, 3);

  // initialize the GPIO pint used for the method
  // thes e gpio pins are fixed in order to connect them to
  // the timer-channel1
  //DCC_GPIO_Init();
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

  
