
#include "Usart.h"

#define __USART_USED              0x01
#define __USART_DETAILS           0x00					  //  2
#define __USART_INTERRUPTS        0x01					  //  3
#define __USART1_BAUDRATE         115200				  //  4
#define __USART1_DATABITS         0x00000000
#define __USART1_STOPBITS         0x00000000
#define __USART1_PARITY           0x00000000
#define __USART1_FLOWCTRL         0x00000000
#define __USART1_REMAP            0x00000000
#define __USART1_CR1              0x000000A0
#define __USART1_CR2              0x00000000
#define __USART1_CR3              0x00000000
#define CFGR_PRE2_MASK      			0x00003800             // Mask for APB2 prescaler
#define CFGR_PLLMULL_MASK   			0x003C0000             // Mask for PLL multiplier
#define __HSI 8000000UL
#define CFGR_SW_MASK        			0x00000003 
#define __RCC_CFGR_VAL            0x001D8402
#define __PLLMULL (((__RCC_CFGR_VAL & CFGR_PLLMULL_MASK) >> 18) + 2)
/*----------------------------------------------------------------------------
 Define SYSCLK
 *----------------------------------------------------------------------------*/
#if   ((__RCC_CFGR_VAL & CFGR_SW_MASK) == 0x00) 
  #define __SYSCLK   __HSI                        // HSI is used as system clock
#elif ((__RCC_CFGR_VAL & CFGR_SW_MASK) == 0x01)
  #define __SYSCLK   __HSE                        // HSE is used as system clock
#elif ((__RCC_CFGR_VAL & CFGR_SW_MASK) == 0x02)
  #if (__RCC_CFGR_VAL & CFGR_PLLSRC_MASK)         // HSE is PLL clock source
    #if (__RCC_CFGR_VAL & CFGR_PLLXTPRE_MASK)     // HSE/2 is used
      #define __SYSCLK  ((__HSE >> 1) * __PLLMULL)// SYSCLK = HSE/2 * pllmull
    #else                                         // HSE is used
      #define __SYSCLK  ((__HSE >> 0) * __PLLMULL)// SYSCLK = HSE   * pllmul
    #endif  
  #else                                           // HSI/2 is PLL clock source
    #define __SYSCLK  ((__HSI >> 1) * __PLLMULL)  // SYSCLK = HSI/2 * pllmul
  #endif
#else
   #error "ask for help"
#endif
/*----------------------------------------------------------------------------
 Define  HCLK
 *----------------------------------------------------------------------------*/
#define __HCLKPRESC  ((__RCC_CFGR_VAL & CFGR_HPRE_MASK) >> 4)
#if (__HCLKPRESC & 0x08)
  #define __HCLK        (__SYSCLK >> ((__HCLKPRESC & 0x07)+1))
#else
  #define __HCLK        (__SYSCLK)
#endif
/*----------------------------------------------------------------------------
 Define  PCLK2
 *----------------------------------------------------------------------------*/
#define __PCLK2PRESC  ((__RCC_CFGR_VAL & CFGR_PRE2_MASK) >> 11)
#if (__PCLK2PRESC & 0x04)
  #define __PCLK2       (__HCLK >> ((__PCLK2PRESC & 0x03)+1))
#else
  #define __PCLK2       (__HCLK)
#endif
/*----------------------------------------------------------------------------
 Define  Baudrate setting (BRR) for USART1 
 *----------------------------------------------------------------------------*/
#define __DIV(__PCLK, __BAUD)       ((__PCLK*25)/(4*__BAUD))
#define __DIVMANT(__PCLK, __BAUD)   (__DIV(__PCLK, __BAUD)/100)
#define __DIVFRAQ(__PCLK, __BAUD)   (((__DIV(__PCLK, __BAUD) - (__DIVMANT(__PCLK, __BAUD) * 100)) * 16 + 50) / 100)
#define __USART_BRR(__PCLK, __BAUD) ((__DIVMANT(__PCLK, __BAUD) << 4)|(__DIVFRAQ(__PCLK, __BAUD) & 0x0F))
#define __USART_BRR(__PCLK, __BAUD) ((__DIVMANT(__PCLK, __BAUD) << 4)|(__DIVFRAQ(__PCLK, __BAUD) & 0x0F))

void Usart_init(){
	
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                     // enable clock for Alternate Function
    AFIO->MAPR   &= ~(1 << 2);                              // clear USART1 remap
    if      ((__USART1_REMAP & 0x04) == 0x00) {             // USART1 no remap
      RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                   // enable clock for GPIOA
      GPIOA->CRH   &= ~(0xFFUL  << 4);                      // Clear PA9, PA10
      GPIOA->CRH   |=  (0x0BUL  << 4);                      // USART1 Tx (PA9)  alternate output push-pull
      GPIOA->CRH   |=  (0x04UL  << 8);                      // USART1 Rx (PA10) input floating
    }
    else {                                                  // USART1    remap
      RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                   // enable clock for Alternate Function
      AFIO->MAPR   |= __USART1_REMAP;                       // set   USART1 remap
      RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;                   // enable clock for GPIOB
      GPIOB->CRL   &= ~(0xFFUL  << 24);                     // Clear PB6, PB7
      GPIOB->CRL   |=  (0x0BUL  << 24);                     // USART1 Tx (PB6)  alternate output push-pull
      GPIOB->CRL   |=  (0x04UL  << 28);                     // USART1 Rx (PB7) input floating
    }

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;                   // enable clock for USART1
        
    USART1->BRR  = __USART_BRR(__PCLK2, __USART1_BAUDRATE); // set baudrate
    USART1->CR1  = __USART1_DATABITS;                       // set Data bits
    USART1->CR2  = __USART1_STOPBITS;                       // set Stop bits
    USART1->CR1 |= __USART1_PARITY;                         // set Parity
    USART1->CR3  = __USART1_FLOWCTRL;                       // Set Flow Control

    USART1->CR1 |= (USART_CR1_RE | USART_CR1_TE);           // RX, TX enable

    if (__USART_INTERRUPTS & 0x01) {                        // interrupts used
      USART1->CR1 |= __USART1_CR1;
      USART1->CR2 |= __USART1_CR2;
      USART1->CR3 |= __USART1_CR3;
      NVIC->ISER[1] |= (1 << (USART1_IRQn & 0x1F));   // enable interrupt
    }
}

