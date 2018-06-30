
#define WA_P           gpio_set(GPIOA,GPIO0); gpio_clear(GPIOA,GPIO1);//set  V+ for A winding
#define WA_N           gpio_clear(GPIOA,GPIO0); gpio_set(GPIOA,GPIO1); //set  V- for A winding
#define WB_P           gpio_set(GPIOA,GPIO3); gpio_clear(GPIOA,GPIO4) //set  V+ for B winding
#define WB_N           gpio_clear(GPIOA,GPIO3); gpio_set(GPIOA,GPIO4); //set  V- for B winding
#include "wave.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <string.h>
#include <stdlib.h>
#include "usart.h"
long int max_steps;
long int backslash;
double step_size;
unsigned int speed;
long int position;
long int delayt=5000;
long int target;
long int backcounter;
//enum motor_state state;
int pcounter;

short resolution;


static void usart_setup(void)
{
    /* Setup USART2 parameters. */
    nvic_enable_irq(NVIC_USART1_IRQ);
    usart_set_baudrate(USART1, 38400);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    USART_CR1(USART1) |= USART_CR1_RXNEIE;

    /* Finally enable the USART. */
    usart_enable(USART1);
}

static void clock_setup(void)
{
    rcc_clock_setup_in_hsi_out_48mhz();
    /* Enable GPIOC clock for LED & USARTs. */
    rcc_periph_clock_enable(RCC_TIM3);

    /* Enable GPIOC, Alternate Function clocks. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    //rcc_periph_clock_enable(RCC_GPIOC);

    /* Enable clocks for USART2. */
    rcc_periph_clock_enable(RCC_USART1);

}

static void tim3_setup(void)
{
    /* Clock division and mode */
    TIM3_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE;
    /* Period */
    //TIM3_ARR = 65535;
    TIM3_ARR = 512;
    /* Prescaler */
    TIM3_PSC = 0;
    TIM3_EGR = TIM_EGR_UG;

    /* ---- */
    /* Output compare 1 mode and preload */
    TIM3_CCMR1 |= TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;

    /* Polarity and state */
    //TIM3_CCER |= TIM_CCER_CC1P | TIM_CCER_CC1E;
    TIM3_CCER |= TIM_CCER_CC1E;

    /* Capture compare value */
    TIM3_CCR1 = 0;

    /* ---- */
    /* Output compare 2 mode and preload */
    TIM3_CCMR1 |= TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;

    /* Polarity and state */
    // TIM3_CCER |= TIM_CCER_CC2P | TIM_CCER_CC2E;
    TIM3_CCER |= TIM_CCER_CC2E;

    /* Capture compare value */
    TIM3_CCR2 = 0;

    /* ---- */
    /* Output compare 3 mode and preload */
    TIM3_CCMR2 |= TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE;

    /* Polarity and state */
//   TIM3_CCER |= TIM_CCER_CC3P | TIM_CCER_CC3E;
    //TIM3_CCER |= TIM_CCER_CC3E;
//timer_set_oc_polarity_high(TIM3, 1);
    /* Capture compare value */
    //  TIM3_CCR3 = 0;

    /* ---- */
    /* Output compare 4 mode and preload */
    //  TIM3_CCMR2 |= TIM_CCMR2_OC4M_PWM1 | TIM_CCMR2_OC4PE;

    /* Polarity and state */
    //TIM3_CCER |= TIM_CCER_CC4P | TIM_CCER_CC4E;
    TIM3_CCER |= TIM_CCER_CC4E;

    /* Capture compare value */
    TIM3_CCR4 = 0;

    /* ---- */
    /* ARR reload enable */
    TIM3_CR1 |= TIM_CR1_ARPE;
    //   timer_set_oc_polarity_high(TIM3, 2);
    //    timer_set_oc_polarity_high(TIM3, 1);

    /* Counter enable */
    TIM3_CR1 |= TIM_CR1_CEN;
}



static void gpio_setup(void)
{
    /* Setup GPIO pin GPIO8/9 on GPIO port C for LEDs. */
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 |GPIO1|GPIO2 | GPIO3 | GPIO4);

    /* Setup GPIO pins for USART2 transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO7| GPIO6|GPIO10);

    /* Setup USART1 TX pin as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF1, GPIO9|GPIO10|GPIO6|GPIO7);


}

void move_mstep(void)
{
    uint8_t p, s, j;
    uint8_t  pwma, pwmb;
    /*  if ((focus_aux.state == slew) && (focus_aux.position == focus_aux.target))
      {
          focus_aux.state = sync;
          focus_aux.resolution = 0;
          //  get_counter(&focus_aux);
          trigg=&focus_aux;
      }


    #ifdef BACKSLASH_COMP
      if (resolution<0)
      {
          if (backcounter==0) position += resolution;
          else backcounter+= resolution ;
      }
      else
      {
          if (backcounter==backslash)position += resolution;
          else backcounter+= resolution ;
      }


    #else

      focus_aux.position += focus_aux.resolution;
    #endif // BACKSLASH_COMP
    */  position += resolution;
    pcounter += resolution;

    /*   if (bit_is_clear(PINF, PF0) &&  (focus_aux.resolution<0))
       {

           focus_aux.backcounter;
           focus_aux.position=focus_aux.resolution = 0;
           focus_aux.state = sync;
       }
       */
    if (pcounter < 0)   pcounter += MSTEPS4;
    else if (pcounter >= MSTEPS4) pcounter -= MSTEPS4;

    s = pcounter / MSTEPS;

    j = MSTEPS - (p = pcounter % (MSTEPS));

    if (s & 1) //check odd even step
    {
        pwma = wave[p];
        pwmb = wave[j];
        if (s == 1)
        {
            WA_P;
            WB_N ;
        }
        else
        {
            WA_N;
            WB_P;
        }
    }
    else
    {
        pwma = wave[j];
        pwmb = wave[p];

        if (s == 0)
        {
            WA_P;
            WB_P;
        }
        else
        {
            WA_N;
            WB_N;
        }
    }
    TIM3_CCR1=(pwmb);
    TIM3_CCR2=(pwma);



}


void serialParse(uint8_t* cad)
{
    uint8_t len,cmd;
    len=strlen((char*) cad);
    cmd =cad[len-2];
    cad[len-2]=0;


    switch(cmd)
    {
    case 'i':
        resolution=atol((char*)cad);

        break;
    case 'd':
        delayt=atol((char*)cad);

        break;


    case 'p':

        USART_Put_Num(position);
        uartwrite("\r\n");
        break;
    default:
        break;


    }


}




int main(void)
{
    int i;

    clock_setup();
    gpio_setup();
    usart_setup();
    parser=&serialParse;
    tim3_setup();
    resolution=1;
    gpio_set(GPIOA,GPIO2);
    gpio_set(GPIOA,GPIO0);
    gpio_set(GPIOA,GPIO3);
    gpio_clear(GPIOA,GPIO1);
    gpio_clear(GPIOA,GPIO4);

    /* Blink the LED (PD12) on the board with every transmitted byte. */
    while (1)
    {


        move_mstep();
        for (i = 0; i < delayt; i++)  	/* Wait a bit. */
        {
            __asm__("NOP");
        }
    }

    return 0;
}
