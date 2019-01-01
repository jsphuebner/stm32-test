/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <stdio.h>
#include "hwdefs.h"
#include "printf.h"

void test_run();

#define OCURMAX 4096
#define NUM_FLOAT_PINS (sizeof(floatingPins) / sizeof(PINDEF))
#define NUM_CONT_PINS (sizeof(continuityPins) / sizeof(CONTINUITYDEF))

typedef struct
{
   const char* name;
   uint32_t port;
   uint32_t pin;
} PINDEF;

typedef struct
{
   PINDEF in;
   PINDEF out;
   bool activeLow;
   int timeConstant;
} CONTINUITYDEF;

static const PINDEF floatingPins[] =
{
   { "PA0", GPIOA, GPIO0 },
   { "PA15", GPIOA, GPIO15 },
   { "PB2", GPIOB, GPIO2 },
   { "PB3", GPIOB, GPIO3 },
   { "PB4", GPIOB, GPIO4 },
};

static const CONTINUITYDEF continuityPins[] =
{
   #ifdef HWCONFIG_REV1
   { { "PB0", GPIOB, GPIO0 }, { "PD2", GPIOD, GPIO2 }, false, 0 },
   #endif
   { { "PC8", GPIOC, GPIO8 }, { "PC5", GPIOC, GPIO5 }, true, 600 },
   { { "PB5", GPIOB, GPIO5 }, { "PC13", GPIOC, GPIO13 }, true, 600 },
   { { "PB6", GPIOB, GPIO6 }, { "PC10", GPIOC, GPIO10 }, true, 600 },
   { { "PA2", GPIOA, GPIO2 }, { "PC11", GPIOC, GPIO11 }, true, 600 },
   { { "PA4", GPIOA, GPIO4 }, { "PB9", GPIOB, GPIO9 }, true, 600 },
   { { "PC6", GPIOC, GPIO6 }, { "PB1", GPIOB, GPIO1 }, true, 600 },
   { { "PA5", GPIOA, GPIO5 }, { "PA8", GPIOA, GPIO8 }, false, 0 },
   { { "PC3", GPIOC, GPIO3 }, { "PA8", GPIOA, GPIO8 }, false, 4000 },
   { { "PC4", GPIOC, GPIO4 }, { "PA8", GPIOA, GPIO8 }, false, 4000 },
   { { "PB0", GPIOB, GPIO0 }, { "PA9", GPIOA, GPIO9 }, false, 0 },
   { { "PA6", GPIOA, GPIO6 }, { "PB13", GPIOB, GPIO13 }, false, 0 },
   { { "PA7", GPIOA, GPIO7 }, { "PB14", GPIOB, GPIO14 }, false, 0 },
   { { "PC9", GPIOC, GPIO9 }, { "PA10", GPIOA, GPIO10 }, false, 0 },
   { { "PC0", GPIOC, GPIO0 }, { "PA10", GPIOA, GPIO10 }, false, 5000 },
   { { "PC1", GPIOC, GPIO1 }, { "PB15", GPIOB, GPIO15 }, false, 5000 },
};

static void clock_setup(void)
{
   RCC_CLOCK_SETUP();

   /* Enable all needed GPIOx clocks */
   rcc_periph_clock_enable(RCC_GPIOA);
   rcc_periph_clock_enable(RCC_GPIOB);
   rcc_periph_clock_enable(RCC_GPIOC);
   rcc_periph_clock_enable(RCC_GPIOD);
   rcc_periph_clock_enable(RCC_AFIO);
   rcc_periph_clock_enable(RCC_USART3);
   rcc_periph_clock_enable(RCC_TIM3); //delay timer
   rcc_periph_clock_enable(RCC_TIM4); //Overcurrent / AUX PWM
}

static void usart_setup(void)
{
    gpio_set_mode(TERM_USART_TXPORT, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, TERM_USART_TXPIN);

    gpio_set_mode(TERM_USART_RXPORT, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT, TERM_USART_RXPIN);

    /* Setup UART parameters. */
    usart_set_baudrate(TERM_USART, USART_BAUDRATE);
    usart_set_databits(TERM_USART, 8);
    usart_set_stopbits(TERM_USART, USART_STOPBITS_1);
    usart_set_mode(TERM_USART, USART_MODE_TX_RX);
    usart_set_parity(TERM_USART, USART_PARITY_NONE);
    usart_set_flow_control(TERM_USART, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(TERM_USART);
}

/**
* Setup timer for generating over current reference values
*/
void tim_setup()
{
   /*** Setup over/undercurrent and PWM output timer */
   timer_disable_counter(OVER_CUR_TIMER);
   //edge aligned PWM
   timer_set_alignment(OVER_CUR_TIMER, TIM_CR1_CMS_EDGE);
   timer_enable_preload(OVER_CUR_TIMER);
   /* PWM mode 1 and preload enable */
   timer_set_oc_mode(OVER_CUR_TIMER, TIM_OC2, TIM_OCM_PWM1);
   timer_set_oc_mode(OVER_CUR_TIMER, TIM_OC3, TIM_OCM_PWM1);
   timer_enable_oc_preload(OVER_CUR_TIMER, TIM_OC2);
   timer_enable_oc_preload(OVER_CUR_TIMER, TIM_OC3);

   timer_set_oc_polarity_high(OVER_CUR_TIMER, TIM_OC2);
   timer_set_oc_polarity_high(OVER_CUR_TIMER, TIM_OC3);
   timer_enable_oc_output(OVER_CUR_TIMER, TIM_OC2);
   timer_enable_oc_output(OVER_CUR_TIMER, TIM_OC3);
   timer_generate_event(OVER_CUR_TIMER, TIM_EGR_UG);
   timer_set_prescaler(OVER_CUR_TIMER, 0);
   /* PWM frequency */
   timer_set_period(OVER_CUR_TIMER, OCURMAX);
   timer_enable_counter(OVER_CUR_TIMER);

   timer_set_oc_value(OVER_CUR_TIMER, TIM_OC2, OCURMAX / 4);
   timer_set_oc_value(OVER_CUR_TIMER, TIM_OC3, (3 * OCURMAX) / 4);

   /* Delay timer */
   timer_set_prescaler(TIM3, 71); //run at 1MHz
   timer_set_period(TIM3, 65535);
   timer_one_shot_mode(TIM3);
   timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM2);
   timer_enable_preload(TIM3);
   timer_direction_up(TIM3);
   timer_generate_event(TIM3, TIM_EGR_UG);

   /** setup gpio */
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7 | GPIO8);
}

/** \brief Delays program execution using timer
 *
 * \param delay delay in µs
 *
 */
static void wait_us(uint32_t delay)
{
   int prescaler = 71;

   if (delay > 65535)
   {
      prescaler = 575; //run at 125kHz
      delay >>= 3;
   }

   timer_disable_counter(TIM3);
   timer_set_prescaler(TIM3, prescaler);
   timer_set_oc_value(TIM3, TIM_OC1, delay);
   timer_set_counter(TIM3, 0);
   timer_clear_flag(TIM3, TIM_SR_CC1IF);
   timer_generate_event(TIM3, TIM_EGR_UG);
   timer_enable_counter(TIM3);
   while (!timer_get_flag(TIM3, TIM_SR_CC1IF));
}

/** \brief Tests if a pin is floating. This test is run on
 * all unconnected pins and indicates that they are not
 * bridged to another pin, ground or Vcc.
 *
 * \param pindef pin to test
 * \return true if test was successful, false otherwise
 *
 */
static bool test_floating(PINDEF pindef)
{
   //A pin is considered floating if its state can be controlled with the internal 30k pull-up/down resistor
   bool isFloating;
   gpio_set_mode(pindef.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, pindef.pin);
   gpio_set(pindef.port, pindef.pin); //pull up with internal ~30k
   wait_us(10000);
   isFloating = gpio_get(pindef.port, pindef.pin);
   gpio_clear(pindef.port, pindef.pin);
   wait_us(10000);
   isFloating &= gpio_get(pindef.port, pindef.pin) == 0;
   return isFloating;
}

/** \brief Use an output pin to generate a stimulus for an input pin
 *
 * \param contdef CONTINUITYDEF structure specifying pins
 * \return bool true if test was successful, false otherwise
 *
 */
static bool test_continuity(CONTINUITYDEF contdef)
{
   bool isContinous;
   gpio_set_mode(contdef.out.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, contdef.out.pin);
   gpio_set_mode(contdef.in.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, contdef.in.pin);
   gpio_clear(contdef.out.port, contdef.out.pin);
   wait_us(4 * contdef.timeConstant);
   gpio_set(contdef.out.port, contdef.out.pin);
   wait_us(contdef.timeConstant);
   //After minimum delay no transition must have happened due to low pass
   isContinous = 0 == contdef.timeConstant || ((gpio_get(contdef.in.port, contdef.in.pin) != 0) == contdef.activeLow);
   wait_us(contdef.timeConstant);
   isContinous &= (gpio_get(contdef.in.port, contdef.in.pin) == 0) == contdef.activeLow;
   gpio_clear(contdef.out.port, contdef.out.pin);
   wait_us(2*contdef.timeConstant);
   isContinous &= (gpio_get(contdef.in.port, contdef.in.pin) != 0) == contdef.activeLow;
   gpio_set_mode(contdef.out.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, contdef.out.pin);
   return isContinous;
}

/** \brief Tests the window comparator
 *
 * \return bool true if test was successful, false otherwise
 *
 */
static bool test_comparator()
{
   bool comparatorOk = gpio_get(GPIOA, GPIO1) != 0;
   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);
   comparatorOk &= gpio_get(GPIOA, GPIO1) == 0;
   gpio_set(GPIOA, GPIO5);
   comparatorOk &= gpio_get(GPIOA, GPIO1) == 0;
   gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO5);
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
   comparatorOk &= gpio_get(GPIOA, GPIO1) == 0;
   gpio_set(GPIOB, GPIO0);
   comparatorOk &= gpio_get(GPIOA, GPIO1) == 0;
   gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0);
   wait_us(1000);
   comparatorOk &= gpio_get(GPIOA, GPIO1) != 0;
   return comparatorOk;
}

/** \brief Tests the inhibit logic AND tree
 *
 * \return bool true if test was successful, false otherwise
 *
 */
static bool test_inhibit()
{
   bool gateOk;
   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO1 | GPIO3);
   gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);
   gpio_set(GPIOA, GPIO1 | GPIO3);
   gpio_set(GPIOC, GPIO9);
   gateOk = gpio_get(GPIOB, GPIO12) == 0;
   gpio_clear(GPIOA, GPIO1);
   gateOk &= gpio_get(GPIOB, GPIO12) != 0;
   gpio_set(GPIOA, GPIO1);
   gpio_clear(GPIOA, GPIO3);
   wait_us(1000);
   gateOk &= gpio_get(GPIOB, GPIO12) != 0;
   gpio_set(GPIOA, GPIO3);
   gpio_clear(GPIOC, GPIO9);
   wait_us(1000);
   gateOk &= gpio_get(GPIOB, GPIO12) != 0;
   gpio_set(GPIOC, GPIO9);
   gateOk &= gpio_get(GPIOB, GPIO12) == 0;
   gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO1 | GPIO3);
   gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO9);
   return gateOk;
}

static void blink(uint32_t delay)
{
   gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);

   while(1)
   {
      gpio_toggle(GPIOC, GPIO12);
      wait_us(delay);
   }
}

static void generate_resolver_excitation()
{
   gpio_set_mode(GPIOD, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);

   for (int i = 0; i < 1000; i++)
   {
      gpio_toggle(GPIOD, GPIO2);
      wait_us(56);
   }
}

static void eval_result(bool result, bool* pass)
{
   if (result)
   {
      printf("\033[32;1;1mPASS\033[0;0;0m\r\n");
   }
   else
   {
      printf("\033[31;1;1mFAIL\033[0;0;0m\r\n");
      *pass = false;
   }
}

void test_run()
{
    bool pass = true;

    printf("Serial Number: %lu%lu%lu\r\n", DESIG_UNIQUE_ID2, DESIG_UNIQUE_ID1, DESIG_UNIQUE_ID0);

    for (uint32_t i = 0; i < NUM_FLOAT_PINS; i++)
    {
        printf("Float test pin %s ... ", floatingPins[i].name);
        eval_result(test_floating(floatingPins[i]), &pass);
    }

    for (uint32_t i = 0; i < NUM_CONT_PINS; i++)
    {
        printf("Continuity test input: %s, output: %s ... ", continuityPins[i].in.name, continuityPins[i].out.name);
        eval_result(test_continuity(continuityPins[i]), &pass);
    }

    //wait_us(100000);
    printf("Testing Window Comparator ... ");
    eval_result(test_comparator(), &pass);

    printf("Testing Inhibit Circuitry ... ");
    eval_result(test_inhibit(), &pass);

    generate_resolver_excitation();

    if (pass)
    {
        printf("\033[32;1;1mAll tests PASSED\033[0;0;0m\r\n");
        blink(100000);
    }
    else
    {
        printf("\033[31;1;1mAt least one test FAILED\033[0;0;0m\r\n");
    }
}

int main(void)
{
   clock_setup();
   usart_setup();
   tim_setup();

   #ifdef HWCONFIG_REV1
   AFIO_MAPR |= AFIO_MAPR_SPI1_REMAP;
   #endif
   #ifdef HWCONFIG_REV3
   AFIO_MAPR |= AFIO_MAPR_SPI1_REMAP | AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;
   #endif

   do {
      test_run();
   } while(usart_recv_blocking(TERM_USART));

   return 0;
}
