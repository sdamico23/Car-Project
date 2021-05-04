/**************************************************
* CMPEN 473, Spring 2018, Penn State University
* 
* Homework 1 Program 1
* Revision V1.1
* On 1/13/2020
* By: Stephen D'Amico
***************************************************/

/* Homework 1 Program 1
 * LED light blinking in sequence program for
 * Raspberry Pi 4 computer with 
 * RED LED on GPIO 12 w/ 330 Ohm resistor in series
 * GREEN LED on GPIO 13 w/ 330 Ohm resistor in series
 * BLUE LED on GPIO 22 w/ 330 Ohm resistor in series
 * ORANGE LED on GPIO 23 w/ 330 Ohm resistor in series
 *
 * 
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include "import_registers.h"
#include "gpio.h"
#include "cm.h"
#include "pwm.h"
#include "spi.h"
#include "io_peripherals.h"
#include "enable_pwm_clock.h"


int main( void )
{
  volatile struct io_peripherals *io;

  io = import_registers();
  if (io != NULL)
    
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned int)io );

    /* set the pin function to OUTPUT for GPIO12 - red LED light   */
    /* set the pin function to OUTPUT for GPIO13 - green LED light */
    /* set the pin function to OUTPUT for GPIO22 - blue LED light */
    /* set the pin function to OUTPUT for GPIO23 - orange LED light */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;

    /* set initial output state - off */
    GPIO_CLR(&(io->gpio), 12);
    GPIO_CLR(&(io->gpio), 13);
    GPIO_CLR(&(io->gpio), 22);
    GPIO_CLR(&(io->gpio), 23);
    
    printf( "hit 'ctl c' to quit\n");

    while (1)
    {
      //RED on and off
      GPIO_SET( &(io->gpio), 12);

      sleep(1);

      GPIO_CLR( &(io->gpio), 12);

      sleep(1);
      //Green on and off
      GPIO_SET( &(io->gpio), 13);

      sleep(1);

      GPIO_CLR( &(io->gpio), 13);

      sleep(1);
      //Blue on and off
      GPIO_SET( &(io->gpio), 22);

      sleep(1);

      GPIO_CLR( &(io->gpio), 22);

      sleep(1);
      //Orange on and off
      GPIO_SET( &(io->gpio), 23);

      sleep(1);

      GPIO_CLR( &(io->gpio), 23);

      sleep(1);

    }

  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}
