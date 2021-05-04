/**************************************************
* CMPEN 473, Spring 2021, Penn State University
* 
* Homework 3 Program 1
* Revision V1.1
* On 2/11  /2021
* By: Stephen D'Amico
***************************************************/

/* Homework 3 Program 1
 * LED light blinking in sequence program for
 * Raspberry Pi 4 computer with 
 * RED LED on GPIO 12 w/ 330 Ohm resistor in series to 3.3V
 * GREEN LED on GPIO 12 w/ 330 Ohm resistor in series to ground
 * BLUE LED on GPIO 22 w/ 330 Ohm resistor in series to 3.3v
 * ORANGE LED on GPIO 22 w/ 330 Ohm resistor in series to ground
 *
 * 
 */
#include <pthread.h>
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
#include "enable_pwm_clock.h"

struct io_peripherals
{
  uint8_t              unused[0x200000];
  struct gpio_register gpio;
};

struct thread_parameter
{
  volatile struct gpio_register * gpio;
  int                             pin1;
  int                             pin2;
};

//keep track of when to quit
bool done;

int get_pressed_key(void)
{
  struct termios  original_attributes;
  struct termios  modified_attributes;
  int             ch;

  tcgetattr( STDIN_FILENO, &original_attributes );
  modified_attributes = original_attributes;
  modified_attributes.c_lflag &= ~(ICANON | ECHO);
  modified_attributes.c_cc[VMIN] = 1;
  modified_attributes.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &modified_attributes );

  ch = getchar();

  tcsetattr( STDIN_FILENO, TCSANOW, &original_attributes );

  return ch;
}

void *InputChar( void * arg )
{
  if (1)
    
  {

     do
    {
      switch (get_pressed_key())
      {
          case 'q':
            done = true;
            break;
        
      }
    } while (!done);

  }
  else
  {
    ; /* warning message already issued */
  }
  return 0;
}

//Function to blink lights in sequence
void *LightSequence( void * arg )
{
   volatile struct io_peripherals *io;
   io = import_registers();
   while(!done)
   {
          /* set the pin function to OUTPUT for GPIO12 - red LED light   */
          /* set the pin function to INPUT to start (turn circuit off) for GPIO22 - blue LED light */
          io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
          io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
          //red on to start when GPIO 12 is 0v
          usleep(400000);

          //Green on and off
          GPIO_SET( &(io->gpio), 12);

          usleep(400000);

          GPIO_CLR( &(io->gpio), 12);
          //set gpio12 to input to turn both lights off 
          io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
          
          //set gpio22 to output to turn on blue
          io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
          
          usleep(400000);
          //Orange on and off
          GPIO_SET( &(io->gpio), 22);

          usleep(400000);

          GPIO_CLR( &(io->gpio), 22);
          //set GPIO22 back to input to turn off both lights
          io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
          //set GPIO12 back to output
          io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
     
   }
    


  return 0;
}



int main( void )
{
  volatile struct io_peripherals *io;
  pthread_t                       thread12_handle;
  pthread_t                       thread22_handle;
  struct thread_parameter         thread12_parameter;
  struct thread_parameter         thread22_parameter;

  io = import_registers();
  if (io != NULL)
    
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned int)io );


  

    /* set initial output state - off */
    GPIO_CLR(&(io->gpio), 12);
    GPIO_CLR(&(io->gpio), 22);
    
    printf( "hit 'q' to quit\n");

#if 0
    Thread18( (void *)io );
#else
    thread12_parameter.pin1 = 12;
    thread12_parameter.pin2 = 22;
    thread12_parameter.gpio = &(io->gpio);
    thread22_parameter.pin1 = 23;
    thread22_parameter.pin1 = 24;
    thread22_parameter.gpio = &(io->gpio);
    //create new threads, 3rd parameter is starting routine, 4th parameter is passed to start routine 
    pthread_create( &thread12_handle, 0, LightSequence, (void *)&thread12_parameter );
    pthread_create( &thread22_handle, 0, InputChar, (void *)&thread22_parameter );
    //wait for threads to terminate before returning back 
    pthread_join( thread12_handle, 0 );
    pthread_join( thread22_handle, 0 );
#endif


    /* turn lights off before quitting */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;

    printf( "\n q is hit, now quiting ... \n");

  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}




