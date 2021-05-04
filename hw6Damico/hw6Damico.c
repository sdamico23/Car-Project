/**************************************************
* CMPEN 473, Spring 2021, Penn State University
* 
* Homework 6 
* Revision V1.1
* On 3/172021
* By Stephen D'Amico
* 
***************************************************/
 
/* Homework 6 Car Self driving program
 * m1 to manually drive the car
 * m2 for self driving around line
 * q will quit the program
 * hit ctl c to quit
 * 
 */


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include "import_registers.h"
#include "cm.h"
#include "gpio.h"
#include "spi.h"
#include "pwm.h"
#include "io_peripherals.h"
#include "enable_pwm_clock.h"


struct pause_flag
{
  pthread_mutex_t lock;
  bool            pause;
};

struct done_flag
{
  pthread_mutex_t lock;
  bool            done;
};

struct thread_parameter
{
  volatile struct gpio_register * gpio;
  volatile struct pwm_register  * pwm;
  int                             pin1;
  int                             pin2;
  struct pause_flag *             pause;
  struct done_flag *              done;
};

struct key_thread_parameter
{
  struct done_flag *  done;
  struct pause_flag * pause1;
  struct pause_flag * pause2;
  struct pause_flag * pause3;
  struct pause_flag * pause4;
};
  

//global variables to be used
int forward1 = -1;
int backward1 = -1;
int stop1 = -1;
bool done = false;
bool m1 = false;
bool m2 = false;
#define PWM_RANGE 100
bool rightSensor = false;
bool leftSensor = false;

int rightSwitchVar = -1;
int leftSwitchVar = -1;
//keep track of duty level for each side
int dutyLevelLeft = 0;
int dutyLevelRight = 0;
int dutyLevelLeftHold = 0;
int dutyLevelRightHold = 0;

int  Tstep = 50;  /* PWM time resolution, number used for usleep(Tstep) */
int  Tlevel = 1;  /* repetition count of each light level, eg. repeat 12% light level for 5 times. */
//keep track of how many times left/right were hit
int leftCount;
int rightCount;



//left motor control function
void *leftHandle( void * arg )
{
  struct thread_parameter * parameter = (struct thread_parameter *)arg;
  int pin1 = parameter->pin1;
  int pin2 = parameter->pin2;
  volatile struct io_peripherals *io;
  int currentD;
  io = import_registers();
    //printf("got here forward");
  //enable_pwm_clock( io );
  //printf("left handle thread created");
do
  {
    //printf("left handle thread");
    switch (leftSwitchVar)
    {
      //stop 
      case 0:
    //(AI1, AI2: 10=>forward, 01=>backward, 00=>stop, 11=>short break
      if ((backward1 == 1) || (forward1 ==1)) {
        backward1 = 0;
        forward1 = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      stop1 = 1;
      //set pins to stop (00)
      GPIO_CLR(&(io->gpio), pin1);
      GPIO_CLR(&(io->gpio), pin2);
      leftSwitchVar = -1;
      break;
      //forward
      case 1: 
        //(AI1, AI2: 10=>forward, 01=>backward, 00=>stop, 11=>short break
      if ((backward1 == 1) || (stop1 ==1))  {
        backward1 = 0;
        stop1 = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      forward1 = 1;
      //set pins to forward (10)
      GPIO_SET(&(io->gpio), pin1);
      GPIO_CLR(&(io->gpio), pin2);  
      leftSwitchVar = -1;
      break;

      //backward
      case 2:
      //(AI1, AI2: 10=>forward, 01=>backward, 00=>stop, 11=>short break
      if ((forward1 == 1) || (stop1 ==1)) {
        forward1 =0;
        stop1 = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      backward1 = 1;
      //set pins to backwards (01) 
      GPIO_CLR(&(io->gpio), pin1);
      GPIO_SET(&(io->gpio), pin2);
      leftSwitchVar = -1;
      break;
      //faster
      case 3:
      //get current duty level
      currentD = dutyLevelLeft;
      //add 15 (one step)
      currentD = currentD + 15;
      if (currentD >=100){
        currentD = 100;
      }
      //set PWM to this level
      io->pwm.DAT2 = currentD;
      //save updated level to global
      dutyLevelLeft = currentD;
      leftSwitchVar = -1;
      break;
      //slower
      case 4:
      //get current duty level
      currentD = dutyLevelLeft;
      //subtract 15 (one step)
      currentD = currentD - 15;
      if (currentD <=0){
        currentD = 0;
      }
      //set PWM to this level
      io->pwm.DAT2 = currentD;
      //save updated level to global
      dutyLevelLeft = currentD;
      leftSwitchVar = -1;
      break;
      //left
      case 5:
      //decrease left motor 
       dutyLevelLeftHold = dutyLevelLeft;
      while ((leftCount > 0) && (!done)){
        //decrease to 0
        io->pwm.DAT2 = 40;
        // sleep for .5 secs
        usleep(500000);
        leftCount = leftCount-1;
      }
      //go back to previous duty level 
      io->pwm.DAT2 = dutyLevelLeftHold;
      leftSwitchVar = -1;
      break;
      //right
      case 6:
      leftSwitchVar = -1;
      break;

      case 7:
      if ((backward1 == 1) || (stop1 ==1))  {
        backward1 = 0;
        stop1 = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      forward1 = 1;
      //set pins to forward (10)
      GPIO_SET(&(io->gpio), pin1);
      GPIO_CLR(&(io->gpio), pin2);  
      //set duty level to 80
      io->pwm.DAT2= 50;
      //check input sensor for black line 
      while ((!done) && (leftSwitchVar ==7)) {
      while ((GPIO_READ(&(io->gpio), 24) == 0) && (!done) && (leftSwitchVar ==7) ){
        //check 20 times a sec
        leftSensor = false;
          //usleep(1000);
      }
      while ((GPIO_READ(&(io->gpio), 24) !=0) && (!done) && (leftSwitchVar ==7) ){
        //turn right
        //decrease to 0
        leftSensor = true;
        io->pwm.DAT2 = 0;
        io->pwm.DAT1 = 100;
        // sleep for .1 secs
        //usleep(100000);
      //go back to previous duty level 
      }
      io->pwm.DAT2= 50;
      io->pwm.DAT1 = 50;
      }

      case -1:
      break;

      default:
        break;
    }
  } while (!done);

  printf( "Left thread exiting\n" );
  return (void *)0;
}

//right motor control function
void *rightHandle( void * arg )
{
  struct thread_parameter * parameter = (struct thread_parameter *)arg;
  int pin1 = parameter->pin1;
  int pin2 = parameter->pin2;
  volatile struct io_peripherals *io;
  int currentD;
  io = import_registers();
  //enable_pwm_clock( io );

  //printf("right handle thread created");
do
  {
    //printf("%i",switchVar);
    switch (rightSwitchVar)
    {
      
      //stop 
      case 0:
        //(AI1, AI2: 10=>forward, 01=>backward, 00=>stop, 11=>short break
      if ((backward1 == 1) || (forward1 ==1)) {
        backward1 = 0;
        forward1 = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      stop1 = 1;
      //set pins to stop (00)
      GPIO_CLR(&(io->gpio), pin1);
      GPIO_CLR(&(io->gpio), pin2);
      rightSwitchVar = -1;
      break;

      //forward
      case 1: 
        //(AI1, AI2: 10=>forward, 01=>backward, 00=>stop, 11=>short break
      if ((backward1 == 1) || (stop1 ==1))  {
        backward1 = 0;
        stop1 = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      forward1 = 1;
      //set pins to forward (10)
      GPIO_SET(&(io->gpio), pin1);
      GPIO_CLR(&(io->gpio), pin2);  
      rightSwitchVar = -1;
      break;
 
      //backward
      case 2:
      //(AI1, AI2: 10=>forward, 01=>backward, 00=>stop, 11=>short break
      if ((forward1 == 1) || (stop1 ==1)) {
        forward1 =0;
        stop1 = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      backward1 = 1;
      //set pins to backwards (01) 
      GPIO_CLR(&(io->gpio), pin1);
      GPIO_SET(&(io->gpio), pin2);
      rightSwitchVar = -1;
      break;
      //faster
      case 3:
      //get current duty level
      currentD = dutyLevelRight;
      //add 15 (one step)
      currentD = currentD + 15;
      if (currentD >=100){
        currentD = 100;
      }
      //set PWM to this level
      io->pwm.DAT1 = currentD;
      //save updated level to global
      dutyLevelRight = currentD;
      rightSwitchVar = -1;
      break;
      //slower
      case 4:
      //get current duty level
      currentD = dutyLevelRight;
      //add 15 (one step)
      currentD = currentD - 15;
      if (currentD <=0){
        currentD = 0;
      }
      //set PWM to this level
      io->pwm.DAT1 = currentD;
      //save updated level to global
      dutyLevelRight = currentD;
      rightSwitchVar = -1;
      break;
      //left
      case 5:
      rightSwitchVar = -1;
      break;
      //right
      case 6:
      //decrease right motor 
      dutyLevelRightHold = dutyLevelRight;
      while ((rightCount > 0) && (!done)){
        //decrease to 40
        io->pwm.DAT1 = 40;
        // sleep for .5 secs
        usleep(500000);
        rightCount = rightCount-1;
      }
      //go back to previous duty level 
      io->pwm.DAT1= dutyLevelRightHold;
      rightSwitchVar = -1;
      break;
      //line tracing mode
      case 7:
      if ((backward1 == 1) || (stop1 ==1))  {
        backward1 = 0;
        stop1 = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      forward1 = 1;
      //set pins to forward (10)
      GPIO_SET(&(io->gpio), pin1);
      GPIO_CLR(&(io->gpio), pin2);  
      //set duty level to 80
      io->pwm.DAT1= 50;
      //check input sensor for black line 
      while ((!done) && (rightSwitchVar ==7)) {
      while ((GPIO_READ(&(io->gpio), 25) == 0) && (!done)&& (rightSwitchVar ==7)) {
        rightSensor = false;
      }
      while ((GPIO_READ(&(io->gpio), 25) !=0) && (!done)&& (rightSwitchVar ==7)) {
        //turn right

        //decrease to 0
        io->pwm.DAT2 = 100;
        io->pwm.DAT1 = 0;
      //go back to previous duty level 
      }
      io->pwm.DAT2 = 50;
      io->pwm.DAT1= 50;
      }
      case -1:
      break;

      default:
        break;
    }
  } while (!done);

  printf( "Right thread exiting\n" );
  return (void *)0;
}

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
//thread function to get user input and set controls
void *ThreadKey( void * arg )
{
  struct key_thread_parameter *thread_key_parameter = (struct key_thread_parameter *)arg;
  printf("Please enter m1 for manual control, m2 for self driving mode");
  printf("\nHw6>");
  fcntl( STDIN_FILENO, F_SETFL, O_NONBLOCK);
  int ch; 
  do
  {
    switch (get_pressed_key())
    {
      case 'm':
        printf("m");
        //wait for a character
        while ((ch = get_pressed_key()) == EOF);
        if (ch == '1') {
        //set the mode
          m1 = true;
          m2 = false;
          printf("1");
          printf("Mode set to m1");
          printf("\nHw6m1>");
        }
        else if (ch == '2') {
        m1 = false;
        m2 = true;
        printf("2");
        printf("Mode set to m2");
        printf("\nHw6m2>");
        }
      else {
        printf("Invalid command \n ");
        }
    
        break;
      case 'q':
        done = true;
        rightSwitchVar = -1;
         leftSwitchVar = -1;
        break;

      //stop
      case 's':
        stop1 = 1;
        rightSwitchVar =0;
        leftSwitchVar = 0;
        printf("s");
        if (m1) {
          printf("\nHw6m1>");
        }
        if (m2) {
          printf("\nHw6m2>");
        }
        break;
      //forward
      case 'w':

        if (m1) {
          forward1 = 1;
          rightSwitchVar =1;
          leftSwitchVar = 1;
        //printf("in threadkey %i",switchVar);
          printf("w\nHw6m1>");
        }
        if (m2) {
          forward1 = 1;
          rightSwitchVar =7;
          leftSwitchVar = 7;
          printf("w\nHw6m2>");
        }
        break;
      //backward
      case 'x':
        if (m1) {
          backward1 = 1;
          rightSwitchVar =2;
           leftSwitchVar = 2;
          printf("x\nHw6m1>");
        }
        break;
      //faster
      case 'i':
        if (m1) {
          rightSwitchVar =3;
           leftSwitchVar = 3;
          printf("i\nHw6m1>");
        }
        break;
      //slower 
      case 'j':
      if (m1) {
        rightSwitchVar = 4;
        leftSwitchVar = 4;
        printf("j\nHw6m1>");
      }
        break;
      //left
      case 'a':
      if (m1) {
        rightSwitchVar = 5;
        leftSwitchVar = 5;
        leftCount = leftCount + 1;
        printf("a\nHw6m1>");
      }
        break;
      //right 
      case 'd':
      if (m1) {
        rightSwitchVar = 6;
        leftSwitchVar = 6;
        rightCount = rightCount+1;
        printf("d\nHw6m1>");
      }
        break;
      //no input
      case EOF: 
        break;

      default:
        break;
    }
  } while (!done);
  printf( "key thread exiting\n" );

  return (void *)0;
}



int main( void )
{
 
  pthread_t                       left_handle;
  pthread_t                       right_handle;
  pthread_t                       thread_key_handle;
  struct thread_parameter         left_parameter;
  struct thread_parameter         right_parameter;
  struct key_thread_parameter     thread_key_parameter;
  volatile struct io_peripherals *io;
  io = import_registers();
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned long)io );

    enable_pwm_clock( io );
     /* set the pin function to alternate function 0 for GPIO12 */
     /* set the pin function to alternate function 0 for GPIO13 */
     io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
     io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;
    //set all pins to output 
     io->gpio.GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT;
     io->gpio.GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT;
     io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
     io->gpio.GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;

     /* configure the PWM channels */
     io->pwm.RNG1 = PWM_RANGE;     /* the default value */
     io->pwm.RNG2 = PWM_RANGE;     /* the default value */
     io->pwm.CTL.field.MODE1 = 0;  /* PWM mode */
     io->pwm.CTL.field.MODE2 = 0;  /* PWM mode */
     io->pwm.CTL.field.RPTL1 = 1;  /* not using FIFO, but repeat the last byte anyway */
     io->pwm.CTL.field.RPTL2 = 1;  /* not using FIFO, but repeat the last byte anyway */
     io->pwm.CTL.field.SBIT1 = 0;  /* idle low */
     io->pwm.CTL.field.SBIT2 = 0;  /* idle low */
     io->pwm.CTL.field.POLA1 = 0;  /* non-inverted polarity */
     io->pwm.CTL.field.POLA2 = 0;  /* non-inverted polarity */
     io->pwm.CTL.field.USEF1 = 0;  /* do not use FIFO */
     io->pwm.CTL.field.USEF2 = 0;  /* do not use FIFO */
     io->pwm.CTL.field.MSEN1 = 1;  /* use M/S algorithm */
     io->pwm.CTL.field.MSEN2 = 1;  /* use M/S algorithm */
     io->pwm.CTL.field.CLRF1 = 1;  /* clear the FIFO, even though it is not used */
     io->pwm.CTL.field.PWEN1 = 1;  /* enable the PWM channel */
     io->pwm.CTL.field.PWEN2 = 1;  /* enable the PWM channel */
     //try to turn the car on
    GPIO_CLR(&(io->gpio), 5);
    GPIO_CLR(&(io->gpio), 6);
    GPIO_CLR(&(io->gpio), 22);
    GPIO_CLR(&(io->gpio), 23);
    io->pwm.DAT1 = 0;
    io->pwm.DAT2 = 0;
    left_parameter.pin1 = 5;
    left_parameter.pin2 = 6;
    //left_parameter.gpio = &(io->gpio);
    //left_parameter.pwm = &(io->pwm);
    right_parameter.pin1 = 22;
    right_parameter.pin2 = 23;
    //right_parameter.pwm = &(io->pwm);
    //right_parameter.gpio = &(io->gpio);
    pthread_create( &left_handle, 0, leftHandle, (void *)&left_parameter );
    pthread_create( &right_handle, 0, rightHandle, (void *)&right_parameter );
    pthread_create( &thread_key_handle, 0, ThreadKey, (void *)&thread_key_parameter );
     //printf("got here");
    pthread_join( left_handle, 0 );
    pthread_join( right_handle, 0 );
    pthread_join( thread_key_handle, 0 );
    //stop the car before quititng
    GPIO_CLR(&(io->gpio), 5);
    GPIO_CLR(&(io->gpio), 6);
    GPIO_CLR(&(io->gpio), 22);
    GPIO_CLR(&(io->gpio), 23);
    io->pwm.DAT1 = 0;
    io->pwm.DAT2 = 0;
    
  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}

