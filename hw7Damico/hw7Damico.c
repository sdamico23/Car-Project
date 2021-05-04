/**************************************************
* CMPEN 473, Spring 2021, Penn State University
* 
* Homework 7 
* Revision V1.1
* On 3/18/2021
* By Stephen D'Amico
* 
***************************************************/
 
/* Homework 7 Car Self driving program
 * m1 to manually drive the car while recording data points from IMU
 * m2 for self driving around line while recording data points from IMU
 * m0 to record 9 data points from IMU 20 times a second
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
#include <sys/time.h>
#include <math.h>
#include "import_registers.h"
#include "cm.h"
#include "gpio.h"
#include "spi.h"
#include "pwm.h"
#include "io_peripherals.h"
#include "enable_pwm_clock.h"
#include "uart.h"
#include "MPU9250.h"
#include "wait_key.h"


#define APB_CLOCK 250000000
#define CS_PIN  8

#define ROUND_DIVISION(x,y) (((x) + (y)/2)/(y))

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
bool m0 = false;
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
//arrays to store the output values 
char outputAcc[4];
char outputMag[4];
float outputAccFloat[3];
float outputMagFloat[3];
int outputAccInt[3];
int outputMagInt[3];
//for distance calculations
float distancem1 = 0;
float distancem2 = 0;
float avgVelm1 = 0;
float avgVelm2 = 0;

//preallocate arrayss to hold readings for up to 20 mins
char m1outputString[864000];
char m2outputString[864000];
char titleStr[] = "AC MG\n";

//initialize map, -1 will be printed as blank
int mapArr[20][20] = {  
   {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,  
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
   {0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0} , 
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
};





void transact_SPI_no_CS(                          /* send/receive SPI data without toggling the chip select */
    uint8_t const *                 write_data,   /* the data to send to a SPI device */
    uint8_t *                       read_data,    /* the data read from the SPI device */
    size_t                          data_length,  /* the length of data to send/receive */
    volatile struct gpio_register * gpio,         /* the GPIO address */
    volatile struct spi_register *  spi )         /* the SPI address */
{
  size_t  write_count;  /* used to index through the bytes to be sent/received */
  size_t  read_count;   /* used to index through the bytes to be sent/received */

  /* clear out anything in the RX FIFO */
  while (spi->CS.field.RXD != 0)
  {
    (void)spi->FIFO;
  }

  /* see section 10.6.1 of the BCM2835 datasheet
   * Note that the loop below is a busy-wait loop and may burn a lot of clock cycles, depending on the amount of data to be transferred
   */
  spi->CS.field.TA = 1;
  write_count = 0;
  read_count  = 0;
  do
  {
    /* transfer bytes to the device */
    while ((write_count != data_length) &&
           (spi->CS.field.TXD != 0))
    {
      spi->FIFO = (uint32_t)*write_data;

      write_data++;
      write_count++;
    }

    /* drain the RX FIFO */
    while ((read_count != data_length) &&
           (spi->CS.field.RXD != 0))
    {
      if (read_data != NULL)
      {
        *read_data = spi->FIFO;

        read_data++;
        read_count++;
      }
      else
      {
        (void)spi->FIFO;

        read_count++;
      }
    }
  } while (spi->CS.field.DONE == 0);
  spi->CS.field.TA = 0;

  return;
}

void read_MPU9250_registers(                      /* read a register */
    MPU9250_REGISTER                address,      /* the address to read from */
    uint8_t *                       read_data,    /* the data read from the SPI device */
    size_t                          data_length,  /* the length of data to send/receive */
    int                             CS_pin,       /* the pin to toggle when communicating */
    volatile struct gpio_register * gpio,         /* the GPIO address */
    volatile struct spi_register *  spi )         /* the SPI address */
{
  union MPU9250_transaction transaction;

  transaction.field.address.AD  = address;
  transaction.field.address.R_W = 1;
  transaction.value[1]          = 0;

  GPIO_CLR( gpio, CS_pin );
  usleep( 10 );

  transact_SPI_no_CS( transaction.value, transaction.value, sizeof(transaction.field.address), gpio, spi );
  transact_SPI_no_CS( read_data, read_data, data_length, gpio, spi );

  usleep( 10 );
  GPIO_SET( gpio, CS_pin );

  return;
}

union MPU9250_transaction_field_data read_MPU9250_register( /* read a register, returning the read value */
    MPU9250_REGISTER                address,                /* the address to read from */
    int                             CS_pin,                 /* the pin to toggle when communicating */
    volatile struct gpio_register * gpio,                   /* the GPIO address */
    volatile struct spi_register *  spi )                   /* the SPI address */
{
  union MPU9250_transaction transaction;

  transaction.field.address.AD  = address;
  transaction.field.address.R_W = 1;
  transaction.value[1]          = 0;

  // this looks very strange (and it is), but it works
  GPIO_CLR( gpio, CS_pin );
  usleep( 10 );
  GPIO_CLR( gpio, CS_pin );
  usleep( 10 );

  transact_SPI_no_CS( transaction.value, transaction.value, sizeof(transaction.value), gpio, spi );

  usleep( 10 );
  GPIO_SET( gpio, CS_pin );
  usleep( 10 );
  GPIO_SET( gpio, CS_pin );

  return transaction.field.data;
}

void write_MPU9250_register(                        /* write a register */
    MPU9250_REGISTER                      address,  /* the address to read from */
    union MPU9250_transaction_field_data  value,    /* the value to write */
    int                                   CS_pin,   /* the pin to toggle when communicating */
    volatile struct gpio_register *       gpio,     /* the GPIO address */
    volatile struct spi_register *        spi )     /* the SPI address */
{
  union MPU9250_transaction transaction;

  transaction.field.address.AD  = address;
  transaction.field.address.R_W = 0;
  transaction.field.data        = value;

  // this looks very strange (and it is), but it works
  GPIO_CLR( gpio, CS_pin );
  usleep( 10 );
  GPIO_CLR( gpio, CS_pin );
  usleep( 10 );

  transact_SPI_no_CS( transaction.value, transaction.value, sizeof(transaction.value), gpio, spi );

  usleep( 10 );
  GPIO_SET( gpio, CS_pin );
  usleep( 10 );
  GPIO_SET( gpio, CS_pin );

  return;
}

void calibrate_accelerometer_and_gyroscope(
    struct calibration_data *     calibration_accelerometer,
    struct calibration_data *     calibration_gyroscope,
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio )
{
  union MPU9250_transaction_field_data  transaction;
  uint8_t                               data_block[12];
  union uint16_to_2uint8                reconstructor;
  uint16_t                              ii;
  uint16_t                              packet_count;
  int32_t                               gyro_bias_x;
  int32_t                               gyro_bias_y;
  int32_t                               gyro_bias_z;
  int32_t                               accel_bias_x;
  int32_t                               accel_bias_y;
  int32_t                               accel_bias_z;
  int32_t                               accel_bias_saved_x;
  int32_t                               accel_bias_saved_y;
  int32_t                               accel_bias_saved_z;
  uint8_t                               bias_mask_bit_x;
  uint8_t                               bias_mask_bit_y;
  uint8_t                               bias_mask_bit_z;

  // reset device
  transaction.PWR_MGMT_1.CLKSEL       = 0;
  transaction.PWR_MGMT_1.PD_PTAT      = 0;
  transaction.PWR_MGMT_1.GYRO_STANDBY = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.H_RESET      = 1;
  write_MPU9250_register( MPU9250_REGISTER_PWR_MGMT_1, transaction, CS_PIN, gpio, spi );
  usleep( 100000 );

  // get stable time source; auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator
  transaction.PWR_MGMT_1.CLKSEL       = 1;
  transaction.PWR_MGMT_1.PD_PTAT      = 0;
  transaction.PWR_MGMT_1.GYRO_STANDBY = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.H_RESET      = 0;
  write_MPU9250_register( MPU9250_REGISTER_PWR_MGMT_1, transaction, CS_PIN, gpio, spi );
  transaction.PWR_MGMT_2.DIS_ZG   = 0;
  transaction.PWR_MGMT_2.DIS_YG   = 0;
  transaction.PWR_MGMT_2.DIS_XG   = 0;
  transaction.PWR_MGMT_2.DIS_ZA   = 0;
  transaction.PWR_MGMT_2.DIS_YA   = 0;
  transaction.PWR_MGMT_2.DIS_XA   = 0;
  transaction.PWR_MGMT_2.reserved = 0;
  write_MPU9250_register( MPU9250_REGISTER_PWR_MGMT_2, transaction, CS_PIN, gpio, spi );
  usleep( 200000 );

  // configure device for bias calculation
  transaction.INT_ENABLE.RAW_RDY_EN    = 0; // disable all interrupts
  transaction.INT_ENABLE.reserved0     = 0;
  transaction.INT_ENABLE.FSYNC_INT_EN  = 0;
  transaction.INT_ENABLE.FIFO_OFLOW_EN = 0;
  transaction.INT_ENABLE.reserved1     = 0;
  transaction.INT_ENABLE.WOM_EN        = 0;
  transaction.INT_ENABLE.reserved2     = 0;
  write_MPU9250_register( MPU9250_REGISTER_INT_ENABLE, transaction, CS_PIN, gpio, spi );
  transaction.FIFO_EN.SLV0         = 0; // disable FIFO
  transaction.FIFO_EN.SLV1         = 0;
  transaction.FIFO_EN.SLV2         = 0;
  transaction.FIFO_EN.ACCEL        = 0;
  transaction.FIFO_EN.GYRO_ZO_UT   = 0;
  transaction.FIFO_EN.GYRO_YO_UT   = 0;
  transaction.FIFO_EN.GYRO_XO_UT   = 0;
  transaction.FIFO_EN.TEMP_FIFO_EN = 0;
  write_MPU9250_register( MPU9250_REGISTER_FIFO_EN, transaction, CS_PIN, gpio, spi );
  transaction.PWR_MGMT_1.CLKSEL       = 0;  // turn on internal clock source
  transaction.PWR_MGMT_1.PD_PTAT      = 0;
  transaction.PWR_MGMT_1.GYRO_STANDBY = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.H_RESET      = 0;
  write_MPU9250_register( MPU9250_REGISTER_PWR_MGMT_1, transaction, CS_PIN, gpio, spi );
  transaction.I2C_MST_CTRL.I2C_MST_CLK   = 0; // disable I2C master
  transaction.I2C_MST_CTRL.I2C_MST_P_NSR = 0;
  transaction.I2C_MST_CTRL.SLV_3_FIFO_EN = 0;
  transaction.I2C_MST_CTRL.WAIT_FOR_ES   = 0;
  transaction.I2C_MST_CTRL.MULT_MST_EN   = 0;
  write_MPU9250_register( MPU9250_REGISTER_I2C_MST_CTRL, transaction, CS_PIN, gpio, spi );
  transaction.USER_CTRL.SIG_COND_RST = 0; // disable FIFO and I2C master modes
  transaction.USER_CTRL.I2C_MST_RST  = 0;
  transaction.USER_CTRL.FIFO_RST     = 0;
  transaction.USER_CTRL.reserved0    = 0;
  transaction.USER_CTRL.I2C_IF_DIS   = 0;
  transaction.USER_CTRL.I2C_MST_EN   = 0;
  transaction.USER_CTRL.FIFO_EN      = 0;
  transaction.USER_CTRL.reserved1    = 0;
  write_MPU9250_register( MPU9250_REGISTER_USER_CTRL, transaction, CS_PIN, gpio, spi );
  transaction.USER_CTRL.SIG_COND_RST = 0; // reset FIFO and DMP
  transaction.USER_CTRL.I2C_MST_RST  = 0;
  transaction.USER_CTRL.FIFO_RST     = 1;
  transaction.USER_CTRL.reserved0    = 0;
  transaction.USER_CTRL.I2C_IF_DIS   = 0;
  transaction.USER_CTRL.I2C_MST_EN   = 0;
  transaction.USER_CTRL.FIFO_EN      = 0;
  transaction.USER_CTRL.reserved1    = 0;
  write_MPU9250_register( MPU9250_REGISTER_USER_CTRL, transaction, CS_PIN, gpio, spi );
  usleep( 15000 );

  // configure MPU9250 gyro and accelerometer for bias calculation
  transaction.CONFIG.DLPF_CFG     = 1;  // set low-pass filter to 188Hz
  transaction.CONFIG.EXT_SYNC_SET = 0;
  transaction.CONFIG.FIFO_MODE    = 0;
  transaction.CONFIG.reserved     = 0;
  write_MPU9250_register( MPU9250_REGISTER_CONFIG, transaction, CS_PIN, gpio, spi );
  transaction.SMPLRT_DIV.SMPLRT_DIV = 0;  // set sample rate to 1kHz
  write_MPU9250_register( MPU9250_REGISTER_SMPLRT_DIV, transaction, CS_PIN, gpio, spi );
  transaction.GYRO_CONFIG.FCHOICE_B   = 0; // set gyro full-scale to 250dps, maximum sensitivity
  transaction.GYRO_CONFIG.reserved    = 0;
  transaction.GYRO_CONFIG.GYRO_FS_SEL = 0;
  transaction.GYRO_CONFIG.ZGYRO_Cten  = 0;
  transaction.GYRO_CONFIG.YGYRO_Cten  = 0;
  transaction.GYRO_CONFIG.XGYRO_Cten  = 0;
  write_MPU9250_register( MPU9250_REGISTER_GYRO_CONFIG, transaction, CS_PIN, gpio, spi );
  transaction.ACCEL_CONFIG.reserved     = 0; // set accelerometer full-scale to 2g, maximum sensitivity
  transaction.ACCEL_CONFIG.ACCEL_FS_SEL = 0;
  transaction.ACCEL_CONFIG.az_st_en     = 0;
  transaction.ACCEL_CONFIG.ay_st_en     = 0;
  transaction.ACCEL_CONFIG.ax_st_en     = 0;
  write_MPU9250_register( MPU9250_REGISTER_ACCEL_CONFIG, transaction, CS_PIN, gpio, spi );

  calibration_accelerometer->scale = 2.0/32768.0;  // measurement scale/signed numeric range
  calibration_accelerometer->offset_x = 0;
  calibration_accelerometer->offset_y = 0;
  calibration_accelerometer->offset_z = 0;

  calibration_gyroscope->scale = 250.0/32768.0;
  calibration_gyroscope->offset_x = 0;
  calibration_gyroscope->offset_y = 0;
  calibration_gyroscope->offset_z = 0;

  // configure FIFO to capture accelerometer and gyro data for bias calculation
  transaction.USER_CTRL.SIG_COND_RST = 0; // enable FIFO
  transaction.USER_CTRL.I2C_MST_RST  = 0;
  transaction.USER_CTRL.FIFO_RST     = 0;
  transaction.USER_CTRL.reserved0    = 0;
  transaction.USER_CTRL.I2C_IF_DIS   = 0;
  transaction.USER_CTRL.I2C_MST_EN   = 0;
  transaction.USER_CTRL.FIFO_EN      = 1;
  transaction.USER_CTRL.reserved1    = 0;
  write_MPU9250_register( MPU9250_REGISTER_USER_CTRL, transaction, CS_PIN, gpio, spi );
  transaction.FIFO_EN.SLV0         = 0; // enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU9250)
  transaction.FIFO_EN.SLV1         = 0;
  transaction.FIFO_EN.SLV2         = 0;
  transaction.FIFO_EN.ACCEL        = 1;
  transaction.FIFO_EN.GYRO_ZO_UT   = 1;
  transaction.FIFO_EN.GYRO_YO_UT   = 1;
  transaction.FIFO_EN.GYRO_XO_UT   = 1;
  transaction.FIFO_EN.TEMP_FIFO_EN = 0;
  write_MPU9250_register( MPU9250_REGISTER_FIFO_EN, transaction, CS_PIN, gpio, spi );
  usleep( 40000 );  // accumulate 40 samples in 40 milliseconds = 480 bytes

  // at end of sample accumulation, turn off FIFO sensor read
  transaction.FIFO_EN.SLV0         = 0; // disable gyro and accelerometer sensors for FIFO
  transaction.FIFO_EN.SLV1         = 0;
  transaction.FIFO_EN.SLV2         = 0;
  transaction.FIFO_EN.ACCEL        = 0;
  transaction.FIFO_EN.GYRO_ZO_UT   = 0;
  transaction.FIFO_EN.GYRO_YO_UT   = 0;
  transaction.FIFO_EN.GYRO_XO_UT   = 0;
  transaction.FIFO_EN.TEMP_FIFO_EN = 0;
  write_MPU9250_register( MPU9250_REGISTER_FIFO_EN, transaction, CS_PIN, gpio, spi );
  read_MPU9250_registers( MPU9250_REGISTER_FIFO_COUNTH, data_block, 2, CS_PIN, gpio, spi ); // read FIFO sample count
  reconstructor.field.H = data_block[0];
  reconstructor.field.L = data_block[1];
  packet_count = reconstructor.unsigned_value / 12; // how many sets of full gyro and accelerometer data for averaging

  accel_bias_x = 0;
  accel_bias_y = 0;
  accel_bias_z = 0;
  gyro_bias_x = 0;
  gyro_bias_y = 0;
  gyro_bias_z = 0;
  for (ii = 0; ii < packet_count; ii++)
  {
    union uint16_to_2uint8  reconstructor_accel_x;
    union uint16_to_2uint8  reconstructor_accel_y;
    union uint16_to_2uint8  reconstructor_accel_z;
    union uint16_to_2uint8  reconstructor_gyro_x;
    union uint16_to_2uint8  reconstructor_gyro_y;
    union uint16_to_2uint8  reconstructor_gyro_z;

    read_MPU9250_registers( MPU9250_REGISTER_FIFO_R_W, data_block, 12, CS_PIN, gpio, spi ); // read data for averaging

    reconstructor_accel_x.field.H = data_block[0]; // sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    reconstructor_accel_x.field.L = data_block[1];
    reconstructor_accel_y.field.H = data_block[2];
    reconstructor_accel_y.field.L = data_block[3];
    reconstructor_accel_z.field.H = data_block[4];
    reconstructor_accel_z.field.L = data_block[5];
    reconstructor_gyro_x.field.H  = data_block[6];
    reconstructor_gyro_x.field.L  = data_block[7];
    reconstructor_gyro_y.field.H  = data_block[8];
    reconstructor_gyro_y.field.L  = data_block[9];
    reconstructor_gyro_z.field.H  = data_block[10];
    reconstructor_gyro_z.field.L  = data_block[11];

    // the compiler is pitching a fit over this loop, this print statement seems to fix it (?!?)
    printf("%d %d %d %d %d %d\n",
        reconstructor_accel_x.signed_value,
        reconstructor_accel_y.signed_value,
        reconstructor_accel_z.signed_value,
        reconstructor_gyro_x.signed_value,
        reconstructor_gyro_y.signed_value,
        reconstructor_gyro_z.signed_value );

    accel_bias_x += reconstructor_accel_x.signed_value;
    accel_bias_y += reconstructor_accel_y.signed_value;
    accel_bias_z += reconstructor_accel_z.signed_value;
    gyro_bias_x  += reconstructor_gyro_x.signed_value;
    gyro_bias_y  += reconstructor_gyro_y.signed_value;
    gyro_bias_z  += reconstructor_gyro_z.signed_value;
  }
  accel_bias_x /= (int32_t)packet_count;
  accel_bias_y /= (int32_t)packet_count;
  accel_bias_z /= (int32_t)packet_count;
  gyro_bias_x /= (int32_t)packet_count;
  gyro_bias_y /= (int32_t)packet_count;
  gyro_bias_z /= (int32_t)packet_count;

  if (accel_bias_z > 0) // remove gravity from the z-axis accelerometer bias calculation
  {
    accel_bias_z -= (int32_t)(1.0/calibration_accelerometer->scale);
  }
  else
  {
    accel_bias_z += (int32_t)(1.0/calibration_accelerometer->scale);
  }

  // construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device start
  // and push gyro biases to hardware registers
  reconstructor.signed_value = -1*gyro_bias_x/4;
  transaction.XG_OFFSET_H.X_OFFS_USR = reconstructor.field.H; // divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  write_MPU9250_register( MPU9250_REGISTER_XG_OFFSET_H, transaction, CS_PIN, gpio, spi );
  transaction.XG_OFFSET_L.X_OFFS_USR = reconstructor.field.L; // biases are additive, so change sign on calculated average gyro biases
  write_MPU9250_register( MPU9250_REGISTER_XG_OFFSET_L, transaction, CS_PIN, gpio, spi );

  reconstructor.signed_value = -1*gyro_bias_y/4;
  transaction.YG_OFFSET_H.Y_OFFS_USR = reconstructor.field.H;
  write_MPU9250_register( MPU9250_REGISTER_YG_OFFSET_H, transaction, CS_PIN, gpio, spi );
  transaction.YG_OFFSET_L.Y_OFFS_USR = reconstructor.field.L;
  write_MPU9250_register( MPU9250_REGISTER_YG_OFFSET_L, transaction, CS_PIN, gpio, spi );

  reconstructor.signed_value = -1*gyro_bias_z/4;
  transaction.ZG_OFFSET_H.Z_OFFS_USR = reconstructor.field.H;
  write_MPU9250_register( MPU9250_REGISTER_ZG_OFFSET_H, transaction, CS_PIN, gpio, spi );
  transaction.ZG_OFFSET_L.Z_OFFS_USR = reconstructor.field.L;
  write_MPU9250_register( MPU9250_REGISTER_ZG_OFFSET_L, transaction, CS_PIN, gpio, spi );

  // output scaled gyro biases
  calibration_gyroscope->offset_x = ((float)gyro_bias_x)*calibration_gyroscope->scale;
  calibration_gyroscope->offset_y = ((float)gyro_bias_y)*calibration_gyroscope->scale;
  calibration_gyroscope->offset_z = ((float)gyro_bias_z)*calibration_gyroscope->scale;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  read_MPU9250_registers( MPU9250_REGISTER_XA_OFFSET_H, data_block, 2, CS_PIN, gpio, spi );
  reconstructor.field.H = data_block[0];
  reconstructor.field.L = data_block[1];
  accel_bias_saved_x = reconstructor.signed_value;
  read_MPU9250_registers( MPU9250_REGISTER_YA_OFFSET_H, data_block, 2, CS_PIN, gpio, spi );
  reconstructor.field.H = data_block[0];
  reconstructor.field.L = data_block[1];
  accel_bias_saved_y = reconstructor.signed_value;
  read_MPU9250_registers( MPU9250_REGISTER_ZA_OFFSET_H, data_block, 2, CS_PIN, gpio, spi );
  reconstructor.field.H = data_block[0];
  reconstructor.field.L = data_block[1];
  accel_bias_saved_z = reconstructor.signed_value;

  // if temperature compensation bit is set, record that fact
  bias_mask_bit_x = (uint8_t)(accel_bias_saved_x%2);
  bias_mask_bit_y = (uint8_t)(accel_bias_saved_y%2);
  bias_mask_bit_z = (uint8_t)(accel_bias_saved_z%2);

  // construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_saved_x -= accel_bias_x/8; // subtract caluclated averaged accelerometer bias scaled to 2048 LSB/g (16g full scale)
  accel_bias_saved_y -= accel_bias_y/8;
  accel_bias_saved_z -= accel_bias_z/8;

  // apparently this is not working for acceleration biases in the MPU9250
  // are we handling the temperature correction bit properly?
  // push accelerometer biases to hardware registers
  reconstructor.signed_value = accel_bias_saved_x;
  transaction.XA_OFFSET_H.XA_OFFS = reconstructor.field.H;
  write_MPU9250_register( MPU9250_REGISTER_XA_OFFSET_H, transaction, CS_PIN, gpio, spi );
  transaction.XA_OFFSET_L.reserved = bias_mask_bit_x; // preserve temperature compensation bit when writing back to accelerometer bias registers
  transaction.XA_OFFSET_L.XA_OFFS = reconstructor.field.L;
  write_MPU9250_register( MPU9250_REGISTER_XA_OFFSET_L, transaction, CS_PIN, gpio, spi );

  reconstructor.signed_value = accel_bias_saved_y;
  transaction.YA_OFFSET_H.YA_OFFS = reconstructor.field.H;
  write_MPU9250_register( MPU9250_REGISTER_YA_OFFSET_H, transaction, CS_PIN, gpio, spi );
  transaction.YA_OFFSET_L.reserved = bias_mask_bit_y; // preserve temperature compensation bit when writing back to accelerometer bias registers
  transaction.YA_OFFSET_L.YA_OFFS = reconstructor.field.L;
  write_MPU9250_register( MPU9250_REGISTER_YA_OFFSET_L, transaction, CS_PIN, gpio, spi );

  reconstructor.signed_value = accel_bias_saved_z;
  transaction.ZA_OFFSET_H.ZA_OFFS = reconstructor.field.H;
  write_MPU9250_register( MPU9250_REGISTER_ZA_OFFSET_H, transaction, CS_PIN, gpio, spi );
  transaction.ZA_OFFSET_L.reserved = bias_mask_bit_z; // preserve temperature compensation bit when writing back to accelerometer bias registers
  transaction.ZA_OFFSET_L.ZA_OFFS = reconstructor.field.L;
  write_MPU9250_register( MPU9250_REGISTER_ZA_OFFSET_L, transaction, CS_PIN, gpio, spi );

  // output scaled accelerometer biases
  calibration_accelerometer->offset_x = ((float)accel_bias_x)*calibration_accelerometer->scale;
  calibration_accelerometer->offset_y = ((float)accel_bias_y)*calibration_accelerometer->scale;
  calibration_accelerometer->offset_z = ((float)accel_bias_z)*calibration_accelerometer->scale;

  return;
}

void initialize_accelerometer_and_gyroscope(
    struct calibration_data *     calibration_accelerometer,
    struct calibration_data *     calibration_gyroscope,
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio )
{
  union MPU9250_transaction_field_data  transaction;

  /* print WHO_AM_I */
  printf( "WHOAMI (0x71) = 0x%2.2X\n",
      read_MPU9250_register( MPU9250_REGISTER_WHO_AM_I, CS_PIN, gpio, spi ).WHO_AM_I.WHOAMI );

  // based off https://github.com/brianc118/MPU9250/blob/master/MPU9250.cpp

  calibrate_accelerometer_and_gyroscope( calibration_accelerometer, calibration_gyroscope, spi, gpio );

  // reset MPU9205
  transaction.PWR_MGMT_1.CLKSEL        = 0;
  transaction.PWR_MGMT_1.PD_PTAT       = 0;
  transaction.PWR_MGMT_1.GYRO_STANDBY  = 0;
  transaction.PWR_MGMT_1.CYCLE         = 0;
  transaction.PWR_MGMT_1.SLEEP         = 0;
  transaction.PWR_MGMT_1.H_RESET       = 1;
  write_MPU9250_register( MPU9250_REGISTER_PWR_MGMT_1, transaction, CS_PIN, gpio, spi );
  usleep( 1000 ); // wait for all registers to reset

  // clock source
  transaction.PWR_MGMT_1.CLKSEL       = 1;
  transaction.PWR_MGMT_1.PD_PTAT      = 0;
  transaction.PWR_MGMT_1.GYRO_STANDBY = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.H_RESET      = 0;
  write_MPU9250_register( MPU9250_REGISTER_PWR_MGMT_1, transaction, CS_PIN, gpio, spi );
  usleep( 1000 );

  // enable acc & gyro
  transaction.PWR_MGMT_2.DIS_ZG   = 0;
  transaction.PWR_MGMT_2.DIS_YG   = 0;
  transaction.PWR_MGMT_2.DIS_XG   = 0;
  transaction.PWR_MGMT_2.DIS_ZA   = 0;
  transaction.PWR_MGMT_2.DIS_YA   = 0;
  transaction.PWR_MGMT_2.DIS_XA   = 0;
  transaction.PWR_MGMT_2.reserved = 0;
  write_MPU9250_register( MPU9250_REGISTER_PWR_MGMT_1, transaction, CS_PIN, gpio, spi );
  usleep( 1000 );

  // use DLPF set gyro bandwidth 184Hz, temperature bandwidth 188Hz
  transaction.CONFIG.DLPF_CFG     = 1;
  transaction.CONFIG.EXT_SYNC_SET = 0;
  transaction.CONFIG.FIFO_MODE    = 0;
  transaction.CONFIG.reserved     = 0;
  write_MPU9250_register( MPU9250_REGISTER_CONFIG, transaction, CS_PIN, gpio, spi );
  usleep( 1000 );

  // +-250dps
  transaction.GYRO_CONFIG.FCHOICE_B   = 0;
  transaction.GYRO_CONFIG.reserved    = 0;
  transaction.GYRO_CONFIG.GYRO_FS_SEL = 0;
  transaction.GYRO_CONFIG.ZGYRO_Cten  = 0;
  transaction.GYRO_CONFIG.YGYRO_Cten  = 0;
  transaction.GYRO_CONFIG.XGYRO_Cten  = 0;
  write_MPU9250_register( MPU9250_REGISTER_GYRO_CONFIG, transaction, CS_PIN, gpio, spi );
  usleep( 1000 );

  // +-2G
  transaction.ACCEL_CONFIG.reserved     = 0;
  transaction.ACCEL_CONFIG.ACCEL_FS_SEL = 0;
  transaction.ACCEL_CONFIG.az_st_en     = 0;
  transaction.ACCEL_CONFIG.ay_st_en     = 0;
  transaction.ACCEL_CONFIG.ax_st_en     = 0;
  write_MPU9250_register( MPU9250_REGISTER_ACCEL_CONFIG, transaction, CS_PIN, gpio, spi );
  usleep( 1000 );

  // set acc data rates,enable acc LPF, bandwidth 184Hz
  transaction.ACCEL_CONFIG_2.A_DLPF_CFG      = 0;
  transaction.ACCEL_CONFIG_2.ACCEL_FCHOICE_B = 0;
  transaction.ACCEL_CONFIG_2.reserved        = 0;
  write_MPU9250_register( MPU9250_REGISTER_ACCEL_CONFIG_2, transaction, CS_PIN, gpio, spi );
  usleep( 1000 );

  // skipping configure int pin

  // I2C master mode and set I2C_IF_DIS to disable slave mode I2C bus
  transaction.USER_CTRL.SIG_COND_RST = 0;
  transaction.USER_CTRL.I2C_MST_RST  = 0;
  transaction.USER_CTRL.FIFO_RST     = 0;
  transaction.USER_CTRL.reserved0    = 0;
  transaction.USER_CTRL.I2C_IF_DIS   = 1;
  transaction.USER_CTRL.I2C_MST_EN   = 1;
  transaction.USER_CTRL.FIFO_EN      = 0;
  transaction.USER_CTRL.reserved1    = 0;
  write_MPU9250_register( MPU9250_REGISTER_USER_CTRL, transaction, CS_PIN, gpio, spi );
  usleep( 1000 );

  // I2C configuration multi-master IIC 400KHz
  transaction.I2C_MST_CTRL.I2C_MST_CLK   = 13;
  transaction.I2C_MST_CTRL.I2C_MST_P_NSR = 0;
  transaction.I2C_MST_CTRL.SLV_3_FIFO_EN = 0;
  transaction.I2C_MST_CTRL.WAIT_FOR_ES   = 0;
  transaction.I2C_MST_CTRL.MULT_MST_EN   = 0;
  write_MPU9250_register( MPU9250_REGISTER_I2C_MST_CTRL, transaction, CS_PIN, gpio, spi );
  usleep( 1000 );

  // set acc scale = 2G, divider = 16384
  transaction.ACCEL_CONFIG.reserved     = 0;
  transaction.ACCEL_CONFIG.ACCEL_FS_SEL = 0;
  transaction.ACCEL_CONFIG.az_st_en     = 0;
  transaction.ACCEL_CONFIG.ay_st_en     = 0;
  transaction.ACCEL_CONFIG.ax_st_en     = 0;
  write_MPU9250_register( MPU9250_REGISTER_ACCEL_CONFIG, transaction, CS_PIN, gpio, spi );
  usleep( 1000 );

  // set gyro scale = 250dps, divider = 131
  transaction.GYRO_CONFIG.FCHOICE_B   = 0;
  transaction.GYRO_CONFIG.reserved    = 0;
  transaction.GYRO_CONFIG.GYRO_FS_SEL = 0;
  transaction.GYRO_CONFIG.ZGYRO_Cten  = 0;
  transaction.GYRO_CONFIG.YGYRO_Cten  = 0;
  transaction.GYRO_CONFIG.XGYRO_Cten  = 0;
  write_MPU9250_register( MPU9250_REGISTER_GYRO_CONFIG, transaction, CS_PIN, gpio, spi );
  usleep( 1000 );

  return;
}

void initialize_magnetometer(
    struct calibration_data *     calibration_magnetometer,
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio )
{
  union MPU9250_transaction_field_data  transaction;
  uint8_t                               data_block[3];
  float                                 magnetometer_sensitivity_scale_factor = 0.15;

  // set the I2C slave address of AK8963 and set for write
  transaction.I2C_SLV0_ADDR.I2C_ID_0      = AK8963_ADDRESS;
  transaction.I2C_SLV0_ADDR.I2C_SLV0_RNW  = 0;
  write_MPU9250_register( MPU9250_REGISTER_I2C_SLV0_ADDR, transaction, CS_PIN, gpio, spi );
  usleep( 1000 );

  // I2C slave 0 register address from where to begin data transfer
  transaction.I2C_SLV0_REG.I2C_SLV0_REG = AK8963_REGISTER_CNTL2;
  write_MPU9250_register( MPU9250_REGISTER_I2C_SLV0_REG, transaction, CS_PIN, gpio, spi );
  usleep( 1000 );

  // reset AK8963
  transaction.CNTL2.SRST      = 1;
  transaction.CNTL2.reserved  = 0;
  write_MPU9250_register( MPU9250_REGISTER_I2C_SLV0_DO, transaction, CS_PIN, gpio, spi );
  usleep( 1000 );

  // enable I2C and set 1 byte
  transaction.I2C_SLV0_CTRL.I2C_SLV0_LENG     = 1;
  transaction.I2C_SLV0_CTRL.I2C_SLV0_GRP      = 0;
  transaction.I2C_SLV0_CTRL.I2C_SLV0_REG_DIS  = 0;
  transaction.I2C_SLV0_CTRL.I2C_SLV0_BYTE_SW  = 0;
  transaction.I2C_SLV0_CTRL.I2C_SLV0_EN       = 1;
  write_MPU9250_register( MPU9250_REGISTER_I2C_SLV0_CTRL, transaction, CS_PIN, gpio, spi );
  usleep( 1000 );

  // I2C slave 0 register address from where to being data transfer
  transaction.I2C_SLV0_REG.I2C_SLV0_REG = AK8963_REGISTER_CNTL1;
  write_MPU9250_register( MPU9250_REGISTER_I2C_SLV0_REG, transaction, CS_PIN, gpio, spi );
  usleep( 1000 );

  // register value to 100Hz continuous measurement in 16bit
  transaction.CNTL1.MODE      = 6;
  transaction.CNTL1.BIT       = 0;
  transaction.CNTL1.reserved  = 0;
  write_MPU9250_register( MPU9250_REGISTER_I2C_SLV0_DO, transaction, CS_PIN, gpio, spi );
  usleep( 1000 );

  // enable I2C and set 1 byte
  transaction.I2C_SLV0_CTRL.I2C_SLV0_LENG     = 1;
  transaction.I2C_SLV0_CTRL.I2C_SLV0_GRP      = 0;
  transaction.I2C_SLV0_CTRL.I2C_SLV0_REG_DIS  = 0;
  transaction.I2C_SLV0_CTRL.I2C_SLV0_BYTE_SW  = 0;
  transaction.I2C_SLV0_CTRL.I2C_SLV0_EN       = 1;
  write_MPU9250_register( MPU9250_REGISTER_I2C_SLV0_CTRL, transaction, CS_PIN, gpio, spi );
  usleep( 1000 );

  // get the magnetometer calibration... extracted from the "calib_mag" function at https://github.com/brianc118/MPU9250/blob/master/MPU9250.cpp
  transaction.I2C_SLV0_ADDR.I2C_ID_0          = AK8963_ADDRESS;                             // set the I2C slave address of the AK8963 and set for read
  transaction.I2C_SLV0_ADDR.I2C_SLV0_RNW      = 1;
  write_MPU9250_register( MPU9250_REGISTER_I2C_SLV0_ADDR, transaction, CS_PIN, gpio, spi );
  transaction.I2C_SLV0_REG.I2C_SLV0_REG       = AK8963_REGISTER_ASAX;                       // I2C slave 0 register address from where to begin data transfer
  write_MPU9250_register( MPU9250_REGISTER_I2C_SLV0_REG, transaction, CS_PIN, gpio, spi );
  transaction.I2C_SLV0_CTRL.I2C_SLV0_LENG     = 3;                                          // read 3 bytes from the magnetometer
  transaction.I2C_SLV0_CTRL.I2C_SLV0_GRP      = 0;
  transaction.I2C_SLV0_CTRL.I2C_SLV0_REG_DIS  = 0;
  transaction.I2C_SLV0_CTRL.I2C_SLV0_BYTE_SW  = 0;
  transaction.I2C_SLV0_CTRL.I2C_SLV0_EN       = 1;
  write_MPU9250_register( MPU9250_REGISTER_I2C_SLV0_CTRL, transaction, CS_PIN, gpio, spi );

  usleep( 200000 );

  read_MPU9250_registers( MPU9250_REGISTER_EXT_SENS_DATA_00, data_block, sizeof(data_block), CS_PIN, gpio, spi );
  calibration_magnetometer->scale = (float)1;
  calibration_magnetometer->offset_x = ((((float)data_block[0])-128.0)/256.0+1.0)*magnetometer_sensitivity_scale_factor;
  calibration_magnetometer->offset_y = ((((float)data_block[1])-128.0)/256.0+1.0)*magnetometer_sensitivity_scale_factor;
  calibration_magnetometer->offset_z = ((((float)data_block[2])-128.0)/256.0+1.0)*magnetometer_sensitivity_scale_factor;

  return;
}

void read_accelerometer(
    struct calibration_data *     calibration_accelerometer,
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio )
{
  uint8_t                   data_block[6];
  union uint16_to_2uint8    ACCEL_XOUT;
  union uint16_to_2uint8    ACCEL_YOUT;
  union uint16_to_2uint8    ACCEL_ZOUT;

  /*
   * poll the interrupt status register and it tells you when it is done
   * once it is done, read the data registers
   */
  do
  {
    usleep( 1000 );
  } while ((read_MPU9250_register( MPU9250_REGISTER_INT_STATUS, CS_PIN, gpio, spi ).INT_STATUS.RAW_DATA_RDY_INT == 0) &&(!done));
  // read the accelerometer values
  read_MPU9250_registers( MPU9250_REGISTER_ACCEL_XOUT_H, data_block, 6, CS_PIN, gpio, spi );
  ACCEL_XOUT.field.H              = data_block[0];
  ACCEL_XOUT.field.L              = data_block[1];
  ACCEL_YOUT.field.H              = data_block[2];
  ACCEL_YOUT.field.L              = data_block[3];
  ACCEL_ZOUT.field.H              = data_block[4];
  ACCEL_ZOUT.field.L              = data_block[5];

      //printf( "Accel X: %.2f m/s^2\ty=%.2f m/s^2\tz=%.2f m/s^2\n",
      //(ACCEL_XOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_x)*9.81,
      //(ACCEL_YOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_y)*9.81,
      //(ACCEL_ZOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_z)*9.81 );
      outputAccFloat[0] = (ACCEL_XOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_x)*9.81;
      outputAccFloat[1] = (ACCEL_YOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_y)*9.81;
      outputAccFloat[2] = (ACCEL_ZOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_z)*9.81 ;
      outputAccInt[0] = (ACCEL_XOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_x)*9.81;
      outputAccInt[1] = (ACCEL_YOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_y)*9.81;
      outputAccInt[2] = (ACCEL_ZOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_z)*9.81 ;
      //scale the output to between 0 and 9 based on max values from data previously saved 
      outputAccInt[0] = (int)((abs(outputAccFloat[0]) / 1.3f) * 9.0f);
      if (outputAccInt[0] >= 9) {
        outputAccInt[0] = 9;
      }
      outputAccInt[1] = (int)((abs(outputAccFloat[1]) / 1.3f) * 9.0f);
      if (outputAccInt[1] >= 9) {
        outputAccInt[1] = 9;
      }
      outputAccInt[2] = (int)((abs(outputAccFloat[2]) / 9.0f) * 9.0f);
      if (outputAccInt[2] >= 9) {
        outputAccInt[2] = 9;
      }
      sprintf(outputAcc, "%d%d%d", outputAccInt[0], outputAccInt[1], outputAccInt[2]);
      
      //add to appriopriate output
      if ((m1) && (forward1 == 1)){
        strcat(m1outputString, outputAcc);
        //printf("m1 output string %s", m1outputString);
      }
      else if ((m2) && (forward1 == 1)){
        strcat(m2outputString, outputAcc);
      }

  return;
}



void read_magnetometer(
    struct calibration_data *     calibration_magnetometer,
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio )
{
  uint8_t                               data_block[7];
  union uint16_to_2uint8                MAG_XOUT;
  union uint16_to_2uint8                MAG_YOUT;
  union uint16_to_2uint8                MAG_ZOUT;
  union MPU9250_transaction_field_data  transaction;

  transaction.I2C_SLV0_ADDR.I2C_ID_0      = AK8963_ADDRESS; // set the I2C slave address of the AK8963 and set for read
  transaction.I2C_SLV0_ADDR.I2C_SLV0_RNW  = 1;
  write_MPU9250_register( MPU9250_REGISTER_I2C_SLV0_ADDR, transaction, CS_PIN, gpio, spi );
  transaction.I2C_SLV0_REG.I2C_SLV0_REG   = AK8963_REGISTER_HXL;  // I2C slave 0 register address from where to begin data transfer
  write_MPU9250_register( MPU9250_REGISTER_I2C_SLV0_REG, transaction, CS_PIN, gpio, spi );
  transaction.I2C_SLV0_CTRL.I2C_SLV0_LENG     = 7;  // read 7 bytes from the magnetometer
  transaction.I2C_SLV0_CTRL.I2C_SLV0_GRP      = 0;
  transaction.I2C_SLV0_CTRL.I2C_SLV0_REG_DIS  = 0;
  transaction.I2C_SLV0_CTRL.I2C_SLV0_BYTE_SW  = 0;
  transaction.I2C_SLV0_CTRL.I2C_SLV0_EN       = 1;
  write_MPU9250_register( MPU9250_REGISTER_I2C_SLV0_CTRL, transaction, CS_PIN, gpio, spi );

  read_MPU9250_registers( MPU9250_REGISTER_EXT_SENS_DATA_00, data_block, 7, CS_PIN, gpio, spi );
  // read must start from HXL and read seven bytes so that ST2 is read and the AK8963 will start the next conversion
  MAG_XOUT.field.L = data_block[0];
  MAG_XOUT.field.H = data_block[1];
  MAG_YOUT.field.L = data_block[2];
  MAG_YOUT.field.H = data_block[3];
  MAG_ZOUT.field.L = data_block[4];
  MAG_ZOUT.field.H = data_block[5];
      //printf( "Mag X: %.2f uT\ty=%.2f uT\tz=%.2f uT\n",
      //MAG_XOUT.signed_value*calibration_magnetometer->scale - calibration_magnetometer->offset_x,
      //MAG_YOUT.signed_value*calibration_magnetometer->scale - calibration_magnetometer->offset_y,
      //MAG_ZOUT.signed_value*calibration_magnetometer->scale - calibration_magnetometer->offset_z );
      //store all values to array 
      outputMagFloat[0] = MAG_XOUT.signed_value*calibration_magnetometer->scale - calibration_magnetometer->offset_x;
      outputMagFloat[1] = MAG_YOUT.signed_value*calibration_magnetometer->scale - calibration_magnetometer->offset_y;
      outputMagFloat[2] = MAG_ZOUT.signed_value*calibration_magnetometer->scale - calibration_magnetometer->offset_z;
      //printf("outputMagFloat0 = %f",outputMagFloat[0]);
      //printf("outputMagFloat1 = %f",outputMagFloat[1]);
      //printf("outputMagFloat2 = %f",outputMagFloat[2]);
      
      outputMagInt[0] = (int)MAG_XOUT.signed_value*calibration_magnetometer->scale - calibration_magnetometer->offset_x;
      outputMagInt[1] = (int)MAG_YOUT.signed_value*calibration_magnetometer->scale - calibration_magnetometer->offset_y;
      outputMagInt[2] = (int)MAG_ZOUT.signed_value*calibration_magnetometer->scale - calibration_magnetometer->offset_z;
      //scale the values from 0 to 9 based on max values from datasheet 
      outputMagInt[0] = (int)((abs(outputMagFloat[0]) / 120) * 9.0f);
      if (outputMagInt[0] >= 9) {
        outputMagInt[0] = 9;
      }
      outputMagInt[1] = (int)((abs(outputMagFloat[1]) / 120) * 9.0f);
      if (outputMagInt[1] >= 9) {
        outputMagInt[1] = 9;
      }
      outputMagInt[2] = (int)((abs(outputMagFloat[2]) /40) * 9.0f);
      if (outputMagInt[2] >= 9) {
        outputMagInt[2] = 9;
      }
      //save data to appropriate string
      sprintf(outputMag, "%d%d%d", outputMagInt[0],outputMagInt[1],outputMagInt[2] );
      if ((m1) && (forward1 ==1)){
        strcat(m1outputString, outputMag);
        strcat(m1outputString, "\n");
      }
      else if ((m2) &&(forward1 == 1)){
        strcat(m2outputString, outputMag);
        strcat(m2outputString, "\n");
      }
  return;
}

  



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
      //set pins to forward (10)
      GPIO_SET(&(io->gpio), pin1);
      GPIO_CLR(&(io->gpio), pin2); 
      //printf("car set to forward"); 
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


//imu data reading function
void *imuHandle( void * arg )
{
  struct thread_parameter * parameter = (struct thread_parameter *)arg;
  //struct for io and the IMU components
  volatile struct io_peripherals *io;
  struct calibration_data         calibration_accelerometer;
  struct calibration_data         calibration_gyroscope;
  struct calibration_data         calibration_magnetometer;
  struct timeval current_time;
  io = import_registers();
  int seconds = 0;
  int prevsecs = 0;
  int prevusecs = 0;
  //initialize delta t to 50 msec
  int deltaT = 50;
  int useconds = 0;
  float timeStamp;
  float usecTimeStamp;
  float prevAccX = 0;
  float prevAccY = 0;
  float velXm1 = 0;
  float velYm1 = 0;
  float velXm2 = 0;
  float velYm2 = 0;
  float totalVelm1 = 0;
  float totalVelm2 = 0;
  float dxm1 = 0;
  float dym1 = 0;
  float dxm2 = 0;
  float dym2 = 0;
  float deltaTSecs = 0;
  int currentVel = 0;
  int scaledCurrentVel = 0;
  int secondCounter = 0;
  int m1Counter = 0;
  int m2Counter = 0;
  int counter = 0;
  float mapDistX =0;
  float mapDistY = 0;
  int mapIndexI = 17;
  int mapIndexJ = 3;
  bool up = true;
  //files to save the data readings for the plots 
  FILE *m0fp = NULL;
  FILE *m1fp = NULL;
  FILE *m2fp = NULL;
  m0fp = fopen("m0_output.txt" ,"a");
  m1fp = fopen("m1_output.txt" ,"a");
  m2fp = fopen("m2_output.txt" ,"a");
  //intitialize files
  fputs("Acc X | Acc Y | Acc Z | Mag X | Mag Y | Mag Z | Timestamp | Delta T\n", m0fp);
  fputs("Acc X | Acc Y | Acc Z | Mag X | Mag Y | Mag Z | Timestamp | Delta T\n", m1fp);
  fputs("Acc X | Acc Y | Acc Z | Mag X | Mag Y | Mag Z | Timestamp | Delta T\n", m2fp);
  strcat(m1outputString, titleStr);
  strcat(m2outputString, titleStr);
  strcat(m1outputString, "\n");
  strcat(m2outputString, "\n");

  if (io != NULL) {
    //initialize imu chip
    initialize_accelerometer_and_gyroscope( &calibration_accelerometer, &calibration_gyroscope, &(io->spi), &(io->gpio) );
    initialize_magnetometer( &calibration_magnetometer, &(io->spi), &(io->gpio) );
  //get data from the IMU chip 
  do
    {
      //printf("reading values");
      //get current time stamp 
      gettimeofday(&current_time, NULL);
      prevsecs = seconds;
      prevusecs = useconds;
      seconds = current_time.tv_sec;
      useconds = current_time.tv_usec;
      usecTimeStamp = useconds / 1000000;
      timeStamp = seconds + usecTimeStamp;
      //calculate delta T value
      if ((seconds - prevsecs) == 0) {
        deltaT = useconds - prevusecs;
        deltaT = deltaT / 1000; //get value in ms
      }
      else if ((seconds-prevsecs) ==1) {
        int prevTotal = 1000000-prevusecs;
        prevTotal = prevTotal + useconds;
        deltaT = prevTotal / 1000.0;
      }
      deltaTSecs = (float)deltaT/1000.0f;
      //printf( "\ndelta T secs : %f\n", deltaTSecs); //correct
      //printf("\nDeltaT: %i \n", deltaT);
      //printf("seconds : %ld\nmicro seconds : %ld", current_time.tv_sec, current_time.tv_usec);
      read_accelerometer( &calibration_accelerometer, &(io->spi), &(io->gpio) );
      read_magnetometer( &calibration_magnetometer, &(io->spi), &(io->gpio) );
      if ((m1) && (forward1 ==1)){
          //calculate distance and velocity
          //printf("calculating distance and velocity m1");
          //x value
          //printf("\nvelXm1 : %f", velXm1);
          //printf("\noutputAccFloat : %f", outputAccFloat[0]);
          //printf("\ndeltaT: %i", deltaT);
          //printf("\ndeltaTsecs : %f", deltaTSecs);
          velXm1 +=(prevAccX + outputAccFloat[0])/2.0f*(deltaTSecs);
          //printf("\nx distance : %f", velXm1*deltaTSecs);
          dxm1 =velXm1*deltaTSecs;
          prevAccX = outputAccFloat[0];
          //y value
          velYm1 +=(prevAccY + outputAccFloat[1])/2.0f*(deltaTSecs);
          dym1 =velYm1*deltaTSecs;
          prevAccY = outputAccFloat[1];          
          distancem1 += sqrt(dxm1*dxm1 + dym1*dym1);
          totalVelm1 += sqrt(velXm1*velXm1 + velYm1*velYm1);
          m1Counter +=1;
          avgVelm1 = totalVelm1/m1Counter;
          
    }
    
    else if ((m2) && (forward1 == 1)){
          counter = counter +1;
          //calculate distance and velocity
          //printf("\ncalculating distance and velocity m2");
          //x value
          //printf("Accel X: %f", outputAccFloat[0]);
          //printf("Accel Y: %f", outputAccFloat[1]);
          velXm2 +=(prevAccX + outputAccFloat[0])/2.0f*(deltaTSecs);
          //printf("\nvel x : %f", velXm2);
          dxm2 =velXm2*deltaTSecs;
          mapDistX += dxm2;
          //printf("\nd x : %f", dxm2);
          prevAccX = outputAccFloat[0];
          //y value
          velYm2 +=(prevAccY + outputAccFloat[1])/2.0f*(deltaTSecs);
          //printf("\nvel y : %f", velYm2);
          dym2 =velYm2*deltaTSecs;
          mapDistY +=dym2;
          //printf("\nd y : %f", dym2);
          prevAccY = outputAccFloat[1];
          
          distancem2 += sqrt(dxm2*dxm2 + dym2*dym2);
          //printf("\ndistance m2 : %f", distancem2);
          currentVel = sqrt(velXm2*velXm2 + velYm2*velYm2);
          scaledCurrentVel = (int)currentVel;
          if (scaledCurrentVel>9) {
            scaledCurrentVel = 9;
          }
          totalVelm2 += sqrt(velXm2*velXm2 + velYm2*velYm2);
          //printf("\ntotalVel m2 : %f", totalVelm2);
          m2Counter +=1;
          if (m2Counter == 20) {
            m2Counter = 0;
            //printf("MapDistX : %f", mapDistX);
            //printf("MapDistY : %f", mapDistY);
            
            //always negative because only going forward in m2
            if (abs(mapDistX) >= 0.2f) {
              //printf("\adding Iindex\n");
              if (up) {
                mapIndexI -= 1;
              }
              else if (!up) {
                mapIndexI += 1;
              }
              if (mapIndexI <= 2) {
                up = false;
              }
              else if (mapIndexI >= 18) {
                up = true;
            }
            mapDistX = 0;
          }
            
            //negative if going right, positive if left
            if (mapDistY <= -0.1f) {
              //printf("\ading to  Jindex\n");
              mapIndexJ -=1;
              mapDistY = 0;
          }
           else if (mapDistY >= 0.1f) {
             //printf("\nsubtracting Jindex\n");
              if (up) {
                
              mapIndexJ +=1;
              mapDistY = 0;
            }
            else if (!up) {
              mapIndexJ -=1;
              mapDistY = 0;
            }
          }
          
          mapArr[mapIndexI][mapIndexJ] = scaledCurrentVel;
        }
        avgVelm2 = totalVelm2/counter;
    }
      
      //write data to the appropriate file
      if (m0==1) {
        for (unsigned i = 0; i < 3; i++) {
          fprintf(m0fp, "%f", outputAccFloat[i]);
          fputs("  ", m0fp);
        }
        for (unsigned i = 0; i < 3; i++) {
          fprintf(m0fp, "%f", outputMagFloat[i]);
          fputs("  ", m0fp);
        }
        fprintf(m0fp, "%f", timeStamp);
        fputs("  ", m0fp);
        fprintf(m0fp, "%i", deltaT);
        fputs("\n", m0fp);
    }
    else if ((m1) && (forward1 == 1)){
        for (unsigned i = 0; i < 3; i++) {
          fprintf(m1fp, "%f", outputAccFloat[i]);
          fputs("  ", m1fp);
        }
        for (unsigned i = 0; i < 3; i++) {
          fprintf(m1fp, "%f", outputMagFloat[i]);
          fputs("  ", m1fp);
        }
        fprintf(m1fp, "%f", timeStamp);
        fputs("  ", m1fp);
        fprintf(m1fp, "%i", deltaT);
        fputs("\n", m1fp);
    }
    else if ((m2) && (forward1 == 1)){
        for (unsigned i = 0; i < 3; i++) {
          fprintf(m2fp, "%f", outputAccFloat[i]);
          fputs("  ", m2fp);
        }
        for (unsigned i = 0; i < 3; i++) {
          fprintf(m2fp, "%f", outputMagFloat[i]);
          fputs("  ", m2fp);
        }

        fprintf(m2fp, "%f", timeStamp);
        fputs("  ", m2fp);
        fprintf(m2fp, "%i", deltaT);
        fputs("\n", m2fp);
    }
    //read approx. every 50 milliseconds (20 times a second)
      usleep(47000);

    } while (!done); 
    
  }

  //close all files
  fclose (m0fp);
  fclose (m1fp);
  fclose (m2fp);

  printf( "IMU Handle thread exiting\n" );
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
  usleep(1000000);
  printf("Homework 7 Program\n");
  printf("Please enter m0 to record 20 IMU data points every second\n");
  printf("Please enter m1 for manual driving mode while recording from IMU\n");
  printf("Please enter m2 for self driving, line tracing mode while recording data from IMU\n");
  printf("m1 commands: s (stop), w (forward), x (backwards), i (faster), j (slower), a (left), d (right)\n");
  printf("m2 commands: s (stop, w (forward), t (display car's traveled path)\n");
  printf("Commands for both modes: p (display IMU data), n (total distance and avg speed)\n");
  printf("\nHw7>");
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
          m0 = false;
          printf("1\n");
          printf("Mode set to m1");
          printf("\nHw7m1>");
        }
        else if (ch == '2') {
        m1 = false;
        m2 = true;
        m0 = false;
        printf("2\n");
        printf("Mode set to m2");
        printf("\nHw7m2>");
        }
        else if (ch == '0') {
        m1 = false;
        m2 = false;
        m0 = true;
        printf("0\n");
        printf("Mode set to m0");
        printf("\nHw7m0>");
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
          printf("\nHw7m1>");
        }
        if (m2) {
          printf("\nHw7m2>");
        }
        if (m0) {
          printf("\nHw7m0>");
        }
        break;
      //forward
      case 'w':

        if (m1) {
          forward1 = 1;
          rightSwitchVar =1;
          leftSwitchVar = 1;
         //printf("in threadkey %i", rightSwitchVar);
          printf("w\nHw7m1>");
        }
        if (m2) {
          forward1 = 1;
          rightSwitchVar =7;
          leftSwitchVar = 7;
          printf("w\nHw7m2>");
        }
        break;
      //backward
      case 'x':
        if (m1) {
          backward1 = 1;
          rightSwitchVar =2;
           leftSwitchVar = 2;
          printf("x\nHw7m1>");
        }
        break;
      //faster
      case 'i':
        if (m1) {
          rightSwitchVar =3;
           leftSwitchVar = 3;
          printf("i\nHw7m1>");
        }
        break;
      //slower 
      case 'j':
      if (m1) {
        rightSwitchVar = 4;
        leftSwitchVar = 4;
        printf("j\nHw7m1>");
      }
        break;
      //left
      case 'a':
      if (m1) {
        rightSwitchVar = 5;
        leftSwitchVar = 5;
        leftCount = leftCount + 1;
        printf("a\nHw7m1>");
      }
        break;
      //right 
      case 'd':
      if (m1) {
        rightSwitchVar = 6;
        leftSwitchVar = 6;
        rightCount = rightCount+1;
        printf("d\nHw7m1>");
      }
        break;
      //print the IMU readings
      case 'p':
      if ((m1) && (stop1 == 1)){
        printf("p");
        printf("\n%s", m1outputString);
        printf("\nHw7m1>");
      }
      else if ((m2) && (stop1 == 1)) {
        printf("p");
        printf("\n%s", m2outputString);
        printf("\nHw7m2>");
      }
        break;
        //print distance and avg speed
      case 'n':
      if ((m1) && (stop1 == 1)){
        printf("n");
        printf("\nAvg Speed: %f m/s", avgVelm1);
        printf("\nTotal Distance traveled: %f meters", distancem1);
        printf("\nHw7m1>");
      }
      else if ((m2) && (stop1 == 1)){
        printf("n");
        printf("\nAvg Speed: %f m/s", avgVelm2);
        printf("\nTotal Distance traveled: %f meters", distancem2);
        printf("\nHw7m2>");
      }
        break;
      //print map
      case 't':
        if ((m2) && (stop1 == 1)) {
          printf("t\n");
          for (int i=0;i<20;i++){
            for (int j=0;j<20;j++) {
              if (mapArr[i][j] != -1) {
              printf("%d", mapArr[i][j]);
            }
            else {
              printf(" ");
            }
            }
            printf("\n");
          }
          printf("\nHw7m2>");
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
  pthread_t                       imu_handle;
  struct thread_parameter         left_parameter;
  struct thread_parameter         right_parameter;
  struct thread_parameter         imu_parameter;
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
     //set IR sensors to input
     io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_INPUT;
     io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
    
    /* set the pin function to alternate function 0 for GPIO09 (SPI, MISO) */
    /* set the pin function to alternate function 0 for GPIO10 (SPI, MOSI) */
    /* set the pin function to alternate function 0 for GPIO11 (SPI, SCK) */
    /* set the pin function to output for GPIO08 (SPI, CS) */
    io->gpio.GPFSEL0.field.FSEL9 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL1.field.FSEL0 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL1.field.FSEL1 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL0.field.FSEL8 = GPFSEL_OUTPUT;

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

    /* set initial output state */
    GPIO_SET(&(io->gpio), CS_PIN);
    usleep( 100000 );

    /* set up the SPI parameters */
    io->spi.CLK.field.CDIV = ((ROUND_DIVISION(APB_CLOCK,800000))>>1)<<1; /* this number must be even, so shift the LSb into oblivion */
    io->spi.CS.field.CS       = 0;
    io->spi.CS.field.CPHA     = 1;  /* clock needs to idle high and clock in data on the rising edge */
    io->spi.CS.field.CPOL     = 1;
    io->spi.CS.field.CLEAR    = 0;
    io->spi.CS.field.CSPOL    = 0;
    io->spi.CS.field.TA       = 0;
    io->spi.CS.field.DMAEN    = 0;
    io->spi.CS.field.INTD     = 0;
    io->spi.CS.field.INTR     = 0;
    io->spi.CS.field.ADCS     = 0;
    io->spi.CS.field.REN      = 0;
    io->spi.CS.field.LEN      = 0;
    /* io->spi.CS.field.LMONO */
    /* io->spi.CS.field.TE_EN */
    /* io->spi.CS.field.DONE */
    /* io->spi.CS.field.RXD */
    /* io->spi.CS.field.TXD */
    /* io->spi.CS.field.RXR */
    /* io->spi.CS.field.RXF */
    io->spi.CS.field.CSPOL0   = 0;
    io->spi.CS.field.CSPOL1   = 0;
    io->spi.CS.field.CSPOL2   = 0;
    io->spi.CS.field.DMA_LEN  = 0;
    io->spi.CS.field.LEN_LONG = 0;
    
     //set car to stop 
    GPIO_SET(&(io->gpio), 5);
    GPIO_SET(&(io->gpio), 6);
    GPIO_SET(&(io->gpio), 22);
    GPIO_SET(&(io->gpio), 23);
    io->pwm.DAT1 = 0;
    io->pwm.DAT2 = 0;
    left_parameter.pin1 = 5;
    left_parameter.pin2 = 6;
    right_parameter.pin1 = 22;
    right_parameter.pin2 = 23;
    pthread_create( &left_handle, 0, leftHandle, (void *)&left_parameter );
    pthread_create( &right_handle, 0, rightHandle, (void *)&right_parameter );
    pthread_create( &thread_key_handle, 0, ThreadKey, (void *)&thread_key_parameter );
    pthread_create( &imu_handle, 0, imuHandle, (void *)&imu_parameter );
     //printf("got here");
    pthread_join( left_handle, 0 );
    pthread_join( right_handle, 0 );
    pthread_join( thread_key_handle, 0 );
    pthread_join( imu_handle, 0 );
    //stop the car before quititng
    GPIO_CLR(&(io->gpio), 5);
    GPIO_CLR(&(io->gpio), 6);
    GPIO_CLR(&(io->gpio), 22);
    GPIO_CLR(&(io->gpio), 23);
    io->pwm.DAT1 = 0;
    io->pwm.DAT2 = 0;
    //set all pins back to input 
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_INPUT;
    io->gpio.GPFSEL0.field.FSEL5 = GPFSEL_INPUT;
    io->gpio.GPFSEL0.field.FSEL6 = GPFSEL_INPUT;
    io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL2.field.FSEL3 = GPFSEL_INPUT;
    //set IR sensors to input
    io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_INPUT;
    io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
    //set pins for IMU chip to inputs 
    io->gpio.GPFSEL0.field.FSEL8 = GPFSEL_INPUT;
    io->gpio.GPFSEL0.field.FSEL9 = GPFSEL_INPUT;
    io->gpio.GPFSEL1.field.FSEL0 = GPFSEL_INPUT;
    io->gpio.GPFSEL1.field.FSEL1 = GPFSEL_INPUT;
    
  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}

