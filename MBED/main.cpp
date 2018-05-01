
#if 0

#include "mbed.h"
#include "rtos.h"
#include "XNucleoIKS01A2.h"

#define SENS	70

/**
 * 	Global Variables
 */

static XNucleoIKS01A2 *mems_exp_brd = XNucleoIKS01A2::instance(D14, D15, D4, D5);
static LSM6DSLSensor * acc_gyro = mems_exp_brd->acc_gyro;
int32_t axes[3];
int32_t offs[3];
float partial[3];
float final[3];

/*
 *  Function Prototypes
 */
void init_vector(int32_t *, uint8_t);
void init_vector(float *, uint8_t);

int main(void)
{
	uint8_t Id;
	uint32_t k = 0;

	init_vector(partial, 3);
	init_vector(final, 3);

	acc_gyro->enable_x();
	acc_gyro->enable_g();
	printf("\nInitialization done!\n");
	acc_gyro->read_id(&Id);
	printf("MEMS Id: 0x%x\n", Id);
	wait (1.5);

	acc_gyro->get_g_axes(axes);
	printf("LSM6DSL [gyro/mdps]: %6ld,\t%6ld,\t%6ld\n", axes[0], axes[1], axes[2]);

	// Set the offsets
	for (int i=0; i<3; i++) offs[i] -= axes[i];
	printf("Offsets: %6ld,\t%6ld,\t%6ld\n", offs[0], offs[1], offs[2]);

	while(true)
	{
		acc_gyro->get_x_axes(axes);
		printf("Ax: %6ld,\tAy: %6ld,\tAz: %6ld\n", axes[0], axes[1], axes[2]);
		acc_gyro->get_g_axes(axes);
		for(int i=0; i<3; i++) axes[i] -= offs[i];
		printf("Corrected Gx: %6ld,\tGy: %6ld,\tGz: %6ld\n", axes[0], axes[1], axes[2]);
		k++;
		wait_ms(250);
		for(int i=0; i<3; i++)
		{
			partial[i] = axes[i] * SENS / 1000;
			partial[i] /= 1000;
			if((axes[i] > 150) || (axes[i] <-150)) final[i] += partial[i];
		}
		printf("Final AngX: %f,\tAngY: %f,\tAngZ: %f\n", final[0], final[1], final[2]);
	}
}

void init_vector(uint32_t *vect, uint8_t dim)
{
	for(int i=0; i<dim; i++) *(vect + i*4) = 0;
}

void init_vector(float * vect, uint8_t dim)
{
	for(int i=0; i<dim; i++) *(vect + i*sizeof(*vect)) = 0.0;
}
#endif

/* Includes */
#include "mbed/mbed.h"
#include "../MBED/mbed-rtos/rtos/rtos.h"
#include "../MBED/X_NUCLEO_IKS01A2/XNucleoIKS01A2.h"
#include "../MBED/ExtendedClock.h"

/* Component specific header files. */
#include "STSpin240_250.h"

#define TIMER_TICK_ms		100

/* Instantiate the expansion board */
static XNucleoIKS01A2 *mems_expansion_board = XNucleoIKS01A2::instance(D14, D15, D4, D5);

/* Retrieve the composing elements of the expansion board */
static LSM303AGRMagSensor *magnetometer = mems_expansion_board->magnetometer;
static HTS221Sensor *hum_temp = mems_expansion_board->ht_sensor;
static LPS22HBSensor *press_temp = mems_expansion_board->pt_sensor;
static LSM6DSLSensor *acc_gyro = mems_expansion_board->acc_gyro;
static LSM303AGRAccSensor *accelerometer = mems_expansion_board->accelerometer;

void baud(int baudrate)
{
	Serial pc(USBTX, USBRX);
	pc.baud(baudrate);
}


/* Helper function for printing floats & doubles */
static char *print_double(char* str, double v, int decimalDigits=2)
{
  int i = 1;
  int intPart, fractPart;
  int len;
  char *ptr;

  /* prepare decimal digits multiplicator */
  for (;decimalDigits!=0; i*=10, decimalDigits--);

  /* calculate integer & fractinal parts */
  intPart = (int)v;
  fractPart = (int)((v-(double)(int)v)*i);

  /* fill in integer part */
  sprintf(str, "%i.", intPart);

  /* prepare fill in of fractional part */
  len = strlen(str);
  ptr = &str[len];

  /* fill in leading fractional zeros */
  for (i/=10;i>1; i/=10, ptr++) {
    if (fractPart >= i) {
      break;
    }
    *ptr = '0';
  }

  /* fill in (rest of) fractional part */
  sprintf(ptr, "%i", fractPart);

  return str;
}

/**
 * Synchronization
 */
Semaphore timer(1);

/**
 * Time synchronization thread
 */
void synch_thread(void const *name)
{
	while(true)
	{
		timer.release();
		Thread::wait(TIMER_TICK_ms);
	}
}

/* Variables -----------------------------------------------------------------*/

/* Initialization parameters of the motor connected to the expansion board. */
 STSpin240_250_init_t init =
 {
  20000, /* Frequency of PWM of Input Bridge A in Hz up to 100000Hz             */
  20000, /* Frequency of PWM of Input Bridge B in Hz up to 100000Hz             */
  20000, /* Frequency of PWM used for Ref pin in Hz up to 100000Hz              */
  50,    /* Duty cycle of PWM used for Ref pin (from 0 to 100)                  */
  TRUE   /* Dual Bridge configuration  (FALSE for mono, TRUE for dual brush dc) */
 };

/* Motor Control Component. */
STSpin240_250 *motor;

/* Functions -----------------------------------------------------------------*/

/**
 * @brief  This is an example of error handler.
 * @param[in] error Number of the error
 * @retval None
 * @note   If needed, implement it, and then attach it:
 *           + motor->attach_error_handler(&my_error_handler);
 */
void my_error_handler(uint16_t error)
{
  /* Printing to the console. */
  printf("Error %d detected\r\n\n", error);

  /* Infinite loop */
  while (true) {
  }
}

/**
 * @brief  This is an example of user handler for the flag interrupt.
 * @param  None
 * @retval None
 * @note   If needed, implement it, and then attach and enable it:
 *           + motor->attach_flag_irq(&my_flag_irq_handler);
 *           + motor->enable_flag_irq();
 *         To disable it:
 *           + motor->DisbleFlagIRQ();
 */
void my_flag_irq_handler(void)
{
   /* Code to be customised */
  /************************/

  printf("    WARNING: \"FLAG\" interrupt triggered.\r\n");

  /* Get the state of bridge A */
  uint16_t bridgeState  = motor->get_bridge_status(0);

  if (bridgeState == 0)  {
    if (motor->get_device_state(0) != INACTIVE) {
      /* Bridges were disabled due to overcurrent or over temperature */
      /* When  motor was running */
      my_error_handler(0XBAD0);
    }
  }
}

/* Simple main function */
int main() {
  uint8_t id;
  float value1, value2;
  char buffer1[32], buffer2[32];
  int32_t axes[3];
  int32_t gyro[3];
  uint32_t count = 0;
  uint32_t old_timing = 0;
  Thread synThd;

  /* Enable all sensors */
  hum_temp->enable();
  press_temp->enable();
  magnetometer->enable();
  accelerometer->enable();
  acc_gyro->enable_x();
  acc_gyro->enable_g();

  baud(115200);

  // Start threads
  synThd.start(callback(synch_thread, (void *)"Synch Thread"));

  printf("\r\n--- Starting new run ---\r\n");

  hum_temp->read_id(&id);
  printf("HTS221  humidity & temperature    = 0x%X\r\n", id);
  press_temp->read_id(&id);
  printf("LPS22HB  pressure & temperature   = 0x%X\r\n", id);
  magnetometer->read_id(&id);
  printf("LSM303AGR magnetometer            = 0x%X\r\n", id);
  accelerometer->read_id(&id);
  printf("LSM303AGR accelerometer           = 0x%X\r\n", id);
  acc_gyro->read_id(&id);
  printf("LSM6DSL accelerometer & gyroscope = 0x%X\r\n", id);

  printf("count, AccX, AccY, AccZ, GyroX, GyroY, GyroZ\n");

  uint8_t demoStep = 0;

  /* Printing to the console. */
  printf("STARTING MAIN PROGRAM\r\n");

//----- Initialization

  /* Initializing Motor Control Component. */
  #if (defined TARGET_NUCLEO_F030R8)||(defined TARGET_NUCLEO_F334R8)
  motor = new STSpin240_250(D2, D9, D6, D7, D5, D4, A2);
  #elif (defined TARGET_NUCLEO_L152RE)
  motor = new STSpin240_250(D2, D9, D6, D7, D5, D4, A3);
  #elif (defined TARGET_NUCLEO_F429ZI)
  motor = new STSpin240_250(D2, D9, D6, D7, D5, D3, A0);
  #else
//  motor = new STSpin240_250(D2, D9, D6, D7, D5, D4, A0);
  motor = new STSpin240_250(PA_10, PC_7, PB_10, PA_8, PB_4, PB_5, PA_0);
  #endif
  if (motor->init(&init) != COMPONENT_OK) exit(EXIT_FAILURE);

  /* Set dual bridge enabled as two motors are used*/
  motor->set_dual_full_bridge_config(1);

  /* Attaching and enabling an interrupt handler. */
  motor->attach_flag_irq(&my_flag_irq_handler);
  motor->enable_flag_irq();

  /* Attaching an error handler */
  motor->attach_error_handler(&my_error_handler);

  /* Printing to the console. */
  printf("Motor Control Application Example for 2 brush DC motors\r\n");

  /* Set PWM Frequency of Ref to 15000 Hz */
  motor->set_ref_pwm_freq(0, 15000);

  /* Set PWM duty cycle of Ref to 60% */
  motor->set_ref_pwm_dc(0, 60);

  /* Set PWM Frequency of bridge A inputs to 10000 Hz */
  motor->set_bridge_input_pwm_freq(0,10000);

  /* Set PWM Frequency of bridge B inputs to 10000 Hz */
  motor->set_bridge_input_pwm_freq(1,10000);

  /* Infinite Loop. */
  printf("--> Infinite Loop...\r\n");
  while (true) {
    switch (demoStep) {
        case 0: {
          printf("STEP 0: Motor(0) FWD Speed=100%% - Motor(1) Inactive\r\n");
          /* Set speed of motor 0 to 100 % */
          motor->set_speed(0,100);
          /* start motor 0 to run forward*/
          /* if chip is in standby mode */
          /* it is automatically awakened */
          motor->run(0, BDCMotor::FWD);
          break;
        }
        case 1: {
          printf("STEP 1: Motor(0) FWD Speed=75%% - Motor(1) BWD Speed=100%%\r\n");
          /* Set speed of motor 0 to 75 % */
          motor->set_speed(0,75);
          /* Set speed of motor 1 to 100 % */
          motor->set_speed(1,100);
          /* start motor 1 to run backward */
          motor->run(1, BDCMotor::BWD);
          break;
        }
        case 2: {
          printf("STEP 2: Motor(0) FWD Speed=50%% - Motor(1) BWD Speed=75%%\r\n");
          /* Set speed of motor 0 to 50 % */
          motor->set_speed(0,50);
         /* Set speed of motor 1 to 75% */
          motor->set_speed(1,75);
          break;
        }
        case 3: {
          printf("STEP 3: Motor(0) FWD Speed=25%% - Motor(1) BWD Speed=50%%\r\n");
          /* Set speed of motor 0 to 25 % */
          motor->set_speed(0,25);
          /* Set speed of motor 1 to 50% */
          motor->set_speed(1,50);
          break;
        }
        case 4: {
          printf("STEP 4: Motor(0) Stopped - Motor(1) BWD Speed=25%%\r\n");
          /* Stop Motor 0 */
          motor->hard_stop(0);
          /* Set speed of motor 1 to 25% */
          motor->set_speed(1,25);
          break;
        }
        case 5: {
          printf("STEP 5: Motor(0) BWD Speed=25%% - Motor(1) Stopped\r\n");
          /* Set speed of motor 0 to 25 % */
          motor->set_speed(0,25);
          /* start motor 0 to run backward */
          motor->run(0, BDCMotor::BWD);
          /* Stop Motor 1 */
          motor->hard_stop(1);
          break;
        }
        case 6: {
          printf("STEP 6: Motor(0) BWD Speed=50%% - Motor(1) FWD Speed=25%%\r\n");
          /* Set speed of motor 0 to 50 % */
          motor->set_speed(0,50);
          /* Set speed of motor 1 to 25 % */
          motor->set_speed(1,25);
          /* start motor 1 to run backward */
          motor->run(1, BDCMotor::FWD);
          break;
        }
        case 7: {
          printf("STEP 7: Motor(0) BWD Speed=75%% - Motor(1) FWD Speed=50%%\r\n");
          /* Set speed of motor 0 to 75 % */
          motor->set_speed(0,75);
          /* Set speed of motor 1 to 50 % */
          motor->set_speed(1,50);
          break;
        }
        case 8: {
          printf("STEP 8: Motor(0) BWD Speed=100%% - Motor(1) FWD Speed=75%%\r\n");
          /* Set speed of motor 0 to 100 % */
          motor->set_speed(0,100);
          /* Set speed of motor 1 to 75 % */
          motor->set_speed(1,75);
          break;
        }
        case 9: {
          printf("STEP 9: Motor(0) BWD Speed=100%% - Motor(1) FWD Speed=100%%\r\n");
          /* Set speed of motor 1 to 100 % */
          motor->set_speed(1,100);
          break;
        }
        case 10: {
          printf("STEP 10\r\n: Stop both motors and disable bridges\r\n");
          /* Stop both motors and disable bridge */
          motor->hard_hiz(0);
          motor->hard_hiz(1);
          break;
        }
        case 11: {
          printf("STEP 11: Motor(0) FWD Speed=100%% - Motor(1) FWD Speed=100%%\r\n");
          /* Start both motors to go forward*/
          motor->run(0,BDCMotor::FWD);
          motor->run(1,BDCMotor::FWD);
          break;
        }
        case 12:
        default: {
          printf("STEP 12: Stop both motors and enter standby mode\r\n");
          /* Stop both motors and put chip in standby mode */
          motor->reset();
          break;
        }
    }

    /* Wait for 5 seconds */
    wait_ms(5000);

    /* Increment demostep*/
    demoStep++;
    if (demoStep > 12) {
      demoStep = 0;
    }
  }

  // Start the clock

  while(1) {
#if 0
    printf("\r\n");

    hum_temp->get_temperature(&value1);
    hum_temp->get_humidity(&value2);
    printf("HTS221: [temp] %7s C,   [hum] %s%%\r\n", print_double(buffer1, value1), print_double(buffer2, value2));

    press_temp->get_temperature(&value1);
    press_temp->get_pressure(&value2);
    printf("LPS22HB: [temp] %7s C, [press] %s mbar\r\n", print_double(buffer1, value1), print_double(buffer2, value2));

    printf("---\r\n");

    magnetometer->get_m_axes(axes);
    printf("LSM303AGR [mag/mgauss]:  %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);

    accelerometer->get_x_axes(axes);
    printf("LSM303AGR [acc/mg]:  %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);

    acc_gyro->get_x_axes(axes);
    printf("LSM6DSL [acc/mg]:      %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);

    acc_gyro->get_g_axes(axes);
    printf("LSM6DSL [gyro/mdps]:   %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);

    wait(1.5);
#endif

    timer.wait();
    acc_gyro->get_x_axes(axes);
    acc_gyro->get_g_axes(gyro);
    uint32_t foo = clock_ms();
    printf("start,%d,%d,%d,%d,%d,%d,%d,#\r\n", foo - old_timing, axes[0], axes[1], axes[2], gyro[0], gyro[1], gyro[2]);
    old_timing = foo;
  }
}

#if 0
/**
 ******************************************************************************
 * @file    main.cpp
 * @author  CLab
 * @version V1.0.0
 * @date    2-December-2016
 * @brief   Simple Example application for using the X_NUCLEO_IKS01A2 
 *          MEMS Inertial & Environmental Sensor Nucleo expansion board.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
*/ 

/* Includes */
#include "mbed.h"
#include "XNucleoIKS01A2.h"

/* Instantiate the expansion board */
static XNucleoIKS01A2 *mems_expansion_board = XNucleoIKS01A2::instance(D14, D15, D4, D5);

/* Retrieve the composing elements of the expansion board */
static LSM6DSLSensor *acc_gyro = mems_expansion_board->acc_gyro;

InterruptIn mybutton(USER_BUTTON);

volatile int mems_event = 0;
volatile int toggle_hw_event_enable = 0;
static int hw_event_is_enabled = 1;
uint16_t step_count = 0;

/* User button callback. */
void pressed_cb() {
  toggle_hw_event_enable = 1;
}

/* Interrupt 1 callback. */
void int1_cb() {
  mems_event = 1;
}

/* Interrupt 2 callback. */
void int2_cb() {
  mems_event = 1;
}

/* Print the orientation. */
void send_orientation() {
  uint8_t xl = 0;
  uint8_t xh = 0;
  uint8_t yl = 0;
  uint8_t yh = 0;
  uint8_t zl = 0;
  uint8_t zh = 0;
  
  acc_gyro->get_6d_orientation_xl(&xl);
  acc_gyro->get_6d_orientation_xh(&xh);
  acc_gyro->get_6d_orientation_yl(&yl);
  acc_gyro->get_6d_orientation_yh(&yh);
  acc_gyro->get_6d_orientation_zl(&zl);
  acc_gyro->get_6d_orientation_zh(&zh);
  
  if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0 ) {
    printf( "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |  *             | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |________________| \r\n" );
  }
  
  else if ( xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
    printf( "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |             *  | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |________________| \r\n" );
  }
  
  else if ( xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0 ) {
    printf( "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |  *             | " \
            "\r\n |________________| \r\n" );
  }
  
  else if ( xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
    printf( "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |             *  | " \
            "\r\n |________________| \r\n" );
  }
  
  else if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1 ) {
    printf( "\r\n  __*_____________  " \
            "\r\n |________________| \r\n" );
  }
  
  else if ( xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0 ) {
    printf( "\r\n  ________________  " \
            "\r\n |________________| " \
            "\r\n    *               \r\n" );
  }
  
  else {
    printf( "None of the 6D orientation axes is set in LSM6DSL - accelerometer.\r\n" );
  }
}

/* Simple main function */
int main() {
  /* Attach callback to User button press */
  mybutton.fall(&pressed_cb);
  /* Attach callback to LSM6DSL INT1 */
  acc_gyro->attach_int1_irq(&int1_cb);
  /* Attach callback to LSM6DSL INT2 */
  acc_gyro->attach_int2_irq(&int2_cb);
  
  /* Enable LSM6DSL accelerometer */
  acc_gyro->enable_x();
  /* Enable HW events. */
  acc_gyro->enable_pedometer();
  acc_gyro->enable_tilt_detection();
  acc_gyro->enable_free_fall_detection();
  acc_gyro->enable_single_tap_detection();
  acc_gyro->enable_double_tap_detection();
  acc_gyro->enable_6d_orientation();
  acc_gyro->enable_wake_up_detection();
  
  printf("\r\n--- Starting new run ---\r\n");
 
  while(1) {
    if (mems_event) {
      mems_event = 0;
      LSM6DSL_Event_Status_t status;
      acc_gyro->get_event_status(&status);
      if (status.StepStatus) {
        /* New step detected, so print the step counter */
        acc_gyro->get_step_counter(&step_count);
        printf("Step counter: %d\r\n", step_count);
      }

      if (status.FreeFallStatus) {
        /* Output data. */
        printf("Free Fall Detected!\r\n");
      }

      if (status.TapStatus) {
        /* Output data. */
        printf("Single Tap Detected!\r\n");
      }

      if (status.DoubleTapStatus) {
        /* Output data. */
        printf("Double Tap Detected!\r\n");
      }

      if (status.D6DOrientationStatus) {
        /* Send 6D Orientation */
        send_orientation();
      }

      if (status.TiltStatus) {
        /* Output data. */
        printf("Tilt Detected!\r\n");
      }

      if (status.WakeUpStatus) {
        /* Output data. */
        printf("Wake Up Detected!\r\n");
      }
    }

    if (toggle_hw_event_enable) {
      toggle_hw_event_enable = 0;
      if (hw_event_is_enabled == 0) {
        /* Enable HW events. */
        acc_gyro->enable_pedometer();
        acc_gyro->enable_tilt_detection();
        acc_gyro->enable_free_fall_detection();
        acc_gyro->enable_single_tap_detection();
        acc_gyro->enable_double_tap_detection();
        acc_gyro->enable_6d_orientation();
        acc_gyro->enable_wake_up_detection();
        hw_event_is_enabled = 1;
      } else {
        acc_gyro->disable_pedometer();
        acc_gyro->disable_tilt_detection();
        acc_gyro->disable_free_fall_detection();
        acc_gyro->disable_single_tap_detection();
        acc_gyro->disable_double_tap_detection();
        acc_gyro->disable_6d_orientation();
        acc_gyro->disable_wake_up_detection();
        hw_event_is_enabled = 0;
      }
    }
  }
}
#endif
