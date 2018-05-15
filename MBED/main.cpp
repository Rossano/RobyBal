/*
 *  Includes
*/
//#include "mbed/mbed.h"
#include "mbed.h"
#include "rtos.h"
//#include "../MBED/mbed-rtos/rtos/rtos.h"
#include "../MBED/X_NUCLEO_IKS01A2/XNucleoIKS01A2.h"
//#include "../MBED/ExtendedClock.h"
#include "ExtendedClock.h"

/* Component specific header files. */
#include "Stspin240.h"
#include "Robo_motor.h"
#include "controller.h"
#include "shell.h"
#include "kalman.h"
#include "board.h"

/*
 * 	Defines
*/
#define TIMER_TICK_ms		100
#define BAUDRATE			(115200)
/*
 * Variables
 */
uint32_t idleCount = 0;
uint8_t count = 0;
double pwm = 0.0;
uint8_t id;
int32_t axes[3];
int32_t gyro[3];
double pwmA, pwmB;
double Fmax = 10;

/* SOL LED */
DigitalOut led1(LED1);
/*	Sampling Timer */
Ticker timerUpdate;
/*	Synchronizing Semaphore	*/
Semaphore semTimer(1);

Serial pc(USBTX, USBRX);

/* Instantiate the expansion board */
static XNucleoIKS01A2 *mems_expansion_board = XNucleoIKS01A2::instance(D14, D15, D4, D5);

/* Retrieve the composing elements of the expansion board */
static LSM303AGRMagSensor *magnetometer = mems_expansion_board->magnetometer;
static HTS221Sensor *hum_temp = mems_expansion_board->ht_sensor;
static LPS22HBSensor *press_temp = mems_expansion_board->pt_sensor;
static LSM6DSLSensor *acc_gyro = mems_expansion_board->acc_gyro;
static LSM303AGRAccSensor *accelerometer = mems_expansion_board->accelerometer;

/*	Instantiate the Motor object	*/
//Stspin240 *motor;

Thread stabilization_thread;
Thread user_thread;

/*
 * Function Prototypes
 */
void baud(int baudrate);
//static char *print_double(char *str, double v, int decimalDigits=2);
void initializeRobot();
void stabilizationThd();
void userThd();
void timerHandler();

#if 0
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
#endif

/* Simple main function */
int main() {
//  uint8_t id;
  float value1, value2;
//  char buffer1[32], buffer2[32];
//  int32_t axes[3];
//  int32_t gyro[3];
//  uint32_t count = 0;
  uint32_t old_timing = 0;
//  Thread synThd;

  Thread stabilization_thread;
  Thread user_thread;

  /* Set baudrate	*/
  baud(BAUDRATE);

  // Start threads
//  synThd.start(callback(synch_thread, (void *)"Synch Thread"));
  stabilization_thread.start(callback(stabilizationThd)); //, (void *)"Stabilization Thread"));
  user_thread.start(callback(userThd));
  //Thread.start(callback(userThd));

  printf("\r\n--- Starting new run ---\r\n");

  uint8_t demoStep = 0;

  /* Printing to the console. */
  printf("STARTING MAIN PROGRAM\r\n");

#if 0
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
#endif

  /* Infinite Loop. */
  printf("--> Infinite Loop...\r\n");

  while(1) {
#if 0
    timer.wait();
    acc_gyro->get_x_axes(axes);
    acc_gyro->get_g_axes(gyro);
    uint32_t foo = clock_ms();
    printf("start,%d,%d,%d,%d,%d,%d,%d,#\r\n", foo - old_timing, axes[0], axes[1], axes[2], gyro[0], gyro[1], gyro[2]);
    old_timing = foo;
#endif
    printf("Should never come here !!!\n");
  }
}

void baud(int baudrate)
{
	//Serial pc(USBTX, USBRX);
	pc.baud(baudrate);
}


#if 0
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
#endif

void initializeRobot()
{
  /* Enable all sensors */
  hum_temp->enable();
  press_temp->enable();
  magnetometer->enable();
  accelerometer->enable();
  acc_gyro->enable_x();
  acc_gyro->enable_g();

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

  /*
   * Motor Config
   */
  motor = new Stspin240(MOTOR_EN, MOTOR_REF, MOTOR_RST, MOTOR_PHA, MOTOR_PWMA, MOTOR_PHB, MOTOR_PWMB, MOTOR_FREQ);
  motor->enableRef();
  motor->releaseReset();
  motor->enableBridge();
  pwmA = pwmB = 0;

  /*
   * 	KLF Configuration
   */
  kalman = new _kalman();

  /*
   * 	Controller Configuration
   */
  controller = new _controller(K1_DEFAULT, K2_DEFAULT, K3_DEFAULT, K4_DEFAULT);
  /*
   * Timer Config
   */
  timerUpdate.attach(&timerHandler, TIMER_TICK_ms/1000);
  for(int i=0; i<10; i++)
  {
	  led1 = !led1;
	  Thread::wait(250);
  }

  cmdString[0] = 0;
  led1 = 0;
}

void stabilizationThd()
{
	double X[2];
	/*	Thread infinite loop */
	while (true)
	{
		semTimer.wait();
		acc_gyro->get_x_axes(axes);
		acc_gyro->get_g_axes(gyro);
		uint32_t foo = clock_ms();
		double roll = axes[1]/axes[2];
		double rolldot = gyro[1] / 70;
//		kalman->stateUpdate(foo, roll, rolldot);
		kalman->update(foo/1000, roll, rolldot);
		controller->set_state(roll, rolldot, 0, 0);
		double F = controller->calculate();
		pwmA = F / Fmax + fMotorA_Move + fMotorA_Offset;
		pwmB = F / Fmax + fMotorB_Move + fMotorB_Offset;
	}
}

void userThd()
{
	char ch;
	/*	Infinite loop	*/
	while (true)
	{
		if(pc.readable() > 0)
		{
			pc.scanf("%c", &ch);
			if(ch == '\r' || ch == '\n' || ch == '\0')
			{
				// End of string, send the command
			}
		}
	}
}

void timerHandler()
{
	semTimer.release();
}
