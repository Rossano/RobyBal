/*
 *  Includes
*/
//#include "mbed/mbed.h"
#include "mbed.h"
#include "rtos.h"
#include <string>
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

using namespace std;

/*
 * 	Defines
*/
#define TIMER_TICK_ms		1000
#define BAUDRATE			(115200)
//#define CMD_STRING_LEN	32
#define DECIMATION		100
//#define AXE_TO_USE		2
#define GYRO_SCALING	16.4	//131.0
#undef STAND_ALONE
#undef DEBUG

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
uint32_t old_timing = 0;
string inputStr;
uint8_t inBufCount;
double estimated_roll = 0;
double gyro_bias = 0;
char ch;

/* SOL LED */
DigitalOut led1(LED1);
/*	Sampling Timer */
Ticker timerUpdate;
//Timer timerUpdate;
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
void CDC_Task();


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
	pc.printf("Start");
//  uint8_t id;
//  float value1, value2;
//  char buffer1[32], buffer2[32];
//  int32_t axes[3];
//  int32_t gyro[3];
//  uint32_t count = 0;
//  Thread synThd;

  Thread stabilization_thread;
  Thread user_thread;

  /* Set baudrate	*/
  baud(BAUDRATE);

  initializeRobot();

  pc.printf("End initialization\n\n");

  // Start threads
//  synThd.start(callback(synch_thread, (void *)"Synch Thread"));
  stabilization_thread.start(callback(stabilizationThd)); //, (void *)"Stabilization Thread"));
//  user_thread.start(callback(userThd));
  //Thread.start(callback(userThd));

  pc.printf("\n\r--- Starting new run ---\n\r");

  //uint8_t demoStep = 0;

  /* Printing to the console. */
  pc.printf("STARTING MAIN PROGRAM\n\r");

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

//  Thread::wait(100);

//  userThd();

  #ifdef USE_NTSHELL
  const char *Mark = Foreground256(62) Background256(225) \
                     "\r\n"\
                     " _   _ _   _  ____ _     _____ ___        _     ___  ____ _____ ____  ___  \r\n" \
                     "| \\ | | | | |/ ___| |   | ____/ _ \\      | |   / _ \\| ___|___ /|  _ \\( _ ) \r\n" \
                     "|  \\| | | | | |   | |   |  _|| | | |_____| |  | | | |___ \\ |_ \\| |_) / _ \\ \r\n" \
                     "| |\\  | |_| | |___| |___| |__| |_| |_____| |__| |_| |___) |__) |  _ < (_) |\r\n" \
                     "|_| \\_|\\___/ \\____|_____|_____\\___/      |_____\\___/|____/____/|_| \\_\\___/ "
                     ResetAll
                     "\r\n";

  pc.printf(Mark);
  pc.printf("\r\n");

  uint32_t regValue = *((uint32_t *)CPUID_ADDR);
  volatile CPUID cpuid = *((CPUID *)&regValue);
  pc.printf("CPU Information\r\n");
  pc.printf("Implementer : 0x%02x\r\n", cpuid.Implementer);
  pc.printf("Variant     : 0x%1x\r\n", cpuid.Variant);
  pc.printf("Constant    : 0x%1x\r\n", cpuid.Constant);
  pc.printf("Partno      : 0x%3x\r\n", cpuid.Partno);
  pc.printf("Revision    : r%dp%d\r\n", (cpuid.Revision & 0x3 >> 2), (cpuid.Revision & 0x3));
  pc.printf("SysClock    : %d\r\n", SystemCoreClock);
  pc.printf("\r\n");

  pc.printf("Type <" Foreground256(193) Background256(1) "LED" ResetAll "> to toggle led");
  pc.printf("\r\n");
  //ntshell_init(&ntshell, func_read, func_write, func_callback, 0);
  shellInit();
  ntshell_set_prompt(&ntshell, SHELL_PROMPT);
  ntshell_execute(&ntshell);
#endif

  pc.printf("Start shell\r\n");
//  shell.startShell(&ntshell);
  //ntshell_execute(&ntshell);

  /* Infinite Loop. */
  pc.printf("--> Infinite Loop...\n\r");

  while(1) {
#if 0
    timer.wait();
    acc_gyro->get_x_axes(axes);
    acc_gyro->get_g_axes(gyro);
    uint32_t foo = clock_ms();
    printf("start,%d,%d,%d,%d,%d,%d,%d,#\r\n", foo - old_timing, axes[0], axes[1], axes[2], gyro[0], gyro[1], gyro[2]);
    old_timing = foo;
#endif
    //printf("Should never come here !!!\n");
 //   pc.printf("IDLE: %ld\n\r",(long)idleCount++);
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
  pc.printf("HTS221  humidity & temperature    = 0x%X\r\n", id);
  press_temp->read_id(&id);
  pc.printf("LPS22HB  pressure & temperature   = 0x%X\r\n", id);
  magnetometer->read_id(&id);
  pc.printf("LSM303AGR magnetometer            = 0x%X\r\n", id);
  accelerometer->read_id(&id);
  pc.printf("LSM303AGR accelerometer           = 0x%X\r\n", id);
  acc_gyro->read_id(&id);
  pc.printf("LSM6DSL accelerometer & gyroscope = 0x%X\r\n", id);

  //printf("count, AccX, AccY, AccZ, GyroX, GyroY, GyroZ\n");

  /*
   * Motor Config
   */
  motor = new Stspin240(MOTOR_EN, MOTOR_REF, MOTOR_RST, MOTOR_PHA, MOTOR_PWMA, MOTOR_PHB, MOTOR_PWMB, MOTOR_FREQ);
  motor->enableRef();
  motor->releaseReset();
  motor->enableBridge();
  pwmA = pwmB = 0;

  pc.printf("Motor Initilized\n\r");
  /*
   * 	KLF Configuration
   */
  kalman = new _kalman();

  /*
   * 	Controller Configuration
   */
  controller = new _controller(K1_DEFAULT, K2_DEFAULT, K3_DEFAULT, K4_DEFAULT);
  pc.printf("Controller Initilized\n\r");
  /*
   * Timer Config
   */
  timerUpdate.attach(&timerHandler, 1);//0.01);//TIMER_TICK_ms/1000);
  /*for(int i=0; i<10; i++)
  {
	  led1 = !led1;
	  //Thread::wait(250);
	  wait(250);
  }*/
  pc.printf("Timer Initilized\n\r");
  cmdString[0] = 0;
  led1 = 0;
}

void stabilizationThd()
{
	double *X;//[2];
	/*	Thread infinite loop */
	while (true)
	{
//		pc.printf("Awaiting semaphone\n\r");
		semTimer.wait();
//		pc.printf("Semaphore released\n\r");
		acc_gyro->get_x_axes(axes);
		acc_gyro->get_g_axes(gyro);
		uint32_t foo = clock_ms();
		double roll = axes[1]/axes[2];
		double rolldot = gyro[1] / 70;
//		kalman->stateUpdate(foo, roll, rolldot);
		kalman->kalmanUpdate(foo/1000, roll, rolldot);
		X = kalman->getKalmanState();
		controller->set_state(X[0], rolldot, 0, 0);
		double F = controller->calculate();
		pwmA = F / Fmax + fMotorA_Move + fMotorA_Offset;
		pwmB = F / Fmax + fMotorB_Move + fMotorB_Offset;
		estimated_roll = X[0];
		gyro_bias = X[1];
		if (bEnableStateControl) {

				/*switch (eMotorMove) {
						case FORWARD: pwm += MOVE_OFFSET;
										break;
						case BACKWARD: pwm = -MOVE_OFFSET + pwm;
										break;
						default: break;
					}; */
					/*
					switch (eMotorTurn) {
						case LEFT: motor.move_A(pwm + TURN_OFFSET);
									motor.move_B(pwm - TURN_OFFSET);
									break;
						case RIGHT: motor.move_A(pwm - TURN_OFFSET);
									motor.move_B(pwm + TURN_OFFSET);
									break;
						default: motor.move_A(pwm);
							motor.move_B(pwm);
							break;
					};

					motor.move_A(pwm);
					motor.move_B(pwm);
					*/
			motor->updateSpeed(pwmA, pwmB);
		#ifdef STAND_ALONE
					Serial.print(F("(PWM, F) = "));
					Serial.print(pwm);
					Serial.print(F(":"));
					Serial.print(F);
		#endif
					//motor.move_A(comp);
					//motor.move_B(comp);
		}
	}
}

void userThd()
{
#ifdef USE_NTSHELL
	ntshell_execute(&ntshell);

	while (true) ;
#else
//	char ch;
	pc.printf("%s ", (char *)SHELL_PROMPT);
	/*	Infinite loop	*/
	while (true)
	{
		/*if(pc.readable() > 0)
		{
			pc.scanf("%c", &ch);
			if(ch == '\r' || ch == '\n' || ch == '\0')
			{
				// End of string, send the command
			}
		}*/
//		pc.printf("User Thread\n\r");
		CDC_Task();
		if(cmdReady)
		{
//			pc.printf("cmdReady true\n\r");
			char buffer[CMD_STRING_LEN];
			char *buf = (char *)&buffer;
			//cmdString.toCharArray(buf, CMD_STRING_LEN);
	/*		for(int i=0; i<CMD_STRING_LEN-1; i++)
			{
				if(cmdString[i] == 0)
				{
					buf[i] = 0;
					break;
				}
				buf[i] = cmdString[i];
			}*/
//			pc.printf("Forse è qui\n\r");
			//buf = (char *)inputStr.data();
			//inputStr.clear();
//			pc.printf("Shell task go\n\r");
			//shell.ShellTask((void *)ShellCommand, buf);
#ifdef DEBUG
			pc.printf("input string: %s\n\r", inputStr.c_str());
#endif
			shell.ShellTask((void *)ShellCommand, (char *)inputStr.c_str());
//			inputStr.clear();
	//#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
	//		Console.print(SHELL_PROMPT);
	//#endif
	//#else
	//		Serial.print(SHELL_PROMPT);
	//#endif
			printf("%s ", (char *)SHELL_PROMPT);
			//cmdString = "";
			inputStr.clear();
			cmdReady = false;
			inBufCount = 0;
		}
		if(++count == DECIMATION)
		{
	//		Serial.print(millis());
	//		Serial.print(ypr[1] - 3.1415 / 2);
	//		Serial.print(pid.getError());
	//		Serial.println(compensation);

	//		char **foo;
	//		vGetValues(0, foo);
			count = 0;
//			pc.printf("End Decimation/Parsing\n\r");
		}
	}
#endif
}

void timerHandler()
{
	//pc.printf("Tick\n\r");
	semTimer.release();
	//pc.printf("Tack\n\r");
}


//
//	USB-CDC Task
//
void CDC_Task()
{
	char ch;
	//
	//	Until data are available from the Serial Port read the data and store it into the input buffer cmdString
	//	Process ends if '\n' is received or the MAX input string length is reached
	//
	//while(Serial.available())
//	pc.printf("CDC task\r\n");
	//ch = pc.getc();
#if 0
	if(pc.readable())
	{
		ch = pc.getc();
		if (ch == '\n') cmdReady = true;
	}
#endif
#if 1
	while(pc.readable())
	{
//		pc.printf("Got one char\r\n");
		//	If '\n" is received or MAX string lenght is reached set the cmdReady flag
		if (++inBufCount == CMD_STRING_LEN) ch = '\n';
		else {
			//ch = (char)Serial.read();
			ch = pc.getc();
			//scanf("%c", &ch);
			inputStr += ch;
//			printf("->%c", ch);
		}
//		pc.printf("%s\n\r", inputStr);
		if (ch == '\n' || ch == '\r') cmdReady = true;
	}
#endif
//	pc.printf("... leaving CDC_Task\r\n");
}

#if 0
///
///	Arduino initialization routine
///
void initialize_robot(void)
{
	uint8_t count = 10;
	uint8_t devStatus;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	TWBR = 24;							// 400KHz I2C
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	FastWire::Setup(400, true);
#else
#error "I2C implementation not found"
#endif
	//Wire.begin();
//	Wire.begin();
//	TWBR = 24;			//	400KHz I2C clock (200KHz if CPU is 8MHz)

	/*pinMode(IRQ_PORT, INPUT);
	digitalWrite(IRQ_PORT, HIGH);
	attachInterrupt(DMP_IRQ, imu_isr, RISING); */

	imu_init();
	dmpReady = true;

	pinMode(SOL_LED, OUTPUT);

	pinMode(IRQ_PORT, INPUT);
	digitalWrite(IRQ_PORT, HIGH);
	attachInterrupt(DMP_IRQ, imu_isr, CHANGE);
#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
	//Console.print(F("IRQ on pin: D"));
	//Console.println(IRQ_PORT);
#else
	Serial.print(F("IRQ on pin: D"));
	Serial.println(IRQ_PORT);
#endif

	for (int i = 0; i < 4; i++) {
#ifdef USE_STATE_VECTOR
		vState.set_element(0.0, i);
#else
		vState[i] = 0.0;
#endif
	controller.set_feedback_vector(K1_DEFAULT, K2_DEFAULT, K3_DEFAULT, K4_DEFAULT);
	}
}
#endif

void vGetValues(int argc, char *argv[]) { 		// Get the IMU and feedback values
	//
	//	Print state variables
	//
	uint32_t foo = clock_ms();
//	printf("MPU: %d %d %d %d %d %d %d #\r\n", foo - old_timing, axes[0], axes[1], axes[2], gyro[0], gyro[1], gyro[2]);
	//
	// Using double
	//
//	pc.printf("MPU: %ld %3.4f %3.4f %3.4f %3.4f %3.4f #\n\r", (long)(foo - old_timing), estimated_roll, gyro_bias, F, pwmA, pwmB);
	//
	// Using long
	//
	int eRoll = trunc(1000*estimated_roll);
	int gyroBias = trunc(1000*gyro_bias);
	int f = trunc(1000*F);
	int PWM_A = trunc(1000*pwmA);
	int PWM_B = trunc(1000*pwmB);
//	pc.printf("MPU:%d,%d,%d,%d,%d,%d,#\n\r", (long)(foo - old_timing), 1000*estimated_roll, 1000*gyro_bias, 1000*F, 1000*pwmA, 1000*pwmB);
	pc.printf("MPU:%d,%d,%d,%d,%d,%d,#\n\r", (foo - old_timing), eRoll, gyroBias, f, PWM_A, PWM_B);
	old_timing = foo;
// Arduino
#if 0
	Serial.print(F("MPU:"));
	Serial.print(millis());
	Serial.print(F(" "));
#ifdef DEBUG
	Serial.print(ypr[0]/3.1415*180);
	Serial.print(F(" "));
	Serial.print(ypr[1]/3.1415*180);
	Serial.print(F(" "));
	Serial.print(ypr[2]/3.1415*180);
	Serial.print(F(" "));
	//Serial.print(pid.getError());
#else
	//Serial.print(ypr[AXE_TO_USE]/3.1415*180);
	Serial.print(ypr[0]/3.1415*180);
	Serial.print(F(" "));
	Serial.print(ypr[1]/3.1415*180);
	Serial.print(F(" "));
	Serial.print(ypr[2]/3.1415*180);
	Serial.print(F(" "));
#endif
	Serial.print(gyro[AXE_TO_USE]);
	Serial.print(F(" "));
	Serial.print(F);
	Serial.print(F(" "));
	Serial.println(pwm);
#endif
}
