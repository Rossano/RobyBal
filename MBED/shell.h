/*
 * shell.h
 *
 *  Created on: 2 mai 2018
 *      Author: Ross
 */

#ifndef SHELL_H_
#define SHELL_H_

/*
 * 	Includes
 */
#include "mbed.h"
//#include <string>
#include "ntshell.h"
#include "ntlibc.h"
#include "ntopt.h"
#include "VT100CtrlCode.h"

/*
 * 	Defines
 */
#define CMD_STRING_LEN		(32)
//#define NULL					0
#define SHELL_MAX_LINE_LENGTH	16
#define SHELL_MAX_ARGUMENTS		4
#define SHELL_PROMPT			"ST32> "
#define FW_VERSION				"0.9.0.0"
#define OS_VERSION				"N/A"
#define CR						"\r\n"
#undef	USE_PID
#define USE_NTSHELL

//#define USE_MBED

//	Function prototype for the action executed by the shell
typedef void (* shellcmd_t)(int argc, char *argv[]);

//
//	Shell Command Data type:
//	Struct composed by the name of the command and a function prototype for the action to execute
//
#ifdef USE_NTSHELL

#define CPUID_ADDR (0xe000ed00)
typedef struct
{
  uint32_t Revision:4;
  uint32_t Partno:12;
  uint32_t Constant:4;
  uint32_t Variant:4;
  uint32_t Implementer:8;
} CPUID;

typedef struct
{
    char *command;
    char *description;
    void (*func)(int argc, char *argv[]);
} command_table_t;

#else

typedef struct
{
	const char *sc_name;
	shellcmd_t  sc_function;
} ShellCommand_t;

//
//	Shell Configuration data type (don't think it is used)
//
typedef struct
{
	const ShellCommand_t *sc_command;
} ShellConfig_t;

#endif

#ifdef USE_NTSHELL

//
//	Shell Class
//
class cShell {
private:
	//char * vStrtok(char *str, const char *delim, char **saveptr);
	//	Function prototype for the action executed by the shell
	//	Function to execute the command
	//int vCmdExec(const ShellCommand_t *scp, char *name, int argc, char *argv[]);

public:
	cShell();
	virtual ~cShell();
	//
	//	Shell main task
	//
#ifdef USE_MBED
//	void ShellTask(void *p, string line);
#else
//	void ShellTask(void *p, char *line);
//	void vShellThread (void *p);
	void startShell(ntshell_t * shell);
#endif
//	void avrPrintf(const char * str);
};

//extern ntshell_t ntshell;

void vListCommands(command_table_t *scp);
/*static int func_read(char *buf, int cnt, void *extobj);
static int func_write(const char *buf, int cnt, void *extobj);
static int func_callback(const char *text, void *extobj);*/

#else

//	Configuration data structure
//static ShellConfig_t ShellConfig;

//
//	Shell Class
//
class cShell {
private:
	char * vStrtok(char *str, const char *delim, char **saveptr);
	//	Function prototype for the action executed by the shell
//	typedef void (* shellcmd_t)(int argc, char *argv[]);
	//	Function to execute the command
	int vCmdExec(const ShellCommand_t *scp, char *name, int argc, char *argv[]);

public:
	cShell();
	virtual ~cShell();
	//
	//	Shell main task
	//
#ifdef USE_MBED
	void ShellTask(void *p, string line);
#else
	void ShellTask(void *p, char *line);
	void vShellThread (void *p);
#endif
//	void avrPrintf(const char * str);
};

void vListCommands(ShellCommand_t *scp);

#endif

/*
 * 	Variables
 */
extern char cmdString[CMD_STRING_LEN];
extern uint8_t inBufCount;// = 0;
extern bool cmdReady;

//extern cShell shell;
//cShell shell;

/*
//
//	Actions to be executed by the shell defined in different files
//
void vpidToggle(int argc, char *argv[]);			//	Toggle ON/OFF the PID
void vpidSet(int argc, char *argv[]);				//	Set PID coefficients
void vpidGet(int argc, char *argv[]);				//	Get PID coefficients
void vpidGetError(int argc, char *argv[]);			//	Get the error out of the PID
*/
//
//	Local Actions of the Shell
//
void vSendACK(int argc, char *argv[]);					//	Send back a ACK to the PC
void vCmdInfo(int argc, char *argv[]);					//	Get back the info about the FW revision
void vCmdSystime(int argc, char *argv[]);				//	Command system time (not implemented)
void vUsage(char *str);

//
// Actions to be executed by the shell defined in different files
//
#ifdef USE_PID
extern void vpidToggle(int argc, char *argv[]); 		// Toggle the PID ON & OFF
extern void vpidSet(int argc, char *argv[]); 			// Set the PID coeff
extern void vpidGet(int argc, char *argv[]); 			// Get the PID coeff
extern void vpidGetError(int argc, char *argv[]); 		// Get the PID error
#else
/*extern void vControllerToggle(int argc, char *argv[]);	// Toggle the controller
extern void vControllerSet(int argc, char *argv[]);		// Set the feedback vector
extern void vControllerGet(int argc, char *argv[]);		// Get the controller feedback vector
extern void vControllerState(int argc, char *argv[]);	// Get the state vector*/
void vControllerToggle(int argc, char *argv[]);	// Toggle the controller
void vControllerSet(int argc, char *argv[]);		// Set the feedback vector
void vControllerGet(int argc, char *argv[]);		// Get the controller feedback vector
void vControllerState(int argc, char *argv[]);	// Get the state vector
#endif
extern void vMotorTurn(int argc, char *argv[]); 		// Turn the Bot
extern void vMotorMove(int argc, char *argv[]); 		// Move the Bot
extern void vGetValues(int argc, char *argv[]); 		// Get the IMU and feedback values
extern void vMotorTurn(int argc, char *argv[]); 		// Turn the Bot
extern void vMotorMove(int argc, char *argv[]); 		// Move the Bot
extern void vGetValues(int argc, char *argv[]); 		// Get the IMU and feedback values
void toggle_led(int argc, char **argv);

//
//	Definition of the shell commands:
//	This data structure links the command to the corrisponding action function
//
#ifdef USE_NTSHELL

void shellInit();

/*
void vSendACK(int argc, char **argv)
{
	vSendACK(argc, argv);
}

void vCmdInfo(int argc, char **argv)
{
	vCmdInfo(argc, argv);
}

void vCmdSystime(int argc, char **argv)
{
	vCmdSystime(argc, argv);
}

void vControllerToggle(int argc, char **argv)
{
	vControllerToggle(argc, argv);
}

void vControllerSet(int argc, char **argv)
{
	vControllerSet(argc, argv);
}

void vControllerGet(int argc, char **argv)
{
	vControllerGet(argc, argv);
}

void vControllerState(int argc, char **argv)
{
	vControllerState(argc, argv);
}

void vMotorTurn(int argc, char **argv)
{
	vMotorTurn(argc, argv);
}

void vMotorMove(int argc, char **argv)
{
	vMotorMove(argc, argv);
}

void vGetValues(int argc, char **argv)
{
	vGetValues(argc, argv);
}

//#endif
*/
const command_table_t ShellCommand[]  =
{
	{"help", "List of command", vCmdInfo},
	{"info", "System Info", vCmdSystime},
	{ "get_ACK", "Get Acknoledge", vSendACK },
#ifdef USE_PID
	{ "pid", vpidToggle	},
	{ "pid_set", vpidSet },
	{ "pid_get", vpidGet },
	{ "pid_error", vpidGetError	},
#else
	{ "cont_toggle", "Toggle Controller ON/OFF", vControllerToggle },
	{ "contr_set", "Set Controller ON", vControllerSet },
	{ "contr_get", "Set Controller OFF", vControllerGet },
	{ "contr_state", "Get controller State", vControllerState },
#endif
	{ "turn", "Turn Robot", vMotorTurn },
	{ "move", "Move Robot", vMotorMove },
	{ "get_val", "Get Robot values (ping)", vGetValues },
	{ "LED", "Toggle LED", toggle_led},
	{ NULL, NULL, NULL }
};

#define IS_DELIM(c) \
    (((c) == '\r') || ((c) == '\n') || ((c) == '\t') || ((c) == '\0') || ((c) == ' '))

int ntopt_get_count(const char *str);
char *ntopt_get_text(const char *str, const int n, char *buf, int siz);

extern ntshell_t ntshell;

#else
static ShellCommand_t ShellCommand[]  =
{
	{
		"get_ACK",	vSendACK
	},
#ifdef USE_PID
	{
		"pid", vpidToggle
	},
	{
		"pid_set", vpidSet
	},
	{
		"pid_get", vpidGet
	},
	{
		"pid_error", vpidGetError
	},
#else
	{
		"cont_toggle", vControllerToggle, // vControllerToggle,
	},
	{
		"contr_set", vControllerSet,
	},
	{
		"contr_get", vControllerGet,
	},
	{
		"contr_state", vControllerState,
	},
#endif
	{
		"turn", vMotorTurn,
	},
	{
		"move", vMotorMove,
	},
	{
		"get_val", vGetValues,
	},
	{
		NULL, NULL
		//(char *)0, (void **)0
	}
} ;
#endif

#endif /* SHELL_H_ */
