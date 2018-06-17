/*
 * shell.cpp
 *
 *  Created on: 11 mai 2018
 *      Author: Ross
 */

#include <MBED/Robo_motor.h>
//#include <string.h>
#include <string>
#include <stdio.h>

//	My included file and headers
#include <stddef.h>

//#define DEBUG

//#include "controller.h"
#if USE_PID
#include "pid.h"
#else
#include "controller.h"
#endif
//#include "Self_Balancing_Bot.h"
//	Shell own inclusion header
#include "shell.h"

//#define USE_NTSHELL

//cShell shell;
char cmdString[CMD_STRING_LEN];
//uint8_t inBufCount = 0;
bool cmdReady = false;
ntshell_t ntshell;

extern Serial pc;
/*
//
//	Definition of the shell commands:
//	This data structure links the command to the corrisponding action function
//
static ShellCommand_t ShellCommand[]  =
{
	{
		"get_ACK",	vSendACK
	},
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
	{
		"turn", vMotorTurn,
	},
	{
		"move", vMotorMove,
	},
	{
		NULL, NULL
		//(char *)0, (void **)0
	}
} ;
*/

#ifdef USE_NTSHELL
static int func_read(char *buf, int cnt, void *extobj)
{
	if (!pc.readable())
	{
		return 0;
	}
    for (int i = 0; i < cnt; i++)
    {
        buf[i] = pc.getc();
    }

    return cnt;
}

static int func_write(const char *buf, int cnt, void *extobj)
{
    for (int i = 0; i < cnt; i++)
    {
        pc.putc(buf[i]);
    }

    return cnt;
}

static int func_callback(const char *text, void *extobj)
{
 /*   if (ntlibc_strncmp(text, "LED", 3) == 0)
    {
        gpio = !gpio;
    }

    return 0;*/
    int execnt = 0;
    const command_table_t *p = &ShellCommand[0];
    int argc;
    char argv[NTOPT_TEXT_MAXARGS][NTOPT_TEXT_MAXLEN];
    char *argvp[NTOPT_TEXT_MAXLEN];
    //void (*func)(int argc, char **argv);

    argc = ntopt_get_count(text);
    if(NTOPT_TEXT_MAXARGS <= argc)
    {
        return -1;
    }

    for(int i = 0; i<argc; i++)
    {
        argvp[i] = ntopt_get_text(text, i, argv[i], sizeof(argv[i]));
    }
    while (p->command != NULL)
    {
        if(ntlibc_strcmp(argvp[0], p->command) ==0) //, p->command_len) == 0)
        {
            //char **foo = &argvp[0];
            p->func(argc, argvp);
            execnt++;
        }
        p++;
    }

    if(!execnt)
    {
        return -2;
    }

    return 0;
}
#endif

cShell::cShell() {
	// TODO Auto-generated constructor stub

#ifdef USE_NTSHELL
	pc.printf("Start Shell Constructor");
/*	const char *Mark = Foreground256(62) Background256(225) \
	                       "\r\n"\
	                       " _   _ _   _  ____ _     _____ ___        _     ___  ____ _____ ____  ___  \r\n" \
	                       "| \\ | | | | |/ ___| |   | ____/ _ \\      | |   / _ \\| ___|___ /|  _ \\( _ ) \r\n" \
	                       "|  \\| | | | | |   | |   |  _|| | | |_____| |  | | | |___ \\ |_ \\| |_) / _ \\ \r\n" \
	                       "| |\\  | |_| | |___| |___| |__| |_| |_____| |__| |_| |___) |__) |  _ < (_) |\r\n" \
	                       "|_| \\_|\\___/ \\____|_____|_____\\___/      |_____\\___/|____/____/|_| \\_\\___/ "
	                       ResetAll
	                       "\r\n";

	pc.printf(Mark);*/
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
    ntshell_init(&ntshell, func_read, func_write, func_callback, 0);
    ntshell_set_prompt(&ntshell, SHELL_PROMPT);
#endif
}

cShell::~cShell() {
	// TODO Auto-generated destructor stub
}

#ifdef USE_NTSHELL
void cShell::startShell(ntshell_t * shell)
{
	ntshell_execute(shell);
}
#endif

//	???
//static bool bEnd = false;


//
//	Shell Local command definition data structure
//
#ifdef USE_NTSHELL

const command_table_t LocalCommands[] =
{
		{ "info", "Get informations", vCmdInfo },
		{ "systime", "Get the system time", vCmdSystime },
		{ NULL, NULL, NULL }
};

#else
static ShellCommand_t LocalCommands[] =
{
	{
		"info", vCmdInfo
	},
	{
		"systime", vCmdSystime
	},
	{
		NULL, NULL
	}
};
#endif

#ifdef USE_NTSHELL

void shellInit()
{
	ntshell_init(&ntshell, func_read, func_write, func_callback, 0);
}

int ntopt_get_count(const char *str)
{
    int cnt = 0;
    int wc = 0;
    char *p = (char *)str;
    while (*p) {
        if (!IS_DELIM(*p)) {
            wc++;
            if (wc == 1) {
                cnt++;
            }
        } else {
            wc = 0;
        }
        p++;
    }
    return cnt;
}

char *ntopt_get_text(const char *str, const int n, char *buf, int siz)
{
    int cnt = 0;
    int wc = 0;
    char *p = (char *)str;
    while (*p) {
        if (!IS_DELIM(*p)) {
            wc++;
            if ((wc == 1)) {
                if (cnt == n) {
                    char *des = buf;
                    int cc = 0;
                    while (!IS_DELIM(*p)) {
                        cc++;
                        if (siz <= cc) {
                            break;
                        }
                        *des = *p;
                        des++;
                        p++;
                    }
                    *des = '\0';
                    return buf;
                }
                cnt++;
            }
        } else {
            wc = 0;
        }
        p++;
    }
    return '\0';
}

#else
#ifdef USE_MBED

/// <summary>
/// Shells  task code.
/// </summary>
/// <param name="p">Pointer to a data structure storing all implemented commands</param>
/// <param name="line">The commanbd line.</param>
void cShell::ShellTask(void *p, string line)
{
	//	Initialize the shell command data structure
	const ShellCommand_t *scp=((ShellConfig_t *)p)->sc_command;
	const string eol = "\n\r";
	//string cmd;
	//string foo;
	char *cmd, *foo;
	char *args[SHELL_MAX_ARGUMENTS];
	uint8_t n = 0;

	// Scan the command
	//if(sscanf(line.c_str(),"%s %s\n\r", (char *)&cmd, (char *)&foo) == 2)
	if(sscanf(line.c_str(),"%s %s\n\r", &cmd, &foo) == 2)
	{
#ifdef DEBUG
		printf("command: %s\n\r", cmd); //.c_str());
#endif
		// Scan the arguments
		//while(sscanf(foo.c_str(), "%s %s", (char *)&args[n], (char *)&foo) == 2)
		while(sscanf(foo, "%s %s", &args[n], &foo) == 2)
		{
			n++;
			if(n > SHELL_MAX_ARGUMENTS) break;
		}
		if (n > SHELL_MAX_ARGUMENTS)
		{
			printf("Too many arguments\n\r");
			return;
		}
		//args[n] = (char *)foo.c_str();
		args[n] = foo;
	}
	else if(sscanf(line.c_str(),"%s\n\r", (char *)&cmd) != 1)
	{
		//printf("Error parsing the command\n\rline -> %s\n\rcmd -> %s\n\rfoo -> %s\n\r", line.c_str(), cmd.c_str(), foo.c_str());
		printf("Error parsing the command\n\rline -> %s\n\rcmd -> %s\n\rfoo -> %s\n\r", line.c_str(), cmd, foo);
		return;
	}

	//
	//	If there is a valid command to execute (not NULL),parse it and execute the corresponding action
	//
	string null = "";
	null.clear();
	string command=cmd;
	//if (cmd.length() != 0)
	if(command.empty())
	{
		//	Exit the Shell
		if (command == "exit")
		{
			//	Exit has no arguments
			if (n > 0)
			{
				vUsage((char *)"exit");
			}
			// Set the shell end flag
			bEnd = true;
			return;
		}
		//	Display the list of supported commands
		else if (command == "help")
		{
			//	Help has no arguments
			printf("Entering help");
			if (n > 1)
			{
				vUsage((char *)"help");
			}
			printf("Commands:");
			//	Display the Local Commands
			vListCommands(LocalCommands);
			//	Display the Shell Commands
			vListCommands(ShellCommand);
			printf("\n\r");
		}
		//	Try to Execute the other command, if it exits an error the command is not recognized
		//else if (vCmdExec(LocalCommands, (char *)cmd.c_str(), n, args) && ((scp == NULL) || vCmdExec(ShellCommand, (char *)cmd.c_str(), n, args)))
		else if (vCmdExec(LocalCommands, (char *)command.c_str(), n, args) && ((scp == NULL) || vCmdExec(ShellCommand, (char *)command.c_str(), n, args)))
		{
			printf("Error: Command not recognized -> %s\n\r", command.c_str());
		}
	}
}

#else

/// <summary>
/// Shells  task code.
/// </summary>
/// <param name="p">Pointer to a data structure storing all implemented commands</param>
/// <param name="line">The commanbd line.</param>
void cShell::ShellTask(void *p, char *line)
{
	int n;
	//	Initialize the shell command data structure
	const ShellCommand_t *scp=((ShellConfig_t *)p)->sc_command;
	char *lp, *cmd, *tokp;
	char *args[SHELL_MAX_ARGUMENTS + 1];

	//	Get all the tokens from the input string and stores them on lp and tokp pointer
	//lp = vStrtok(line, " \r\n", &tokp);
	lp = vStrtok(line, " \n\r", &tokp);

	#ifdef DEBUG
	printf("lp -> %s\n\rtokp -> %s\n\r", lp, tokp);
	#endif // DEBUG

	//	The command to execute is stored into lp
	cmd = lp;
	n = 0;

	//
	//	Until there are valid tokens fill the arguments array args with them
	//
	while ((lp = vStrtok(NULL, " \n\r", &tokp)) != NULL)
	{
		//	If there are too many arguments display an error
		if (n > SHELL_MAX_ARGUMENTS)
		{
			printf("Too many arguments");
			cmd = NULL;
			break;
		}
		// else fill arguments array
		args[n++] = lp;
	}
	//	End the args array with a NULL argument
	args[n] = NULL;
	// do we really need it ????
	if (n == 0)
	{
		#ifdef DEBUG
		printf("Forcing end of string\n\r");
		#endif // DEBUG
		int len = strlen(cmd);
		cmd[len] = '\0';
	}

	#ifdef DEBUG
	printf("Cmd -> %s\n\r", cmd);
	char numArgv[2];
	numArgv[0] = '0' + n;
	numArgv[1] = '\0';
	printf("n -> %s\n\r", numArgv);
	#endif // DEBUG

	//
	//	If there is a valid command to execute (not NULL),parse it and execute the corresponding action
	//
	if (cmd != NULL)
	{
		//	Exit the Shell
		if (strcasecmp(cmd, "exit") == 0)
		{
			//	Exit has no arguments
			if (n > 0)
			{
				vUsage((char *)"exit");
			}
			// Set the shell end flag
			bEnd = true;
			return;
		}
		//	Display the list of supported commands
		else if (strcasecmp(cmd, "help") == 0)
		{
			//	Help has no arguments
			printf("Entering help");
			if (n > 1)
			{
				vUsage((char *)"help");
			}
			printf("Commands:");
			//	Display the Local Commands
			vListCommands(LocalCommands);
			//	Display the Shell Commands
			vListCommands(ShellCommand);
			printf("\n\r");
		}
		//	Try to Execute the other command, if it exits an error the command is not recognized
		else if (vCmdExec(LocalCommands, cmd, n, args) && ((scp == NULL) || vCmdExec(/*scp*/ ShellCommand, cmd, n, args)))
		{
			printf("Error: Command not recognized -> ");
			printf (cmd);
		}
	}
}

/// <summary>
/// Shell thread loop.
/// this function read from stdin and calls the real thread code
/// </summary>
/// <param name="p">A pointer to the implemented commands.</param>
void cShell::vShellThread(void *p)
{
	char line[SHELL_MAX_LINE_LENGTH];
	// chRegSetThreadName("shell");

	printf("\r\nSTM32 Shell;");

	while (!bEnd)
	{
		// Display the prompt
		printf(SHELL_PROMPT);
		// Get the command line from stdin
		gets(line);
		// Calls the shell task
		ShellTask(p, line);
	}
	return;
}

/// <summary>
/// Substring token extraction.
/// </summary>
/// <param name="strc">Input string.</param>
/// <param name="delim">String containing a list of delimiters.</param>
/// <param name="saveptr">Vector of String containing all the substrings.</param>
char * cShell::vStrtok(char *str, const char *delim, char **saveptr)
{
	char *token;
	if (str) *saveptr = str;
	token = *saveptr;

	if (!token) return NULL;

	token += strspn(token, delim);
	*saveptr = strpbrk(token, delim);

	if (*saveptr) *(*saveptr)++ = '\0';

	return *token ? token : NULL;
}

#endif

/// <summary>
/// Command Execution.
/// Execute a command and return the result.
/// </summary>
/// <param name="scp">A pointer to the implemented commands.</param>
/// <param name="name">Name of the command to execute</param>
/// <param name="argv">A pointer to the Argument list.</param>
int cShell::vCmdExec(const ShellCommand_t *scp, char *name, int argc, char *argv[])
{
	while (scp->sc_name != NULL)
	{
		if(strcmp(scp->sc_name, name) == 0)
		{
			scp->sc_function(argc, argv);
			return 0;
		}
		scp++;
	}
	return 1;
}

#endif

/// <summary>
/// System Time Command.
/// Not implemented.
/// </summary>
/// <param name="argc">Number of parameters.</param>
/// <param name="argv">A pointer to the Argument list.</param>
void vCmdSystime(int argc, char *argv[])
{
	(void) argv;
	//	If there are arguments display and error message
	if(argc > 0)
	{
		vUsage((char *)"systime");
		return;
	}
	//	Else display a string stating that it is not implemented
	printf("Sys Time: Not implemented yet\r\nOK");
}

/// <summary>
/// Info Command.
/// Display firmware information.
/// </summary>
/// <param name="argc">Number of parameters.</param>
/// <param name="argv">A pointer to the Argument list.</param>

void vCmdInfo(int argc, char *argv[])
{
	(void)argv;
	//	If there are arguments plot an error message
	if(argc > 1)
	{
		vUsage((char *)"info");
		return;
	}
	//	Else display Firmware and OS versions
	printf("Firmware: ");
	printf(FW_VERSION);
	printf("\r\nOS Version: ");
	printf(OS_VERSION);

	//Serial.println(F("\r\nOK\r\n"));
}

/// <summary>
/// List the Commands in the scp data structure.
/// </summary>
/// <param name="scp">Command data structure.</param>
#ifdef USE_NTSHELL
void vListCommands(command_table_t *p)
{
	//	Until the commands data structure has valid elements display the command name
	while (p->command != NULL)
		{
			pc.printf("%s : %s%S",(char *)p->command, p->description, CR);
			p++;
		}
}

#else
void vListCommands(ShellCommand_t *scp)
{
	//	Until the commands data structure has valid elements display the command name
	while (scp->sc_name != NULL)
	{
		printf((char *)scp->sc_name);
		scp++;
	}
}
#endif

/// <summary>
/// Command Usage Function.
/// Display information how to use the command.
/// </summary>
/// <param name="strc">Command usage string.</param>
void vUsage(char *str)
{
	printf("Error: Usage-> ");
	printf("%s\r\n", str);
}

/// <summary>
/// Send the ACK
/// </summary>
/// <param name="argc">Number of parameters.</param>
/// <param name="argv">A pointer to the Argument list.</param>
void vSendACK(int argc, char *argv[])
{
	//	If there are arguments send an error message
	if (argc > 0)
	{
		vUsage((char *)"get_ACK");
	}
	else
	{
		//	Send the ACK
		printf("ACK\r\nOK\r\n");
	}
}

//#endif // ARDUINO_AVR_LEONARDO



