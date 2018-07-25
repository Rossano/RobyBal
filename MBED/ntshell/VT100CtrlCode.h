/*
 * VT100CtrlCode.h
 *
 *  Created on: 2015/3/27
 *      Author: ChiaYen
 */

#ifndef VT100CTRLCODE_H_
#define VT100CTRLCODE_H_

// 16 Color
// Foreground
#define ForegroundDefault         "\e[39m"
#define ForegroundBlack           "\e[30m"
#define ForegroundRed             "\e[31m"
#define ForegroundGreen           "\e[32m"
#define ForegroundYellow          "\e[33m"
#define ForegroundBlue            "\e[34m"
#define ForegroundMagenta         "\e[35m"
#define ForegroundCyan            "\e[36m"
#define ForegroundLightGray       "\e[37m"
#define ForegroundDarkGray        "\e[90m"
#define ForegroundLightRed        "\e[91m"
#define ForegroundLightGreen      "\e[92m"
#define ForegroundLightYellow     "\e[93m"
#define ForegroundLightBlue       "\e[94m"
#define ForegroundLightMagenta    "\e[95m"
#define ForegroundLightCyan       "\e[96m"
#define ForegroundWhite           "\e[97m"
// Background
#define BackgroundDefault         "\e[49m"
#define BackgroundBlack           "\e[40m"
#define BackgroundRed             "\e[41m"
#define BackgroundGreen           "\e[42m"
#define BackgroundYellow          "\e[43m"
#define BackgroundBlue            "\e[44m"
#define BackgroundMagenta         "\e[45m"
#define BackgroundCyan            "\e[46m"
#define BackgroundLightGray       "\e[47m"
#define BackgroundDarkGray        "\e[100m"
#define BackgroundLightRed        "\e[101m"
#define BackgroundLightGreen      "\e[102m"
#define BackgroundLightYellow     "\e[103m"
#define BackgroundLightBlue       "\e[104m"
#define BackgroundLightMagenta    "\e[105m"
#define BackgroundLightCyan       "\e[106m"
#define BackgroundWhite           "\e[107m"

// Reset
#define ResetAll                  "\e[0m"

// 256 Color
// Foreground
#define Foreground256(color)     "\e[38;5;"#color"m"
// Background
#define Background256(color)     "\e[48;5;"#color"m"

#endif /* VT100CTRLCODE_H_ */

