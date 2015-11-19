/*================================================================================*
 * O     O          __             ______  __   __  ____     __  ___          __  *
 *  \   /      /\  / /_      _    / /___/ / /  / / / __ \   / / /   \    /\  / /  *
 *   [+]      /  \/ / \\    //   / /____ / /  / /  \ \_    / / | | | |  /  \/ /   *
 *  /   \    / /\  /   \\__//   / /----// /__/ /  \ \__ \ / /  | | | | / /\  /    *
 * O     O  /_/  \/     \__/   /_/      \_ ___/    \___ //_/    \___/ /_/  \/     *
 *                                                                                *
 *                                                                                *
 * Nuvoton Sensor Fusion Application Firmware for Cortex M4 Series                *
 *                                                                                *
 * Written by by T.L. Shen for Nuvoton Technology.                                *
 * tlshen@nuvoton.com/tzulan611126@gmail.com                                      *
 *                                                                                *
 *================================================================================*
 */
#ifndef _TIMER_CTRL_H
#define _TIMER_CTRL_H
#include <stdint.h>
#define configTICK_RATE_HZ    1000 
#define ChronMain 0
#define ChronMAG  1 
#define ChronRC   2 
#define ChronTemp 3 
#define NumCron   4
//Milliseconds to OS Ticks
#define M2T(X) ((unsigned int)(X*(configTICK_RATE_HZ/1000.0)))
void ChronographSet(char Chron);
void ChronographStart(char Chron);
int32_t ChronographRead(char Chron);
int getTickCount(void);
void SetTickSSVRC(uint8_t tick);
void DelayLoop(unsigned short delay);
void DelayUsec(unsigned int usec);
void DelayMsec(unsigned short msec);
void setup_system_tick(uint32_t sampleRate);
void IncFrameCount(int inc);
uint32_t GetFrameCount(void);
uint32_t micros(void);
uint32_t millis(void);
void TIMER_Init(void);
void TIMER_Enable(char enable);
float getUpdateDT(void);
#endif
