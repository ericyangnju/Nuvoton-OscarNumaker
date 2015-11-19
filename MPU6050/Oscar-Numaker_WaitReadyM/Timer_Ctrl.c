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
#include <stdio.h>
#include "math.h"
#include "Def.h"
#include "isd9100.h"
#include "Timer_Ctrl.h"
#include "AHRSLib.h"


typedef struct {
	int lastTime;
	int interval0;
	int currentTime; 
}Chronograph_T;
Chronograph_T Chronograph[NumCron];

volatile float UPDATE_DT;
volatile uint32_t tick_counter = 0;
volatile uint32_t frame_counter = 0;
volatile uint32_t GPABCallback_Counter=0;

volatile uint32_t u32Timer0Cnt=0, u32Timer1Cnt=0, u32Timer2Cnt=0, u32Timer3Cnt=0;

void Delay(uint32_t delayCnt)
{
	while(delayCnt--) {
		__NOP();
		__NOP();
	}
}
void DelayLoop(unsigned short delay){
	while(--delay);//About 200 psec per run
}
void DelayUsec(unsigned int usec){
	while(usec--)
		DelayLoop(5);
}
void DelayMsec(unsigned short msec){
	int tick_count= getTickCount();
	while((getTickCount()-tick_count)<msec);
}


void setup_system_tick(uint32_t sampleRate)
{
	uint32_t tickPeriod = SystemCoreClock/sampleRate;
	
	SysTick_Config(tickPeriod);
	ChronographStart(ChronMain);
//	printf("SystemCoreClock:%d\n",SystemCoreClock);
//	printf("Tick Time: %d ms\n",1000/sampleRate);
}
void SysTick_Handler(void)
{
	static int FC_Last;
	int freqCount=0;

//PB->DOUT &= ~(1 << 0);	
	
	if((tick_counter%1000)==0) {
		freqCount=frame_counter-FC_Last;
		UPDATE_DT = (float)(1.0f/freqCount);
#if DISPLAY_LOOP_TIME
		printf("FC:%d\n",freqCount);
#endif
		FC_Last = frame_counter;
	}

	tick_counter++;
	nvtMillisecondTick();
//PB->DOUT |= (1 << 0);
}

void TMR0_IRQHandler(void)
{
  static int FC_Last;
	int freqCount=0; 
	// printf("Timer IRQ handler test #%d/3.\n", ++u8Counter );

PB->DOUT &= ~(1 << 0);
	TIMER_ClearIntFlag(TIMER0);	
	
	if((tick_counter%1000)==0) {
		freqCount=frame_counter-FC_Last;
		UPDATE_DT = (float)(1.0f/freqCount);
#if DISPLAY_LOOP_TIME
		printf("FC:%d\n",freqCount);
#endif
		FC_Last = frame_counter;
	}

	tick_counter++;
	nvtMillisecondTick();
PB->DOUT |= (1 << 0);	
}

void IncFrameCount(int inc)
{
	frame_counter+=inc;
}

uint32_t GetFrameCount()
{
	return frame_counter;
}

void ChronographSet(char Chron)
{
	Chronograph[Chron].lastTime = Chronograph[Chron].currentTime;
	Chronograph[Chron].currentTime = tick_counter;  
}

void ChronographStart(char Chron)
{
	Chronograph[Chron].currentTime = Chronograph[Chron].lastTime = tick_counter;
}

int32_t ChronographRead(char Chron)
{
	int32_t chron_diff = (tick_counter - Chronograph[Chron].currentTime);
	
	if(Chron==ChronMain)
		UPDATE_DT = (float)chron_diff/1000;
	
	return chron_diff;
}

float getUpdateDT()
{
	return UPDATE_DT;
}

int getTickCount()
{
	return tick_counter;
}

uint32_t micros()
{
	return u32Timer2Cnt;
}

uint32_t millis()
{
	return (u32Timer2Cnt/1000);
}


void TIMER_Init()
{
	NVIC_EnableIRQ(SysTick_IRQn);
}
