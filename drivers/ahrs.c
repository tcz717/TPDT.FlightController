#include "ahrs.h"
#include "stm32f10x.h"                  // Device header

/*=====================================================================================================*/
/*=====================================================================================================*/
/*=====================================================================================================*/
/*=====================================================================================================*
**: MoveAve_SMA
**: Simple Moving Average
**: NewData, MoveAve_FIFO, SampleNum
**: AveData
**: MoveAve_SMA(NewData, MoveAve_FIFO, SampleNum)
**=====================================================================================================*/
/*=====================================================================================================*/
static s16 MoveAve_SMA(volatile int16_t NewData, volatile int16_t *MoveAve_FIFO, u8 SampleNum)
{
	u8 i = 0;
	s16 AveData = 0;
	s32 MoveAve_Sum = 0;

	for (i = 0; i < SampleNum - 1; i++)       // ????
		MoveAve_FIFO[i] = MoveAve_FIFO[i + 1];

	MoveAve_FIFO[SampleNum - 1] = NewData;    // ?????

	for (i = 0; i < SampleNum; i++)           // ??
		MoveAve_Sum += MoveAve_FIFO[i];

	AveData = (s16)(MoveAve_Sum / SampleNum); // ?????

	return AveData;
}
/*=====================================================================================================*/
/*=====================================================================================================*
**: MoveAve_WMA
**: Weighted Moving Average
**: NewData, MoveAve_FIFO, SampleNum
**: AveData
**: MoveAve_WMA(NewData, MoveAve_FIFO, SampleNum)
**=====================================================================================================*/
/*=====================================================================================================*/
static s16 MoveAve_WMA(volatile int16_t NewData, volatile int16_t *MoveAve_FIFO, u8 SampleNum)
{
	u8 i = 0;
	s16 AveData = 0;
	u16 SampleSum = 0;
	s32 MoveAve_Sum = 0;

	for (i = 0; i < SampleNum - 1; i++)         // ????
		MoveAve_FIFO[i] = MoveAve_FIFO[i + 1];

	MoveAve_FIFO[SampleNum - 1] = NewData;      // ?????

	for (i = 0; i < SampleNum; i++)             // ?? & ??
		MoveAve_Sum += MoveAve_FIFO[i] * (i + 1);

	SampleSum = (SampleNum * (SampleNum + 1)) / 2; // ??????
	AveData = (s16)(MoveAve_Sum / SampleSum);   // ?????

	return AveData;
}
