#ifndef __DEBUG_H_
#define __DEBUG_H_

extern void TxSend(u8 soft_uart,u8 dat);
extern void Fake_PrintString(u8 soft_uart,u8 *puts);
extern void Fake_PrintString1(u8 soft_uart,u8 *puts);
extern void Debug_LedReverse(void);
extern void Debug_LedOn(BOOL enable);

#endif
