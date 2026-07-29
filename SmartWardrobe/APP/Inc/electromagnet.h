#ifndef ELECTROMAGNET_H
#define ELECTROMAGNET_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* PD8 驱动继电器，GPIO 高电平表示吸合。 */
void Electromagnet_Init(void);
int Electromagnet_Set(uint8_t on);
void Electromagnet_ForceOff(void);
uint8_t Electromagnet_IsOn(void);
uint8_t Electromagnet_PinHigh(void);

#ifdef __cplusplus
}
#endif

#endif
