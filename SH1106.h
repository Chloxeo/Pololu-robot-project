#include <stdlib.h>
extern void sh1106InitPins();
extern void sh1106Reset();
extern void sh1106TransferStart();
extern void sh1106TransferEnd();
extern void sh1106CommandMode();
extern void sh1106DataMode();
extern void sh1106Write(uint8_t d);
