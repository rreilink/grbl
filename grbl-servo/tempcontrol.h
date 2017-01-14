#include <grbl.h>

#ifndef tempcontrol_h
#define tempcontrol_h
void tempcontrol_init();
void tempcontrol_update();
float tempcontrol_getextrudertemp();
void tempcontrol_setextrudersetpoint(float t);
#endif
