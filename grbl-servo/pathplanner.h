#include <grbl.h>

#ifndef pathplanner_h
#define pathplanner_h
void pathplanner_init();
void pathplanner_jog(float v[N_AXIS]);
void pathplanner_home(float v[N_AXIS], uint8_t axes_mask);
int pathplanner_ishomingdone();
void pathplanner_endjog();
void pathplanner_set_override(float override);
void pathplanner_execute();
#endif
