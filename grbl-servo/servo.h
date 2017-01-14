#ifndef servo_h
#define servo_h
typedef struct  {
    float Kp;
    float Ti;
    float Td;
    float i;
    float d;
    float limit;
    float previous;
    float ts;


} PIDControllerState;



typedef struct {
    PIDControllerState PID;
    TIM_HandleTypeDef encoder;
    int position;
} ServoAxis;


float computePID(float input, PIDControllerState *state);
void resetPID(PIDControllerState *state);

void encoder_home(uint8_t i);
void servo_init() ;
#endif
