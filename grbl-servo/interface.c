#include <grbl.h>
#include "pathplanner.h"
volatile io_sim_t io;

void _delay_ms(int i) { HAL_Delay(i); }

void stepper_init() { }



system_t sys; 


void coolant_run(uint8_t mode)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Ensure coolant turns on when specified in program.  
  coolant_set_state(mode);
}

void coolant_stop() {  }

void spindle_run(uint8_t state, float rpm)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.  
  spindle_set_state(state, rpm);
}

void spindle_stop() { }

char pgm_read_byte_near(const char* s) {
  return s[0];
}

void spindle_set_state(uint8_t state, float rpm) {
    if ((state == SPINDLE_ENABLE_CW) || (state == SPINDLE_ENABLE_CCW)) {
        //serial_write('S');
    } else {
        spindle_stop();
    }
}

void coolant_set_state(uint8_t state) {
    if (((state == COOLANT_MIST_ENABLE) || (state == COOLANT_FLOOD_ENABLE))) {
        //serial_write('C');
    } else {
        coolant_stop();
    }
}


// Returns machine position of axis 'idx'. Must be sent a 'step' array.
// NOTE: If motor steps and machine position are not in the same coordinate frame, this function
//   serves as a central place to compute the transformation.
float system_convert_axis_steps_to_mpos(int32_t *steps, uint8_t idx)
{
  float pos;
  pos = steps[idx]/settings.steps_per_mm[idx];

  return(pos);
}


void system_convert_array_steps_to_mpos(float *position, int32_t *steps)
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    position[idx] = system_convert_axis_steps_to_mpos(steps, idx);
  }
  return;
}

void limits_disable() { }


uint8_t limits_get_state() {
    return (~GPIOC->IDR) & 0x7;
}

void limits_go_home(uint8_t cycle_mask) {
    float v[N_AXIS];
    uint8_t moveaxes;
    int i, j;

    float rate;

    rate = settings.homing_seek_rate;

    for(j=0; j<2;j++) {

        moveaxes = cycle_mask;
        do {
            protocol_execute_realtime();
            if (sys.abort) return;
            // move away from limit switch
            for (i=0;i<N_AXIS;i++) {

                // once a switch has disengaged, ignore its state
                moveaxes &= limits_get_state();
                if (moveaxes & (1<<i)) {
                    v[i] = settings.homing_feed_rate;
                } else {
                    v[i] = 0.0f;
                }

                pathplanner_jog(v);
            }

        } while (moveaxes);

        pathplanner_endjog(); // wait for jogging to come to complete stop

        for (i=0;i<N_AXIS;i++) {
            v[i] = cycle_mask & (1<<i) ? -rate :0.0f;
        }

        pathplanner_home(v, cycle_mask);
        do {
            protocol_execute_realtime();
            if (sys.abort) return;
        } while (!pathplanner_ishomingdone());

        pathplanner_endjog(); // wait for jogging to come to complete stop

        rate = settings.homing_feed_rate;
    }


}



void limits_init() {}
void limits_soft_check(float *target) { }

uint8_t eeprom_data[1024];

void eeprom_put_char( unsigned int addr, unsigned char new_value ) {
    if (addr<sizeof(eeprom_data)) eeprom_data[addr] = new_value;
}

unsigned char eeprom_get_char( unsigned int addr ) {
    if (addr<sizeof(eeprom_data)) return eeprom_data[addr];
    return 0xff;
}

void st_generate_step_dir_invert_masks() { }

