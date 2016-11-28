#include <grbl.h>
#include "servo.h"

#define PP_FLAG_ACTIVE (1<<4)
#define PP_FLAG_JOG (1<<5)
#define PP_FLAG_NEED_DISCARD (1<<6)

#define PP_FLAG_HOMEMASK (0xf)

typedef struct {
    int flags;
    // Current block base position in machine coords
    int base[N_AXIS];
    // Current block total steps in machine coords
    int steps[N_AXIS];
    float p;
    float v;
    float v_overridden;
    float vexit_sqr;
    float nominal_speed;
    float acceleration;
    float millimeters;
    float override;

    float jog_v[N_AXIS];
    float jog_vset[N_AXIS];
    float jog_p[N_AXIS];

} pathplanner_t;


static pathplanner_t pp;

const float dt = 1.0f/60/20000;



void pathplanner_init() {
    
    // Force path planner to load new block
    pp.millimeters = 0.0f;
    pp.p = 0.0f;
    pp.override = 1.0f;
    pp.flags = 0;
}

void pathplanner_jog(float v[N_AXIS]) {
    int i;
    if (bit_isfalse(pp.flags, PP_FLAG_JOG)) {
        protocol_buffer_synchronize(); // flush gcode queue
        for(i=0;i<N_AXIS;i++) {
            pp.jog_v[i] = 0.0f;
            pp.jog_p[i] = 0.0f;
            pp.base[i] = sys.position[i];
        }
        bit_true(pp.flags, PP_FLAG_JOG);
    }

    for(i=0;i<N_AXIS;i++) {
        pp.jog_vset[i] = v[i];
    }

}

void pathplanner_home(float v[N_AXIS], uint8_t axes_mask) {
    bit_true(pp.flags, axes_mask & PP_FLAG_HOMEMASK);
    pathplanner_jog(v);
}

int pathplanner_ishomingdone() {
    return (pp.flags & PP_FLAG_HOMEMASK) == 0;
}

void pathplanner_endjog() {
    int i;
    uint8_t stopped;
    if (bit_istrue(pp.flags, PP_FLAG_JOG)) {
        // Request jogging mode stop
        for(i=0;i<N_AXIS;i++) {
            pp.jog_vset[i] = 0.0f;
        }

        // Wait for system to come to complete stop
        do {
            protocol_execute_realtime();
            if (sys.abort) return;
            
            stopped = 1;
            for(i=0;i<N_AXIS;i++) {
                if (pp.jog_v[i] != 0.0f) stopped = 0;
            }
        } while (!stopped);
        
        for(i=0;i<N_AXIS;i++) {
            pp.base[i] = sys.position[i];
            pp.steps[i] = 0;
        }
        pp.millimeters = 0.0f;
        pp.p = 0.0f;
        
        plan_sync_position();
        gc_sync_position();
                
        bit_false(pp.flags, PP_FLAG_JOG);
        

    }

}


void pathplanner_set_override(float override) {
    pp.override = min(max(override, 0.0f), 1.0f);
}


static inline float sqr(float x) { return (x*x); }

// Called by planner_recalculate() when the executing block is updated by the new plan.
void st_update_plan_block_parameters() {
    pp.vexit_sqr = sqr(plan_get_exec_block_exit_speed());
}

static void pathplanner_discard_current_block_if_required() {
    if (bit_istrue(pp.flags, PP_FLAG_NEED_DISCARD)) {
        plan_discard_current_block();
        bit_false(pp.flags, PP_FLAG_NEED_DISCARD);
    }
}

void st_prep_buffer() {
    // called from protocol_execute_realtime via st_prep_buffer

    // Get new block
    plan_block_t *b;
    int i;

    if (plan_get_current_block() && bit_istrue(pp.flags, PP_FLAG_JOG)) {
        //raise alarm when plan contains a block (i.e. g-code issued while jogging)
        bit_true_atomic(sys_rt_exec_alarm, EXEC_ALARM_ABORT_CYCLE);
        return;
    }

    while (pp.p >= pp.millimeters || pp.millimeters == 0.0f) {
        // Current block done; discard it
        pathplanner_discard_current_block_if_required();

        // Update path planner to complete previous block: step-accurate update
        // of the path
        for(i=0;i<N_AXIS;i++) {
            pp.base[i] += pp.steps[i];
        }
        pp.p -= pp.millimeters;

        // Get new block from the buffer
        b = plan_get_current_block();

        if (!b) {
            // No more blocks, load dummy zero-length block
            pp.millimeters = 0.0f;
            for(i=0;i<N_AXIS;i++) {
                pp.steps[i] = 0;
            }
            return;
        }
        
        bit_true(pp.flags, PP_FLAG_NEED_DISCARD);

        for(i=0;i<N_AXIS;i++) {
            // Update path planner to complete previous block: step-accurate update
            // of the path
            pp.steps[i] = (b->direction_bits & get_direction_pin_mask(i)) ? -((int32_t)b->steps[i]): b->steps[i];
        }


        pp.millimeters = b->millimeters;
    
    
        pp.vexit_sqr = sqr(plan_get_exec_block_exit_speed());
        
        pp.nominal_speed = sqrt(b->nominal_speed_sqr);
        pp.acceleration = b->acceleration;

        
    }
}



void pathplanner_execute() {

    // Called from control loop interrupt routine

    float fact;
    int32_t pos[N_AXIS];
    int i;
    int axes_home;
    float v;


    float remaining;
    float override;


    override = pp.override;

    if (bit_isfalse(pp.flags, PP_FLAG_ACTIVE)) {
        return;
    }

    if (sys.state & (STATE_HOLD|STATE_MOTION_CANCEL|STATE_SAFETY_DOOR)) {
        override = 0;
    }


    if (bit_istrue(pp.flags, PP_FLAG_JOG)) {
        /* Jogging pathplanner: individual 2nd order motion for each axis
         *
         */

        /* Homing: sampling of 0-position and setting velocity to zero */
        axes_home = pp.flags & PP_FLAG_HOMEMASK;
        if (axes_home) {
            axes_home &= limits_get_state();

            for(i=0;i<N_AXIS;i++) {
                if (axes_home & (1<<i)) {
                    pp.jog_vset[i] = 0.0f;
                    bit_false(pp.flags, (1<<i));
                    encoder_home(i);
                    pp.base[i] = 0;
                    pp.jog_p[i] = 0.0f;
                }
            }

        }


        for(i=0;i<N_AXIS;i++) {
            v = min(max(pp.jog_vset[i] * override, -settings.max_rate[i]), settings.max_rate[i]);

            pp.jog_v[i] = min(max(
                v,
                pp.jog_v[i] - settings.acceleration[i] * dt),
                pp.jog_v[i] + settings.acceleration[i] * dt);

            pp.jog_p[i] += pp.jog_v[i] * dt;

            pos[i] = pp.base[i] + pp.jog_p[i] * settings.steps_per_mm[i];
        }

    } else {
        /* Position-based pathplanner
         *
         * single 2nd-order motion pathplanner
         */

        if (pp.millimeters <= 0) {
            fact = 0;
        } else {
            fact = pp.p/pp.millimeters;
        }

        if (fact>1.0f) fact = 1.0f; // should not happen if pathplanner is fed appropriately

        for(i=0;i<N_AXIS;i++) {
            pos[i] = pp.base[i] + fact * pp.steps[i];
        }

        // Accelerate max
        pp.v += pp.acceleration * dt * override;

        // Limit velocity to ensure exit speed is reached
        remaining = max(0, pp.millimeters-pp.p);
        pp.v = min(pp.v, sqrt(2*(pp.acceleration)*remaining + pp.vexit_sqr) );

        // Limit velocity to nominal
        pp.v = min(pp.v, pp.nominal_speed);

        // Move v_overridden to pp.v*override, but limit acceleration to pp.acceleration * dt
        pp.v_overridden = min(max(pp.v*override, pp.v_overridden - pp.acceleration *dt), pp.v_overridden + pp.acceleration * dt);

        // Update position within block
        pp.p += pp.v_overridden * dt;

        if (pp.v_overridden == 0) {
            bit_true_atomic(sys_rt_exec_state,EXEC_CYCLE_STOP); // Flag main program for cycle end
        }

    }


    // Update system position
    memcpy(&sys.position, pos, sizeof(pos));
}




void st_wake_up() {
    bit_true(pp.flags, PP_FLAG_ACTIVE);
}

void st_go_idle() {
    bit_false(pp.flags, PP_FLAG_ACTIVE);

}

void st_reset() {
    int i;
    st_go_idle();
    pathplanner_discard_current_block_if_required();

    for(i=0;i<N_AXIS;i++) {
        pp.base[i] = sys.position[i];
        pp.steps[i] = 0;
    }
    pp.millimeters = 0.0f;
    pp.p = 0.0f;

}
