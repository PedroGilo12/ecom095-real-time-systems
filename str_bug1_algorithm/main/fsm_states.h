#ifndef FSM_STATES
#define FSM_STATES

#include "fsm.h"

enum fsm_state {
    FSM_STATE_WAITING_GOAL = 0,
    FSM_STATE_MOTION_TO_GOAL,
    FSM_STATE_BOUNDARY_FOLLOWING,
    __FSM_STATE_AMOUNT,
};

int fsm_state_waiting_goal(struct fsm_event *event);
int fsm_state_motion_to_goal(struct fsm_event *event);
int fsm_state_boundary_following(struct fsm_event *event);

#endif /* FSM_STATES */