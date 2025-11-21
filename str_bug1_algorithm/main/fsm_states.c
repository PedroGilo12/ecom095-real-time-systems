#include <stdio.h>

#include "fsm.h"
#include "motor_driver.h"

int fsm_state_waiting_goal(struct fsm_event *event)
{
    switch (event->id) {
    case FSM_EVENT_ID_ENTRY: {
        printf("Entry %s\n", __func__);
        motor_driver_move_left(160, false);
        motor_driver_move_right(160, false);
    } break;
    case FSM_EVENT_ID_EXIT: {
        printf("Exit %s\n", __func__);
    } break;
    }
    return 0;
}

int fsm_state_motion_to_goal(struct fsm_event *event)
{
    switch (event->id) {
    case FSM_EVENT_ID_ENTRY: {
        printf("Entry %s\n", __func__);
        motor_driver_move_left(0, false);
        motor_driver_move_right(0, false);
    } break;
    case FSM_EVENT_ID_EXIT: {
        printf("Exit %s\n", __func__);
    } break;
    }
    return 0;
}

int fsm_state_boundary_following(struct fsm_event *event)
{
    switch (event->id) {
    case FSM_EVENT_ID_ENTRY: {
    } break;
    case FSM_EVENT_ID_EXIT: {
    } break;
    }
    return 0;
}