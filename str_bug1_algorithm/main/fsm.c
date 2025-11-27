#include "fsm.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "fsm_states.h"

typedef int (*fsm_state_t)(struct fsm_event *event);

static int fsm_execute(struct fsm_event *event);

static struct fsm {
    fsm_state_t current_state;
    fsm_state_t states[__FSM_STATE_AMOUNT];
    struct fsm_event event_queue[FSM_MAX_EVENT_QUEUE_SIZE];
    size_t head;
    size_t tail;
    size_t count;
} self = {
    .current_state = fsm_state_waiting_goal,
    .states =
        {
            [FSM_STATE_WAITING_GOAL]       = fsm_state_waiting_goal,
            [FSM_STATE_MOTION_TO_GOAL]     = fsm_state_motion_to_goal,
            [FSM_STATE_BOUNDARY_FOLLOWING] = fsm_state_boundary_following,
        },
    .event_queue = {},
    .head        = 0,
    .tail        = 0,
    .count       = 0,
};

int fsm_event_post(enum fsm_event_id id, uint8_t *data, size_t len)
{
    if (self.count == FSM_MAX_EVENT_QUEUE_SIZE) {
        return -1;
    }

    self.event_queue[self.tail].id  = id;
    self.event_queue[self.tail].len = len;

    if (data && len > 0) {
        memcpy(self.event_queue[self.tail].data, data, len);
    }

    self.tail = (self.tail + 1) % FSM_MAX_EVENT_QUEUE_SIZE;
    self.count++;
    return 0;
}


int fsm_state_task(void)
{
    struct fsm_event event = {0};
    int err                = 0;

    if (self.count == 0) {
        return err;
    }

    err = fsm_event_get(&event);
    if (err == 0) {
        err = fsm_execute(&event);
    }

    return err;
}

int fsm_event_get(struct fsm_event *evt)
{
    if (self.count == 0) {
        return -1;
    }

    *evt      = self.event_queue[self.head];
    self.head = (self.head + 1) % FSM_MAX_EVENT_QUEUE_SIZE;
    self.count--;
    return 0;
}

static int fsm_execute(struct fsm_event *event)
{
    int next_state = -1;

    self.current_state(event);

    if (event->id == FSM_EVENT_ID_EXIT) {
        next_state = event->data[0];
        self.current_state = self.states[next_state];

        struct fsm_event entry_event = {
            .id = FSM_EVENT_ID_ENTRY,
            .len = 0
        };

        self.current_state(&entry_event);
    }

    return 0;
}