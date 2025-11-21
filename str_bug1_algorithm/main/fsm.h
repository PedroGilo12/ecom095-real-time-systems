#ifndef FSM_H
#define FSM_H

#include <stdint.h>
#include <stddef.h>

#define FSM_MAX_EVENT_LENGHT 4
#define FSM_MAX_EVENT_QUEUE_SIZE 5

enum fsm_event_id {
    FSM_EVENT_ID_ENTRY = 0,
    FSM_EVENT_ID_EXIT,
};

struct fsm_event {
    enum fsm_event_id id;
    uint8_t data[FSM_MAX_EVENT_LENGHT];
    size_t len;
};

int fsm_state_task(void);

int fsm_event_post(enum fsm_event_id id, uint8_t *data, size_t len);

int fsm_event_get(struct fsm_event *evt);

#endif /* FSM_H */