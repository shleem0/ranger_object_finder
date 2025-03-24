#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define OBJECT_ID_LEN 16 // no null terminator!

typedef struct {
    char value[OBJECT_ID_LEN];
} object_id_t;

typedef struct {
    uint8_t timeout;
    uint16_t max_radius;
} search_parameters_t;

typedef struct {
    void (*on_demo_start)();
    void (*on_demo_cancel)();
    void (*on_search_start)(object_id_t* oid, search_parameters_t* params);
    void (*on_search_cancel)();
    void (*on_search_update)(search_parameters_t* params);
    void (*on_user_photo_response)(bool response);
} ranger_callbacks_t;

typedef struct {
    void (*demo_ended)();
    void (*photo_prompt)(size_t photo_num_bytes, uint8_t *photo_bytes[]);
    void (*search_failed)();
    void (*retrieving_object)();
    void (*object_return_success)();
    void (*object_return_fail)();
} ranger_signals_t;
