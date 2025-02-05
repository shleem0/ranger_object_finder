#pragma once

#include <simpleble_c/simpleble.h>

// Haskell FFI doesn't support working with by-value C structs, so we need to wrap them in pointers

simpleble_err_t adapter_set_callback_on_scan_updated(
    simpleble_adapter_t handle,
    void (*callback)(simpleble_adapter_t adapter, simpleble_peripheral_t* peripheral),
);

simpleble_err_t adapter_set_callback_on_scan_found(
    simpleble_adapter_t handle,
    void (*callback)(simpleble_adapter_t adapter, simpleble_peripheral_t* peripheral),
);

simpleble_err_t adapter_set_callback_on_scan_start(simpleble_adapter_t handle, void (*callback)(simpleble_adapter_t adapter));

simpleble_err_t adapter_set_callback_on_scan_stop(simpleble_adapter_t handle, void (*callback)(simpleble_adapter_t adapter));
