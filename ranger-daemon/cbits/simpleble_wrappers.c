#include <stdio.h>

#include <simpleble_c/simpleble.h>


typedef void scan_peripheral_callback_hs_t(simpleble_adapter_t adapter, simpleble_peripheral_t* peripheral);

void call_peripheral_userdata(simpleble_adapter_t adapter, simpleble_peripheral_t peripheral, void* userdata) {
    scan_peripheral_callback_hs_t* callback = (scan_peripheral_callback_hs_t*) userdata;
    callback(adapter, &peripheral);
} 

simpleble_err_t adapter_set_callback_on_scan_updated(
    simpleble_adapter_t handle,
    scan_peripheral_callback_hs_t* callback
) {
    return simpleble_adapter_set_callback_on_scan_updated(handle, &call_peripheral_userdata, callback);
}

simpleble_err_t adapter_set_callback_on_scan_found(
    simpleble_adapter_t handle,
    scan_peripheral_callback_hs_t* callback
) {
    return simpleble_adapter_set_callback_on_scan_found(handle, &call_peripheral_userdata, callback);
}


typedef void scan_callback_hs_t(simpleble_adapter_t adapter);

void call_userdata(simpleble_adapter_t adapter, void* userdata) {
    scan_callback_hs_t* callback = (scan_callback_hs_t*) userdata;
    callback(adapter);
}

simpleble_err_t adapter_set_callback_on_scan_start(
    simpleble_adapter_t handle,
    scan_callback_hs_t* callback
) {
    return simpleble_adapter_set_callback_on_scan_start(handle, &call_userdata, NULL);
}

simpleble_err_t adapter_set_callback_on_scan_stop(
    simpleble_adapter_t handle,
    scan_callback_hs_t* callback
) {
    return simpleble_adapter_set_callback_on_scan_stop(handle, &call_userdata, callback);
}
