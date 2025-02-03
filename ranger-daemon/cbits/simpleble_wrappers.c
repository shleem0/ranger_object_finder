#include <simpleble_c/simpleble.h>

bool adapter_is_bluetooth_enabled() {
    return simpleble_adapter_is_bluetooth_enabled();
}

size_t adapter_get_count() {
    return simpleble_adapter_get_count();
}
