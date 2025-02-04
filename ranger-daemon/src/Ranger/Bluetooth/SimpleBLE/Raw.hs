module Ranger.Bluetooth.SimpleBLE.Raw where

import Ranger.Bluetooth.SimpleBLE.Raw.Types
import Foreign.C

foreign import ccall unsafe "simpleble_c/simpleble.h simpleble_adapter_get_count"
  simpleble_adapter_get_count :: IO CSize

foreign import ccall unsafe "simpleble_c/simpleble.h simpleble_adapter_is_bluetooth_enabled"
  simpleble_adapter_is_bluetooth_enabled :: IO CBool
