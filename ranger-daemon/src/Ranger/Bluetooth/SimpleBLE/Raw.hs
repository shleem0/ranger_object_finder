module Ranger.Bluetooth.SimpleBLE.Raw where

import Ranger.Bluetooth.SimpleBLE.Raw.Types
import Foreign.C

foreign import ccall unsafe "simpleble_c/adapter.h simpleble_adapter_get_count"
  simpleble_adapter_get_count :: IO CSize
