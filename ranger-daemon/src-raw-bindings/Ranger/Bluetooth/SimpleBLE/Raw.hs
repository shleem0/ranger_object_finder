module Ranger.Bluetooth.SimpleBLE.Raw where

import Ranger.Bluetooth.SimpleBLE.Raw.Types
import Foreign.C
import Foreign.Ptr

foreign import ccall unsafe "simpleble_c/simpleble.h simpleble_adapter_get_count"
  getAdapterCount :: IO CSize

foreign import ccall unsafe "simpleble_c/simpleble.h simpleble_adapter_is_bluetooth_enabled"
  isBluetoothEnabled :: IO Bool

foreign import ccall unsafe "simpleble_c/simpleble.h simpleble_adapter_get_handle"
  simpleble_adapter_get_handle :: CSize -> IO SimpleBleAdapter

foreign import ccall unsafe "simpleble_c/simpleble.h &simpleble_adapter_release_handle"
  simpleble_adapter_release_handle :: FunPtr (SimpleBleAdapter -> IO ())

foreign import ccall unsafe "simpleble_c/simpleble.h simpleble_adapter_identifier"
  simpleble_adapter_identifier :: SimpleBleAdapter -> IO CString

foreign import ccall unsafe "simpleble_c/simpleble.h simpleble_adapter_address"
  simpleble_adapter_address :: SimpleBleAdapter -> IO CString

foreign import ccall unsafe "simpleble_c/simpleble.h simpleble_free"
  simpleble_free :: Ptr a -> IO ()
