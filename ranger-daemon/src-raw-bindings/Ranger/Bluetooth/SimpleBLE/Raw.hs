module Ranger.Bluetooth.SimpleBLE.Raw where

import Ranger.Bluetooth.SimpleBLE.Raw.Types
import Foreign.C
import Foreign.Ptr

foreign import ccall "simpleble_c/simpleble.h simpleble_free"
  simpleble_free :: Ptr a -> IO ()


foreign import ccall "simpleble_c/simpleble.h simpleble_adapter_get_count"
  getAdapterCount :: IO CSize

foreign import ccall "simpleble_c/simpleble.h simpleble_adapter_is_bluetooth_enabled"
  isBluetoothEnabled :: IO Bool

foreign import ccall "simpleble_c/simpleble.h simpleble_adapter_get_handle"
  simpleble_adapter_get_handle :: CSize -> IO SimpleBleAdapter

foreign import ccall "simpleble_c/simpleble.h &simpleble_adapter_release_handle"
  simpleble_adapter_release_handle :: FunPtr (SimpleBleAdapter -> IO ())

foreign import ccall "simpleble_c/simpleble.h simpleble_adapter_identifier"
  simpleble_adapter_identifier :: SimpleBleAdapter -> IO CString

foreign import ccall "simpleble_c/simpleble.h simpleble_adapter_address"
  simpleble_adapter_address :: SimpleBleAdapter -> IO CString

foreign import ccall "simpleble_c/simpleble.h simpleble_adapter_scan_for"
  simpleble_adapter_scan_for :: SimpleBleAdapter -> CInt -> IO SimpleBleResult


type HsScanPeripheralCallback = SimpleBleAdapter -> SimpleBlePeripheral -> IO ()

foreign import ccall "wrapper"
  mkScanPeripheralCallback :: HsScanPeripheralCallback -> IO (FunPtr HsScanPeripheralCallback)

foreign import ccall "adapter_set_callback_on_scan_updated"
  adapter_set_callback_on_scan_updated :: SimpleBleAdapter -> FunPtr HsScanPeripheralCallback -> IO SimpleBleResult

foreign import ccall "adapter_set_callback_on_scan_found"
  adapter_set_callback_on_scan_found :: SimpleBleAdapter -> FunPtr HsScanPeripheralCallback -> IO SimpleBleResult


type HsScanStartStopCallback = SimpleBleAdapter -> IO ()

foreign import ccall "wrapper"
  mkScanStartStopCallback :: HsScanStartStopCallback -> IO (FunPtr HsScanStartStopCallback)

foreign import ccall "adapter_set_callback_on_scan_start"
  adapter_set_callback_on_scan_start :: SimpleBleAdapter -> FunPtr HsScanStartStopCallback -> IO SimpleBleResult

foreign import ccall "adapter_set_callback_on_scan_stop"
  adapter_set_callback_on_scan_stop :: SimpleBleAdapter -> FunPtr HsScanStartStopCallback -> IO SimpleBleResult


foreign import ccall "simpleble_c/simpleble.h &simpleble_peripheral_release_handle"
  simpleble_peripheral_release_handle :: FunPtr (SimpleBlePeripheral -> IO ())

foreign import ccall "simpleble_c/simpleble.h simpleble_adapter_scan_get_results_handle"
  simpleble_adapter_scan_get_results_handle :: SimpleBleAdapter -> CSize -> IO SimpleBlePeripheral

foreign import ccall "simpleble_c/simpleble.h simpleble_adapter_scan_get_results_count"
  simpleble_adapter_scan_get_results_count :: SimpleBleAdapter -> IO CSize
