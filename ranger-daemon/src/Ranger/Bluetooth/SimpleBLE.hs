module Ranger.Bluetooth.SimpleBLE
  ( isBluetoothEnabled
  , getAdapterCount
  , getBluetoothAdapters
  , adapterMacAddress
  , Adapter
  , Peripheral
  ) where

import Ranger.Bluetooth.SimpleBLE.Raw
import Foreign.C
import Ranger.Bluetooth.SimpleBLE.Raw.Types
import Foreign.ForeignPtr
import Control.Monad
import Data.Text (Text)
import qualified Data.Text as T
import qualified Data.Text.Foreign as T
import Data.Map.Strict (Map)
import qualified Data.Map.Strict as Map


newtype Adapter = Adapter { adapterHandle :: ForeignPtr () }
newtype Peripheral = Peripheral { peripheralHandle :: ForeignPtr () }

adapterMacAddress :: Adapter -> IO Text
adapterMacAddress Adapter{adapterHandle} = withForeignPtr adapterHandle $ \ptr -> do
  addrPtr <- simpleble_adapter_address ptr
  address <- T.peekCString addrPtr
  simpleble_free addrPtr
  pure address

getBluetoothAdapters :: IO (Map Text Adapter)
getBluetoothAdapters = do
  count <- getAdapterCount
  adapters <- forM [0..count - 1] $ \adapterIndex -> do
    ptr <- simpleble_adapter_get_handle adapterIndex
    idPtr <- simpleble_adapter_identifier ptr
    identifier <- T.peekCString idPtr
    simpleble_free idPtr
    fptr <- newForeignPtr simpleble_adapter_release_handle ptr
    pure (identifier, Adapter fptr)
  pure $ Map.fromList adapters
