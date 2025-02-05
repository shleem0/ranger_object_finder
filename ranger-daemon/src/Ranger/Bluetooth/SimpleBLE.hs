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
import Foreign.Ptr
import Data.Maybe
import Ranger.Bluetooth.SimpleBLE.Raw.Types
import Foreign.ForeignPtr
import Control.Monad
import Data.Text (Text)
import qualified Data.Text as T
import qualified Data.Text.Foreign as T
import Data.Map.Strict (Map)
import qualified Data.Map.Strict as Map
import Control.Exception (bracket, finally)
import Control.Concurrent.Async
import Control.Concurrent.Chan
import Control.Monad.IO.Class
import Conduit


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

data ScanStatus = ScanFound Peripheral | ScanUpdated Peripheral

adapterScanFor :: Adapter -> Int -> ConduitT () ScanStatus IO [Peripheral]
adapterScanFor Adapter{adapterHandle} timeoutMillis = do

  results <- liftIO newChan

  ps <- liftIO $ withAsyncBound
    (bracket
      (liftIO newChan)

      (\fptrs -> do
        -- Terminate results stream
        writeChan results Nothing

        -- Free the funptrs when done
        writeChan fptrs Nothing
        fptrs' <- catMaybes . takeWhile isJust <$> getChanContents fptrs
        mapM_ freeHaskellFunPtr fptrs')

      -- Scan for timeoutMillis, write intermediate results to results chan
      (\fptrs -> withForeignPtr adapterHandle $ \ptr -> do

        foundFptr <- mkScanPeripheralCallback $ \_ peripheralPtr -> do
          peripheralPtr' <- newForeignPtr simpleble_peripheral_release_handle peripheralPtr
          writeChan results (Just . ScanFound $ Peripheral peripheralPtr')

        updatedFptr <- mkScanPeripheralCallback $ \_ (peripheralPtr :: Ptr ()) -> do
          peripheralPtr' <- newForeignPtr simpleble_peripheral_release_handle peripheralPtr
          writeChan results (Just . ScanUpdated $ Peripheral peripheralPtr')

        e1 <- adapter_set_callback_on_scan_found ptr foundFptr
        writeChan fptrs (Just foundFptr)

        e2 <- adapter_set_callback_on_scan_updated ptr updatedFptr
        writeChan fptrs (Just updatedFptr)

        e3 <- simpleble_adapter_scan_for ptr (fromIntegral timeoutMillis)

        unless (all (== SIMPLEBLE_SUCCESS) [e1,e2,e3]) $ fail "adapterScanFor: simpleble scan fail"

        -- todo: final scan result
        undefined)
    )
    wait
  
  -- Send statuses into the conduit stream while the channel is open
  statuses <- liftIO (catMaybes . takeWhile isJust <$> getChanContents results)
  yieldMany statuses

  pure ps
 