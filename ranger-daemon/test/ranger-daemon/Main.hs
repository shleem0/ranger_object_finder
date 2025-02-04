module Main (main) where

import Test.Tasty
import Test.Tasty.HUnit
import Ranger.Bluetooth.SimpleBLE.Raw

main :: IO ()
main = defaultMain $
  testGroup "Ranger.Bluetooth.SimpleBLE"
    [ testGroup "Raw" [ callAdapterGetCount, callAdapterIsBluetoothEnabled ] ]

callAdapterGetCount :: TestTree
callAdapterGetCount = testCase "simpleble_adapter_get_count" $ do
  c <- simpleble_adapter_get_count
  print c

callAdapterIsBluetoothEnabled :: TestTree
callAdapterIsBluetoothEnabled = testCase "simpleble_adapter_is_bluetooth_enabled" $ do
  b <- simpleble_adapter_is_bluetooth_enabled
  print b
