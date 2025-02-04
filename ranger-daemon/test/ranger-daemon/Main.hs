module Main (main) where

import Test.Tasty
import Test.Tasty.HUnit
import Ranger.Bluetooth.SimpleBLE

main :: IO ()
main = defaultMain $
  testGroup "Ranger.Bluetooth"
    [ testGroup "SimpleBLE" [ callAdapterGetCount, callAdapterIsBluetoothEnabled ] ]

callAdapterGetCount :: TestTree
callAdapterGetCount = testCase "getAdapterCount" $ do
  c <- getAdapterCount
  print c

callAdapterIsBluetoothEnabled :: TestTree
callAdapterIsBluetoothEnabled = testCase "isBluetoothEnabled" $ do
  b <- isBluetoothEnabled
  print b
