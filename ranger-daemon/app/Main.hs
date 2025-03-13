module Main (main) where

import Ranger.Bluetooth
import GHC.Stack

main :: HasCallStack => IO ()
main = runRangerBluetooth
