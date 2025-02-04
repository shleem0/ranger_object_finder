{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE LambdaCase #-}
{-# LANGUAGE RankNTypes #-}
module Main (main) where

import Foreign.C
import System.Exit

foreign import ccall unsafe "run"
  simpleble_example_run :: IO CSize

main :: IO ()
main = simpleble_example_run >>= \case
  0 -> exitSuccess
  n -> exitWith $ ExitFailure $ fromIntegral n
