{-# LANGUAGE ForeignFunctionInterface #-}
module Ranger.CApi (runRangerC, RangerCallbacks(..), RangerSignals(..)) where

import Foreign
import Ranger.CApi.Types

runRangerC :: Ptr RangerCallbacks -> IO (Ptr RangerSignals)
runRangerC = undefined

foreign export ccall runRangerC :: Ptr RangerCallbacks -> IO (Ptr RangerSignals)
