{-# LANGUAGE ForeignFunctionInterface #-}
module Ranger.CApi (runRangerC, RangerCallbacks(..), RangerSignals(..)) where

import Foreign
import Ranger.CApi.Types
import Foreign.C

fooC :: CInt -> CInt
fooC x = 69 * x

foreign export ccall "foo" fooC :: CInt -> CInt

runRangerC :: Ptr RangerCallbacks -> IO (Ptr RangerSignals)
runRangerC = undefined

foreign export ccall "run_ranger" runRangerC :: Ptr RangerCallbacks -> IO (Ptr RangerSignals)
