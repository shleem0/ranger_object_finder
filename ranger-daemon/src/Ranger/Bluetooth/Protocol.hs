{-# LANGUAGE GADTs #-}
{-# LANGUAGE DataKinds #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE PolyKinds #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-# LANGUAGE TypeOperators #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE StandaloneDeriving #-}
{-# LANGUAGE DerivingVia #-}
{-# LANGUAGE PartialTypeSignatures #-}
{-# LANGUAGE LambdaCase #-}
module Ranger.Bluetooth.Protocol (bluetoothProtocol, RPSync, SideM(..)) where

import Control.Monad.Sync
import Conduit
import Bluetooth
import Data.Singletons
import Data.Kind
import Ranger.Bluetooth.Msg
import Control.Monad.Trans.Reader
import Data.Type.Bool
import Data.Type.Equality
import Data.Void
import Ranger.Demo
import Control.Monad

-- | A monad that can never be executed.
--
-- Useful for writing pseudocode.
newtype VoidM a = VoidM (Reader Void a) deriving (Functor, Applicative, Monad)

neverDone :: VoidM a
neverDone = VoidM ask >>= absurd

-- | Bluetooth communication monad.
newtype SideM (s :: Side) a = SideM { unSide :: If (s == 'Ranger) IO VoidM a }

deriving instance Functor (SideM 'Ranger)
deriving instance Applicative (SideM 'Ranger)
deriving instance Monad (SideM 'Ranger)
deriving instance Functor (SideM 'Phone)
deriving instance Applicative (SideM 'Phone)
deriving instance Monad (SideM 'Phone)

pseudocode :: SideM 'Phone a
pseudocode = SideM neverDone

-- | Ranger-Phone sync
type RPSync s = Sync s Msg (SideM s)

phoneCallsFunction :: RPSync s FunctionCall
phoneCallsFunction = sync SPhone FunctionCall pseudocode

bluetoothProtocol :: RPSync s ()
bluetoothProtocol = do
  runConduit
    $ repeatMC phoneCallsFunction
    .| takeWhileC (\case PowerOff -> False; _ -> True)
    .| mapM_C dispatchFunction
  dispatchFunction PowerOff

dispatchFunction :: FunctionCall -> RPSync s ()
dispatchFunction = \case
  StartDemo -> void $ private SRanger (Proxy :: Proxy ()) (SideM startDemo)
  CancelDemo -> void $ private SRanger (Proxy :: Proxy ()) (SideM cancelDemo)
  StartSearch _ _ -> undefined
  ModifySearchParameters _ -> undefined
  CancelSearch -> undefined
  UpdateObject _ _ -> undefined
  DeleteObject _ -> undefined
  GetObjectPhotos _ -> undefined
  DownloadNotificationPhoto -> undefined
  PowerOff -> undefined

