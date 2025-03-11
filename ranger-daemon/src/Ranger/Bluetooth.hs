{-# LANGUAGE TypeOperators #-}
{-# LANGUAGE DataKinds #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE LambdaCase #-}
{-# LANGUAGE BlockArguments #-}
{-# LANGUAGE GADTs #-}
{-# LANGUAGE ScopedTypeVariables #-}
module Ranger.Bluetooth (runRangerBluetooth) where

import Ranger.Bluetooth.Protocol
import qualified Ranger.Bluetooth.Msg as Msg
import Ranger.Bluetooth.Gatt
import Control.Exception
import Effectful
import Ranger.Effect
import Effectful.Dispatch.Dynamic
import Ranger.Demo
import Effectful.Error.Static
import Control.Monad.Sync
import Ranger.Bluetooth.Types
import Ranger.Bluetooth.Msg (Msg)
import Effectful.Concurrent.STM
import Effectful.State.Static.Local
import Data.Word
import Control.Monad

runRangerBluetooth :: IO ()
runRangerBluetooth = do
  result <- runRangerGatt
  (phoneToRanger, rangerToPhone) <- case result of
    Right comms -> pure comms
    Left e -> throwIO (RangerBluetoothException e)
  undefined

data RangerInvalidState = RangerInvalidState deriving Show

newtype PacketIndex = PacketIndex Word8

itp :: (RangerControl :> es, Error RangerInvalidState :> es, Concurrent :> es, State PacketIndex :> es) => RangerComms -> Interpreter 'Ranger Msg (Eff es)
itp (phoneToRanger, rangerToPhone) = Interpreter
  { side = SRanger
  , Control.Monad.Sync.send = \msg x -> atomically $ writeTQueue rangerToPhone (SomeMsg (msg, x))
  , recv = \_ SPhone msg -> do
      (idx, SomeMsg (msg', val)) <- atomically $ readTQueue phoneToRanger
      PacketIndex idx' <- get
      when (idx /= idx') $ throwError RangerInvalidState -- TODO: handle out-of-order packets better
      case (msg, msg') of
        (Msg.FunctionCall, Msg.FunctionCall) -> pure val
        (Msg.AnnounceNFragments, Msg.AnnounceNFragments) -> pure val
        (Msg.SendPhotoFragment, Msg.SendPhotoFragment) -> pure val
        _ -> throwError RangerInvalidState
  }

runRangerControl :: (HasCallStack, IOE :> es) => Eff (RangerControl : es) a -> Eff es a
runRangerControl = interpret $ \_ -> \case
  StartDemo -> liftIO demoProcedure
  CancelDemo -> liftIO cancelDemoProcedure
  StartSearch _ _ -> liftIO $ fail "unimplemented"
  CancelSearch -> liftIO $ fail "unimplemented"
  UpdateSearch _ -> liftIO $ fail "unimplemented"
  SaveObject _ _ -> liftIO $ fail "unimplemented"
  GetObjectPhotos _ -> liftIO $ fail "unimplemented"
  DeleteObject _ -> liftIO $ fail "unimplemented"
  GetNotificationPhoto -> liftIO $ fail "unimplemented"
  PowerOff -> liftIO $ fail "unimplemented"
