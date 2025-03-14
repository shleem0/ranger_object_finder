{-# LANGUAGE TypeOperators #-}
{-# LANGUAGE DataKinds #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE LambdaCase #-}
{-# LANGUAGE BlockArguments #-}
{-# LANGUAGE GADTs #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-# LANGUAGE KindSignatures #-}
{-# LANGUAGE RankNTypes #-}
{-# LANGUAGE NamedFieldPuns #-}
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
import System.IO
import Data.Bifunctor

runRangerBluetooth :: HasCallStack => IO ()
runRangerBluetooth = do
  result <- runRangerGatt
  comms <- case result of
    Left err -> do
      hPutStrLn stderr "epic fail!"
      throwIO $ RangerBluetoothException err
    Right comms -> pure comms
  go comms
  where
    go comms = do
      result <- runEff . runConcurrent . runRangerControl $ runRangerProtocol comms
      case result of
        Just err -> do
          hPutStrLn stderr $ "Ranger protocol error: " ++ show err
          -- will freeze up until we get a new value in the queues/state is no longer poisoned
          (q1, q2) <- runEff . runConcurrent $ atomically $ do
            writeTVar (poisoned comms) True
            q1 <- flushTQueue (phoneToRanger comms)
            q2 <- flushTQueue (rangerToPhone comms)
            pure (q1, q2)
          let showSomeMsg (SomeMsg (msg, _)) = show msg
          hPutStrLn stderr $ "Rest of queues before flush:" ++ show (bimap (map (second showSomeMsg)) (map showSomeMsg) (q1, q2))
          runEff. runConcurrent . atomically $ do
            p <- readTVar (poisoned comms)
            check (not p)
          hPutStrLn stderr "Reset."
          go comms
        Nothing -> do
          hPutStrLn stderr "Done"
          pure ()

data RangerInvalidState = RangerDesync | RangerPacketIndexError deriving Show

newtype PacketIndex = PacketIndex Word8

-- | Runs Ranger's bluetooth protocol until failure or power off
runRangerProtocol
  :: (RangerControl :> es, Concurrent :> es)
  => RangerComms
  -> Eff es (Maybe RangerInvalidState)
runRangerProtocol comms = fmap (fmap snd . either Just (const Nothing))
                        . runError
                        . evalState (PacketIndex 0)
                        . unSide
                        $ runSync (itp comms) bluetoothProtocol

itp :: (RangerControl :> es, Error RangerInvalidState :> es, Concurrent :> es, State PacketIndex :> es) => RangerComms -> Interpreter 'Ranger Msg (SideM (Eff es) 'Ranger)
itp RangerComms{phoneToRanger, rangerToPhone} = Interpreter
  { side = SRanger
  , Control.Monad.Sync.send = \msg x -> SideM . atomically $ writeTQueue rangerToPhone (SomeMsg (msg, x))
  , recv = \_ SPhone msg -> SideM $ do
      (idx, SomeMsg (msg', val)) <- atomically $ readTQueue phoneToRanger
      PacketIndex idx' <- get
      when (idx /= idx') $ throwError RangerPacketIndexError -- TODO: handle out-of-order packets better
      val' <- case (msg, msg') of
        (Msg.FunctionCall, Msg.FunctionCall) -> pure val
        (Msg.FunctionCall, _) -> throwError RangerDesync
        (Msg.AnnounceSizeBytes, Msg.AnnounceSizeBytes) -> pure val
        (Msg.AnnounceSizeBytes, _) -> throwError RangerDesync
        (Msg.SendPhotoFragment, Msg.SendPhotoFragment) -> pure val
        (Msg.SendPhotoFragment, _) -> throwError RangerDesync
      modify $ const $ PacketIndex (idx + 1)
      pure val'
  }

runRangerControl :: (HasCallStack, IOE :> es) => Eff (RangerControl : es) a -> Eff es a
runRangerControl = interpret $ \_ -> \case
  StartDemo -> liftIO demoProcedure
  CancelDemo -> liftIO cancelDemoProcedure
  StartSearch oid params -> do
    liftIO $ putStrLn $ "placeholder: starting search with oid " ++ show oid ++ " and params " ++ show params
    pure True
  CancelSearch -> do
    liftIO $ putStrLn "placeholder: cancelling search"
    pure ()
  UpdateSearch params -> do
    liftIO $ putStrLn $ "placeholder: updating search with params " ++ show params
    pure ()
  SaveObject oid _photos -> do
    liftIO $ putStrLn $ "placeholder: pretending to save photos for " ++ show oid
    pure ()
  GetObjectPhotos oid -> do
    liftIO $ putStrLn $ "placeholder: pretending to have no photos for " ++ show oid
    pure mempty
  DeleteObject oid -> do
    liftIO $ putStrLn $ "placeholder: pretending to delete object " ++ show oid
    pure ()
  GetNotificationPhoto -> do
    liftIO $ putStrLn "placeholder: returning nothing as notification"
    pure mempty
  PowerOff -> do
    liftIO $ putStrLn "placeholder: failing to power off"
    pure False
