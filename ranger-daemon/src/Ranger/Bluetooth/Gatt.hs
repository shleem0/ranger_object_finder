{-# LANGUAGE OverloadedStrings #-}
{-# LANGUAGE DataKinds #-}
{-# LANGUAGE ExistentialQuantification #-}
{-# LANGUAGE NamedFieldPuns #-}
{-# LANGUAGE GADTs #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-# LANGUAGE BlockArguments #-}
{-# LANGUAGE LambdaCase #-}
-- TODO: use Effectful
module Ranger.Bluetooth.Gatt (runRangerGatt, RangerBluetoothException(..), RangerComms(..), SomeMsg(..)) where

import Bluetooth
import qualified Bluetooth.Internal.Types as Bluetooth
import Control.Concurrent.STM
import Ranger.Bluetooth.Msg
import Control.Monad.IO.Class
import Data.Word
import qualified Data.ByteString as B
import Data.Maybe
import Data.ByteString (ByteString)
import Ranger.Demo
import Control.Monad.Trans.Cont
import Control.Monad.Trans.Except
import Control.Concurrent.Async
import Control.Monad
import Control.Exception hiding (Handler)
import GHC.Stack
import System.IO

data SomeMsg s = forall a. SomeMsg (Msg s a, a)

-- | (poisoned, phoneToRanger, rangerToPhone)
--
-- setting 'poisoned' to true will cause nothing else to be enqueued on 'phoneToRanger'
-- until phone writes to the 'resetPoison' characteristic.

data RangerComms = RangerComms
  { phoneToRanger :: TQueue (Word8, SomeMsg 'Phone)
  , rangerToPhone :: TQueue (SomeMsg 'Ranger)
  , poisoned :: TVar Bool
  }

newtype RangerBluetoothException = RangerBluetoothException Bluetooth.Error deriving Show

instance Exception RangerBluetoothException

-- | WATCH OUT: will throw asynchronous 'RangerBluetoothException' if there is
-- an error post-initialisation. 
runRangerGatt :: HasCallStack => IO (Either Bluetooth.Error RangerComms)
runRangerGatt = evalContT . runExceptT $ do
  conn <- liftIO connect

  liftIO $ hPutStrLn stderr "Connected to DBus"

  phoneToRanger' <- liftIO newTQueueIO
  rangerToPhone <- liftIO newTQueueIO
  demoStateVar <- liftIO $ getDemoState >>= newTVarIO
  poisoned' <- liftIO $ newTVarIO False

  let state = RangerState { phoneToRanger'
                          , demoStateVar
                          , poisoned'
                          }
      comms = RangerComms { phoneToRanger = phoneToRanger'
                          , rangerToPhone
                          , poisoned = poisoned'
                          }

  liftIO $ hPutStrLn stderr "Registering GATT application..."

  registered <- ExceptT . liftIO $ runBluetoothM (registerAndAdvertiseApplication (app state)) conn

  liftIO $ hPutStrLn stderr "Registered GATT application"

  a <- liftIO . async $ (forever :: IO () -> IO a) $ do
    msg <- atomically $ readTQueue rangerToPhone
    -- TODO
    -- change characteristic values + trigger notifications according to Ranger's responses
    case msg of
      (SomeMsg (StartSearchResult, _)) -> pure ()
      (SomeMsg (AnnounceNFragments, _)) -> pure ()
      (SomeMsg (SendPhotoFragment, _)) -> pure ()
      (SomeMsg (RequestedObjectPhotoCount, _)) -> pure ()
      (SomeMsg (SchedulePowerOffResult, _)) -> pure ()

  b <- liftIO . async . forever $ waitDemoStateChange $ \b -> do
    atomically $ writeTVar demoStateVar b
    hPutStrLn stderr "b: demo state changed, notifying"
    result <- runBluetoothM (triggerNotification registered (demoState state)) conn
    case result of
      Right () -> pure ()
      Left err -> throwIO $ RangerBluetoothException err

  c <- liftIO . async . forever $ do
    atomically $ do
      p <- readTVar poisoned'
      check (not p)
    atomically $ do
      p <- readTVar poisoned'
      check p
    hPutStrLn stderr "c: poison detected, notifying"
    result <- runBluetoothM (triggerNotification registered (isPoisoned state)) conn
    case result of
      Right () -> pure ()
      Left err -> throwIO $ RangerBluetoothException err

  -- 'a/c' may get blocked if we forget about rangerToPhone, and we don't care
  liftIO $ linkOnly (\e -> isNothing (fromException e :: Maybe BlockedIndefinitelyOnSTM)) a
  liftIO $ linkOnly (\e -> isNothing (fromException e :: Maybe BlockedIndefinitelyOnSTM)) c
  liftIO $ link b

  pure comms

data RangerState = RangerState
  { phoneToRanger' :: TQueue (Word8, SomeMsg 'Phone)
  , demoStateVar :: TVar Bool
  , poisoned' :: TVar Bool
  }

app :: RangerState -> Application
app c = "/su/ranger/ranger_daemon" & services .~ [rangerService c]

rangerService :: RangerState -> Service 'Local
rangerService c = "fbb876fb-3ee3-5315-9716-01ede2358aab" & characteristics .~ [startDemo c, cancelDemo c, demoState c, isPoisoned c, resetPoison c]

-- | Write request where the first byte is an index for the message.
handleIndexedWrite :: (Word8 -> ByteString -> Handler Bool) -> ByteString -> Handler Bool
handleIndexedWrite f bs = case listToMaybe $ B.unpack bs of
  Just idx -> f idx (B.drop 1 bs)
  Nothing -> pure False

-- | Will not send anything if poisoned is True
sendMessage :: RangerState -> Word8 -> Msg 'Phone a -> a -> Handler Bool
sendMessage RangerState{poisoned', phoneToRanger'} idx msg a = liftIO . atomically $
  readTVar poisoned' >>= \case
    True -> pure False
    False -> writeTQueue phoneToRanger' (idx, SomeMsg (msg, a)) >> pure True

-- | Write-only.
--
-- Format: [Word8 message index] ++ arbitrary
--
-- Will fail when poisoned.
startDemo :: RangerState -> CharacteristicBS 'Local
startDemo s = "82e761bc-8508-5f80-90ee-9b3455444798"
  & properties .~ [CPWrite]
  & writeValue ?~ handleIndexedWrite (\idx _ -> sendMessage s idx FunctionCall StartDemo)

-- | Write-only.
-- Format: [Word8 message index] ++ arbitrary
--
-- Will fail when poisoned.
cancelDemo :: RangerState -> CharacteristicBS 'Local
cancelDemo s = "19a368f7-b27f-557b-81c5-be1130a406f5"
  & properties .~ [CPWrite]
  & writeValue ?~ handleIndexedWrite (\idx _ -> sendMessage s idx FunctionCall CancelDemo)

-- | Read-only.
--
-- Format: [0/1]
demoState :: RangerState -> CharacteristicBS 'Local
demoState RangerState{demoStateVar} = "a7926d00-c4b5-5335-98b6-17937cbb03f4"
  & properties .~ [CPRead, CPIndicate]
  & readValue ?~ encodeRead (liftIO $ readTVarIO demoStateVar)

-- | Read-only.
--
-- Format: [0/1]
--
-- If 1, then writes to characteristics other than 'resetPoison' will be ignored
-- until 'resetPoison' is written to, which will set this to 0.
isPoisoned :: RangerState -> CharacteristicBS 'Local
isPoisoned RangerState{poisoned'} = "286d24e6-5611-51b2-a2b3-6fb9d9aa9566"
  & properties .~ [CPRead, CPIndicate]
  & readValue ?~ encodeRead (liftIO $ readTVarIO poisoned')

-- | Write-only.
--
-- Format: anything
resetPoison :: RangerState -> CharacteristicBS 'Local
resetPoison RangerState{poisoned'} = "c0d915c8-26b1-50da-951e-d91bc4d3c5e1"
  & properties .~ [CPWrite]
  & writeValue ?~ (\_ -> do
      liftIO . atomically $ writeTVar poisoned' False
      pure True)