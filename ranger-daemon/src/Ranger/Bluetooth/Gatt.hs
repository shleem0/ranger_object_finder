{-# LANGUAGE OverloadedStrings #-}
{-# LANGUAGE DataKinds #-}
{-# LANGUAGE ExistentialQuantification #-}
{-# LANGUAGE NamedFieldPuns #-}
{-# LANGUAGE GADTs #-}
{-# LANGUAGE ScopedTypeVariables #-}
-- TODO: use Effectful
module Ranger.Bluetooth.Gatt (runRangerGatt, RangerBluetoothException(..), RangerComms, SomeMsg(..)) where

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
import Control.Monad.Trans.Class
import Control.Monad.Trans.Except
import Control.Concurrent.Async
import Control.Monad
import Control.Exception hiding (Handler)

data SomeMsg s = forall a. SomeMsg (Msg s a, a)

type RangerComms = (TQueue (Word8, SomeMsg 'Phone), TQueue (SomeMsg 'Ranger))

newtype RangerBluetoothException = RangerBluetoothException Bluetooth.Error deriving Show

instance Exception RangerBluetoothException

-- | WATCH OUT: will throw asynchronous 'RangerBluetoothException' if there is
-- an error post-initialisation. 
runRangerGatt :: IO (Either Bluetooth.Error RangerComms)
runRangerGatt = evalContT . runExceptT $ do
  conn <- liftIO connect

  phoneToRanger <- liftIO newTQueueIO
  rangerToPhone <- liftIO newTQueueIO
  demoStateVar <- liftIO $ getDemoState >>= newTVarIO

  let state = RangerState { phoneToRanger
                          , demoStateVar
                          }

  registered <- ExceptT . lift $ runBluetoothM (registerAndAdvertiseApplication (app state)) conn

  a <- liftIO . async $ (forever :: IO () -> IO a) $ do
    msg <- atomically $ readTQueue rangerToPhone
    -- TODO
    -- change characteristic values + triggers notifications according to the message
    case msg of
      (SomeMsg (StartSearchResult, _)) -> pure ()
      (SomeMsg (AnnounceNFragments, _)) -> pure ()
      (SomeMsg (SendPhotoFragment, _)) -> pure ()
      (SomeMsg (RequestedObjectPhotoCount, _)) -> pure ()
      (SomeMsg (SchedulePowerOffResult, _)) -> pure ()

  b <- liftIO . async . forever $ waitDemoStateChange $ \b -> do
    atomically $ writeTVar demoStateVar b
    result <- runBluetoothM (triggerNotification registered (demoState state)) conn
    case result of
      Right () -> pure ()
      Left err -> throwIO $ RangerBluetoothException err

  -- 'a' may get blocked if we forget about rangerToPhone, and we don't care
  liftIO $ linkOnly (\e -> isNothing (fromException e :: Maybe BlockedIndefinitelyOnSTM)) a
  liftIO $ link b

  pure (phoneToRanger, rangerToPhone)

data RangerState = RangerState
  { phoneToRanger :: TQueue (Word8, SomeMsg 'Phone)
  , demoStateVar :: TVar Bool
  }

app :: RangerState -> Application
app c = "/su/ranger/ranger-daemon" & services .~ [demoService c]

demoService :: RangerState -> Service 'Local
demoService c = "fbb876fb-3ee3-5315-9716-01ede2358aab" & characteristics .~ [startDemo c, cancelDemo c, demoState c]

-- | Write request where the first byte is an index for the message.
handleIndexedWrite :: (Word8 -> ByteString -> Handler Bool) -> ByteString -> Handler Bool
handleIndexedWrite f bs = case listToMaybe $ B.unpack bs of
  Just idx -> f idx (B.drop 1 bs)
  Nothing -> pure False

-- | Write-only.
-- Format: [Word8 message index] ++ arbitrary
startDemo :: RangerState -> CharacteristicBS 'Local
startDemo RangerState{phoneToRanger} = "82e761bc-8508-5f80-90ee-9b3455444798"
  & properties .~ [CPWrite, CPWriteWithoutResponse]
  & writeValue ?~ handleIndexedWrite (\idx _ -> do
      liftIO . atomically $ writeTQueue phoneToRanger (idx, SomeMsg (FunctionCall, StartDemo))
      pure True)

-- | Write-only.
-- Format: [Word8 message index] ++ arbitrary
cancelDemo :: RangerState -> CharacteristicBS 'Local
cancelDemo RangerState{phoneToRanger} = "19a368f7-b27f-557b-81c5-be1130a406f5"
  & properties .~ [CPWrite, CPWriteWithoutResponse]
  & writeValue ?~ handleIndexedWrite (\idx _ -> do
      liftIO . atomically $ writeTQueue phoneToRanger (idx, SomeMsg (FunctionCall, CancelDemo))
      pure True)

-- | Read-only.
-- Format: [0/1]
demoState :: RangerState -> CharacteristicBS 'Local
demoState RangerState{demoStateVar} = "a7926d00-c4b5-5335-98b6-17937cbb03f4"
  & properties .~ [CPRead]
  & readValue ?~ encodeRead (liftIO $ readTVarIO demoStateVar)