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
import Ranger.Bluetooth.Types (PhotoFragment)

data SomeMsg s = forall a. SomeMsg (Msg s a, a)

-- | setting 'poisoned' to true will cause nothing else to be enqueued on 'phoneToRanger'
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
  startSearchResponse <- liftIO $ newTVarIO Nothing
  nBytesFromRanger <- liftIO $ newTVarIO Nothing
  photoFragmentFromRanger <- liftIO $ newTVarIO Nothing
  requestedPhotoCount <- liftIO $ newTVarIO Nothing
  powerOffResult <- liftIO $ newTVarIO Nothing

  let state = RangerState { phoneToRanger'
                          , demoStateVar
                          , poisoned'
                          , startSearchResponse
                          , nBytesFromRanger
                          , photoFragmentFromRanger
                          , requestedPhotoCount
                          , powerOffResult
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
    result <- flip runBluetoothM conn $ case msg of
      (SomeMsg (StartSearchResult, b)) -> do
        liftIO . atomically $ writeTVar startSearchResponse (Just b)
        triggerNotification registered (startSearchResult state)
      (SomeMsg (AnnounceSizeBytes, i)) -> do
        liftIO . atomically $ writeTVar nBytesFromRanger (Just i)
        triggerNotification registered (announceSizeBytes state)
      (SomeMsg (SendPhotoFragment, p)) -> do
        liftIO . atomically $ writeTVar photoFragmentFromRanger (Just p)
        triggerNotification registered (photoFragment state)
      (SomeMsg (RequestedObjectPhotoCount, c)) -> do
        liftIO . atomically $ writeTVar requestedPhotoCount (Just c)
        triggerNotification registered (requestedObjectPhotoCount state)
      (SomeMsg (SchedulePowerOffResult, r)) -> do
        liftIO . atomically $ writeTVar powerOffResult (Just r)
        triggerNotification registered (schedulePowerOffResult state)
    case result of
      Right () -> pure ()
      Left err -> throwIO $ RangerBluetoothException err

  b <- liftIO . async . forever $ waitDemoStateChange $ \b -> do
    atomically $ writeTVar demoStateVar b
    result <- runBluetoothM (triggerNotification registered (demoState state)) conn
    case result of
      Right () -> pure ()
      Left err -> throwIO $ RangerBluetoothException err

  c <- liftIO . async . forever $ do
    p1 <- readTVarIO poisoned'
    atomically $ do
      p2 <- readTVar poisoned'
      check (p1 /= p2)
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
  , startSearchResponse :: TVar (Maybe Bool)
  , nBytesFromRanger :: TVar (Maybe Word16)
  , photoFragmentFromRanger :: TVar (Maybe PhotoFragment)
  , requestedPhotoCount :: TVar (Maybe Word8)
  , powerOffResult :: TVar (Maybe Bool)
  }

app :: RangerState -> Application
app c = "/su/ranger/ranger_daemon" & services .~ [rangerService c]

rangerService :: RangerState -> Service 'Local
rangerService c = "fbb876fb-3ee3-5315-9716-01ede2358aab" & characteristics .~
  [ startDemo c, cancelDemo c, demoState c, isPoisoned c, resetPoison c
  , startSearch c, modifySearchParams c, cancelSearch c, updateObject c
  , deleteObject c, getObjectPhotos c, downloadNotificationPhoto c, poweroff c
  , startSearchResult c, announceSizeBytes c, photoFragment c
  , requestedObjectPhotoCount c, schedulePowerOffResult c
  ]

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

-- | Write-only. Function call.
--
-- Format: [Word8 message index] ++ arbitrary
--
-- Will fail when poisoned.
startDemo :: RangerState -> CharacteristicBS 'Local
startDemo s = "82e761bc-8508-5f80-90ee-9b3455444798"
  & properties .~ [CPWrite]
  & writeValue ?~ handleIndexedWrite (\idx _ -> sendMessage s idx FunctionCall StartDemo)

-- | Write-only. Function call.
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

-- | Write-only. Not a function call, but can be written to at any time.
--
-- Format: anything
resetPoison :: RangerState -> CharacteristicBS 'Local
resetPoison RangerState{poisoned'} = "c0d915c8-26b1-50da-951e-d91bc4d3c5e1"
  & properties .~ [CPWrite]
  & writeValue ?~ (\_ -> do
      liftIO . atomically $ writeTVar poisoned' False
      pure True)

-- | Write-only. Function call.
--
-- Format: Word8 message index ++ Object id (16 bytes) ++ Search parameters (3 bytes)
startSearch :: RangerState -> CharacteristicBS 'Local
startSearch s = "4fc605c8-c03f-59df-8f25-a68b2b3de548"
  & properties .~ [CPWrite]
  & writeValue ?~ handleIndexedWrite (\idx ->
    encodeWrite (sendMessage s idx FunctionCall . uncurry StartSearch)
  )

-- | Write-only. Function call.
--
-- Format: Word8 message index ++ Search parameters (3 bytes)
modifySearchParams :: RangerState -> CharacteristicBS 'Local
modifySearchParams s = "1ee2c3df-769d-5611-998b-d9f2dc22207a"
  & properties .~ [CPWrite]
  & writeValue ?~ handleIndexedWrite (\idx ->
    encodeWrite (sendMessage s idx FunctionCall . ModifySearchParameters)
  )

-- | Write-only. Function call.
--
-- Format: Word8 message index ++ arbitrary
cancelSearch :: RangerState -> CharacteristicBS 'Local
cancelSearch s = "795aa39e-a468-56e4-a501-22bfe3f2c51a"
  & properties .~ [CPWrite]
  & writeValue ?~ handleIndexedWrite (\idx _ -> sendMessage s idx FunctionCall CancelSearch)

-- | Write-only. Function call.
--
-- Format: Word8 message index ++ Object id (16 bytes) ++ Photo count (1 byte)
updateObject :: RangerState -> CharacteristicBS 'Local
updateObject s = "11eef0d8-a0ee-54c2-9afc-7aac51056c53"
  & properties .~ [CPWrite]
  & writeValue ?~ handleIndexedWrite (\idx ->
    encodeWrite (sendMessage s idx FunctionCall . uncurry UpdateObject)
  )

-- | Write-only. Function call.
--
-- Format: Word8 message index ++ Object id (16 bytes)
deleteObject :: RangerState -> CharacteristicBS 'Local
deleteObject s = "b98a3be5-acb7-519c-b15a-b962b617d2dc"
  & properties .~ [CPWrite]
  & writeValue ?~ handleIndexedWrite (\idx ->
    encodeWrite (sendMessage s idx FunctionCall . DeleteObject)
  )

-- | Write-only. Function call.
--
-- Format: Word8 message index ++ object id (16 bytes)
getObjectPhotos :: RangerState -> CharacteristicBS 'Local
getObjectPhotos s = "89360a5a-9bdd-584a-acdc-f141a0e3b46c"
  & properties .~ [CPWrite]
  & writeValue ?~ handleIndexedWrite (\idx ->
    encodeWrite (sendMessage s idx FunctionCall . GetObjectPhotos)
  )

-- | Write-only. Function call.
--
-- Format: Word8 message index ++ arbitrary
downloadNotificationPhoto :: RangerState -> CharacteristicBS 'Local
downloadNotificationPhoto s = "f0fce81f-4128-563f-9842-bde2a490009f"
  & properties .~ [CPWrite]
  & writeValue ?~ handleIndexedWrite (\idx _ -> sendMessage s idx FunctionCall DownloadNotificationPhoto)

-- | Write-only. Function call.
--
-- Format: Word8 message index ++ arbitrary
poweroff :: RangerState -> CharacteristicBS 'Local
poweroff s = "695aceb7-9558-5293-95ff-cfef7e25b193"
  & properties .~ [CPWrite]
  & writeValue ?~ handleIndexedWrite (\idx _ -> sendMessage s idx FunctionCall PowerOff)

-- | Read-only.
--
-- Format: [0/1]
startSearchResult :: RangerState -> CharacteristicBS 'Local
startSearchResult RangerState{startSearchResponse} = "dad9f4f1-23e5-56c6-b324-dff8983dba83"
  & properties .~ [CPRead, CPIndicate]
  & readValue ?~ encodeRead (liftIO $ readTVarIO startSearchResponse)

-- | Read-write. Not a function call, so cannot be written to arbitrarily.
--
-- Read format: Word16
--
-- Write format: Word8 message index ++ Word16
announceSizeBytes :: RangerState -> CharacteristicBS 'Local
announceSizeBytes s@RangerState{nBytesFromRanger} = "8a091a09-037b-5916-9d47-f1083ded62e4"
  & properties .~ [CPWrite, CPRead, CPIndicate]
  & readValue ?~ encodeRead (liftIO $ readTVarIO nBytesFromRanger)
  & writeValue ?~ handleIndexedWrite (\idx -> encodeWrite (sendMessage s idx AnnounceSizeBytes))

-- | Read-write. Not a function call, so cannot be written to arbitrarily.
--
-- Read format: 19-byte photo fragment
--
-- Write format: Word8 message index ++ 19-byte photo fragment
photoFragment :: RangerState -> CharacteristicBS 'Local
photoFragment s@RangerState{photoFragmentFromRanger} = "4802d29c-63c2-56a8-ab25-2466b1503098"
  & properties .~ [CPWrite, CPRead, CPIndicate]
  & readValue ?~ encodeRead (liftIO $ readTVarIO photoFragmentFromRanger)
  & writeValue ?~ handleIndexedWrite (\idx -> encodeWrite (sendMessage s idx SendPhotoFragment))

-- | Read-only.
--
-- Format: Word8
requestedObjectPhotoCount :: RangerState -> CharacteristicBS 'Local
requestedObjectPhotoCount RangerState{requestedPhotoCount} = "1d0aeed1-7afe-506f-804c-866f384cc10d"
  & properties .~ [CPRead, CPIndicate]
  & readValue ?~ encodeRead (liftIO $ readTVarIO requestedPhotoCount)

-- | Read-only.
--
-- Format: [0/1]
schedulePowerOffResult :: RangerState -> CharacteristicBS 'Local
schedulePowerOffResult RangerState{powerOffResult} = "4906c5b7-82bf-5379-87a4-f4edaff16118"
  & properties .~ [CPRead, CPIndicate]
  & readValue ?~ encodeRead (liftIO $ readTVarIO powerOffResult)