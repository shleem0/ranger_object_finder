{-# LANGUAGE OverloadedStrings #-}
{-# LANGUAGE LambdaCase #-}
{-# LANGUAGE GADTs #-}
{-# LANGUAGE NamedFieldPuns #-}
{-# LANGUAGE DataKinds #-}
{-# LANGUAGE ScopedTypeVariables #-}
module Main (main) where

import Ranger.Bluetooth.Protocol
import Bluetooth
import System.Exit
import Data.List
import Control.Monad.IO.Class
import Control.Monad.Sync
import Ranger.Bluetooth.Msg
import Control.Concurrent.STM
import Ranger.Bluetooth.Types
import Data.Word
import Ranger.Effect
import Effectful
import Control.Concurrent.Async
import Control.Monad

main :: IO ()
main = do
  conn <- connect
  res <- runBluetoothM imgTransmit conn
  case res of
    Left e -> do
      putStrLn "Epic fail"
      exitFailure
    Right b -> do
      print b
      putStrLn "Done"

imgTransmit = do
  s <- getService "fbb876fb-3ee3-5315-9716-01ede2358aab" >>= maybe (liftIO $ fail "no") pure
  let cs = s ^. characteristics
      mupdateObjChar = find (\x -> x ^. uuid == "11eef0d8-a0ee-54c2-9afc-7aac51056c53") cs
      mfragChar = find (\x -> x ^. uuid == "4802d29c-63c2-56a8-ab25-2466b1503098") cs
  updateObjChar <- maybe (liftIO $ fail "no1") pure mupdateObjChar
  fragChar <- maybe (liftIO $ fail "no2") pure mfragChar
  (q :: (TQueue (Either FunctionCall (Either PhotoFragment Word16)))) <- liftIO newTQueueIO
  
  let itp :: Interpreter 'Phone Msg (SideM m 'Phone)
      itp = Interpreter
        { side = SPhone
        , send = \case
            FunctionCall -> SideM . liftIO . atomically . writeTQueue q . Left
            SendPhotoFragment -> SideM . liftIO . atomically . writeTQueue q . Right . Left
            AnnounceSizeBytes ->  SideM . liftIO . atomically . writeTQueue q . Right . Right
        , recv = undefined -- receiving nothing rn
        }
      test' :: Word16 -> RPSync s (Eff '[RangerControl]) Bool
      test' = test

  writer <- liftIO . async . forever $ do
    x <- atomically $ readTQueue q
    case x of
      Left (UpdateObject{objectId, photoCount}) -> undefined
      Right _ -> undefined
  
  liftIO $ link writer

  liftIO $ unSide $ runSync itp (test' 16)
