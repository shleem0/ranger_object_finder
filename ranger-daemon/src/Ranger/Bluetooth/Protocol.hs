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
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE NamedFieldPuns #-}
{-# LANGUAGE RankNTypes #-}
module Ranger.Bluetooth.Protocol (bluetoothProtocol, RPSync, SideM(..)) where

import Control.Monad.Sync
import Data.Singletons
import Ranger.Bluetooth.Msg
import Control.Monad.Trans.Reader
import Data.Type.Bool
import Data.Type.Equality
import Data.Void
import Ranger.Effect (RangerControl, startDemo, cancelDemo, startSearch, updateSearch, cancelSearch, saveObject, deleteObject, getObjectPhotos, getNotificationPhoto, powerOff)
import Control.Monad
import Effectful
import Ranger.Bluetooth.Types
import Data.ByteString (ByteString)
import Data.Vector (Vector)
import qualified Data.Vector as V
import Conduit
import Data.Word

-- | A monad that can never be executed.
--
-- Useful for writing pseudocode.
newtype VoidM a = VoidM (Reader Void a) deriving (Functor, Applicative, Monad)

pseudocode :: VoidM a
pseudocode = VoidM ask >>= absurd

-- | Monad that only exists if the side is 'Ranger'.
newtype SideM m (s :: Side) a = SideM { unSide :: If (s == 'Ranger) m VoidM a }

deriving instance Functor m => Functor (SideM m 'Ranger)
deriving instance Applicative m => Applicative (SideM m 'Ranger)
deriving instance Monad m => Monad (SideM m 'Ranger)
deriving instance Functor (SideM m 'Phone)
deriving instance Applicative (SideM m 'Phone)
deriving instance Monad (SideM m 'Phone)

-- | Ranger-Phone sync
type RPSync s m = Sync s Msg (SideM m s)

privateR :: m a -> RPSync s m (Private s 'Ranger a)
privateR a = private SRanger Proxy (SideM a)

privateP :: VoidM a -> RPSync s m (Private s 'Phone a)
privateP a = private SPhone Proxy (SideM a)

syncR :: Msg 'Ranger a -> (s ~ 'Ranger => m a) -> RPSync s m a
syncR msg a = sync SRanger msg (SideM a)

syncP :: Msg 'Phone a -> VoidM a -> RPSync s m a
syncP msg a = sync SPhone msg (SideM a)

-- | Phone calls function -> Phone & Ranger carry out the procedure defined in dispatchFunction -> Repeat until desync or successful power off
bluetoothProtocol :: RangerControl :> es => RPSync s (Eff es) ()
bluetoothProtocol = do
  runConduit
    $ repeatMC (syncP FunctionCall pseudocode)
    .| takeWhileC (\case PowerOff -> False; _ -> True)
    .| mapM_C dispatchFunction
  dispatchFunction PowerOff

dispatchFunction :: RangerControl :> es => FunctionCall -> RPSync s (Eff es) ()
dispatchFunction = \case
  StartDemo -> void $ privateR startDemo
  CancelDemo -> void $ privateR cancelDemo
  StartSearch{objectId, searchParams} -> do
    success <- syncR StartSearchResult $ startSearch objectId searchParams
    void $ privateP (pseudocode <*> pure success) -- phone does something with the result
  ModifySearchParameters p -> void $ private SRanger (Proxy :: Proxy ()) (SideM $ updateSearch p)
  CancelSearch -> void $ privateR cancelSearch
  UpdateObject{objectId, photoCount} -> do
    photos <- replicateM (fromIntegral photoCount) phoneUploadsPhoto
    void . privateR $ saveObject objectId photos
  DeleteObject oid -> void . privateR $ deleteObject oid
  GetObjectPhotos oid -> do
    photos <- privateR $ getObjectPhotos oid
    photoCount <- syncR RequestedObjectPhotoCount $ do
      let photos' = fromPrivate photos
          len = length photos'
      if len > fromIntegral (maxBound :: Word8)
        then -- semi-UB, must truncate photos list here, will realistically never happen
          pure (fromIntegral (maxBound :: Word8))
        else
          pure (fromIntegral len)
    photos' <- private SRanger (Proxy :: Proxy (Vector ByteString)) $ pure $ V.take (fromIntegral photoCount) (fromPrivate photos)
    photos'' <- V.generateM (fromIntegral photoCount) $ \i -> rangerUploadsPhoto (fmap (V.! i) photos')
    void . privateP $ pseudocode <*> pure photos'' -- phone does something with photos
  DownloadNotificationPhoto -> do
    photo <- privateR getNotificationPhoto
    photo' <- rangerUploadsPhoto photo
    void . privateP $ pseudocode <*> pure photo' -- phone does something with photo
  PowerOff -> do
    success <- syncR SchedulePowerOffResult powerOff
    void $ privateP (pseudocode <*> pure success) -- phone does something with the result

phoneUploadsPhoto :: RPSync s m ByteString
phoneUploadsPhoto = do
  nFragments <- syncP AnnounceNFragments pseudocode
  fmap fromFragments . replicateM nFragments $ syncP SendPhotoFragment pseudocode

rangerUploadsPhoto :: Applicative m => Private s 'Ranger ByteString -> RPSync s m ByteString
rangerUploadsPhoto bs = do
  fragments <- private SRanger (Proxy :: Proxy (Vector PhotoFragment)) $
    let bs' = fromPrivate bs
     in pure (toFragments bs')
  nFragments <- syncR AnnounceNFragments (pure $ V.length (fromPrivate fragments))
  fragments' <- V.generateM nFragments $ \i -> sync SRanger SendPhotoFragment $ pure (fromPrivate fragments V.! i)
  pure $ fromFragments (V.toList fragments')
