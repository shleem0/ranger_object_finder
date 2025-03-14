{-# LANGUAGE DataKinds #-}
{-# LANGUAGE GADTs #-}
{-# LANGUAGE StandaloneDeriving #-}

-- | Messages that we can send over Bluetooth.
module Ranger.Bluetooth.Msg
  ( -- * Types
    Side(..)
  , FunctionCall(..)
  , Msg(..)
  -- * Singletons
  , PhoneSym0
  , RangerSym0
  , SSide(..)
  ) where

import Ranger.Bluetooth.Types
import Data.Word

-- | Any functions that may require some physical action from Ranger/prolonged
-- communication with the phone to execute.
--
-- Simple, read-only values like the battery level or the list of available
-- objects will just be exposed directly as GATT characteristics.
data FunctionCall
  = StartDemo
  | CancelDemo
  | StartSearch { objectId :: ObjectId, searchParams :: SearchParameters }
  | ModifySearchParameters { searchParams :: SearchParameters }
  | CancelSearch
  | UpdateObject { objectId :: ObjectId, photoCount :: Word8 }
  | DeleteObject { objectId :: ObjectId }
  | GetObjectPhotos { objectId :: ObjectId }
  | DownloadNotificationPhoto
  | PowerOff

data Msg s a where
  FunctionCall :: Msg 'Phone FunctionCall
  StartSearchResult :: Msg 'Ranger Bool
  AnnounceSizeBytes :: Msg s Int
  SendPhotoFragment :: Msg s PhotoFragment 
  RequestedObjectPhotoCount :: Msg 'Ranger Word8
  SchedulePowerOffResult :: Msg 'Ranger Bool

deriving instance Eq (Msg s a)
deriving instance Show (Msg s a)
