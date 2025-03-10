{-# LANGUAGE DataKinds #-}
{-# LANGUAGE EmptyDataDecls #-}
{-# LANGUAGE KindSignatures #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE GADTs #-}
{-# LANGUAGE TypeOperators #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RankNTypes #-}
{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE ScopedTypeVariables #-}
module Ranger.Effect
  ( RangerControl(..)
  , ObjectId(..)
  , SearchParameters(..)
  , startDemo
  , cancelDemo
  , startSearch
  , cancelSearch
  , updateSearch
  , saveObject
  , getObjectPhotos
  , deleteObject
  , powerOff
  , getNotificationPhoto
  ) where
import Effectful
import Effectful.TH
import Data.ByteString (ByteString)
import Ranger.Types
import Data.Vector (Vector)

data RangerControl :: Effect where
  StartDemo :: RangerControl m ()
  CancelDemo :: RangerControl m ()

  StartSearch :: ObjectId -> SearchParameters -> RangerControl m Bool
    -- ^ start search with the given parameters, returns False if we don't have
    -- photos associated with the object
  CancelSearch :: RangerControl m ()
    -- ^ Immediately cancel an ongoing search.
  UpdateSearch :: SearchParameters -> RangerControl m ()
    -- ^ Update an ongoing search with the given parameters.
  SaveObject :: ObjectId -> [ByteString] -> RangerControl m ()
    -- ^ save the object with the given photos
  GetObjectPhotos :: ObjectId -> RangerControl m (Vector ByteString)
    -- ^ get the photos associated with the object.
    -- The list is empty if and only if there is no such object.
  DeleteObject :: ObjectId -> RangerControl m ()
    -- ^ delete the object and its photos
  GetNotificationPhoto :: RangerControl m ByteString
    -- ^ get the photo that the ROS nodes want the phone to have
  PowerOff :: RangerControl m Bool
    -- ^ power off the device, returns false if something goes wrong with scheduling shutdown

$(makeEffect ''RangerControl)
