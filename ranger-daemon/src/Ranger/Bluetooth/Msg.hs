{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE DataKinds #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE PolyKinds #-}
{-# LANGUAGE GADTs #-}
{-# LANGUAGE StandaloneKindSignatures #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE EmptyCase #-}

-- | Messages that we can send over Bluetooth.
--
-- In a separate module because old HLS versions shrivel up and die
-- with false conflicting instance errors when using singletons-2.7
--
-- You will probably have to restart HLS every time you change this file
-- (vscode: Ctrl+Shift+P -> "Haskell: Restart Haskell LSP server")
module Ranger.Bluetooth.Msg
  ( -- * Types
    Side(..)
  , FunctionCall(..)
  , Msg(..)
  , SearchParameters(..)
  , ObjectId(..)
  -- * Singletons
  , PhoneSym0
  , RangerSym0
  , SSide(..)
  , SMsg(..)
  ) where

import Data.Kind
import Data.Singletons
import Data.Singletons.TH
import Data.Text (Text)
import Data.Fixed

data SearchParameters = SearchParameters
  { timeout :: Maybe Int -- ^ seconds
  , withReturn :: Bool
  , maxRadius :: Maybe Centi -- ^ metres
  }

data Side = Ranger | Phone

-- | Short name for an object, maximum size of 16 bytes.
newtype ObjectId = ObjectId Text

-- | Any functions that may require some physical action from Ranger/prolonged
-- communication with the phone to execute.
--
-- Simple, read-only values like the battery level or the list of available
-- objects will just be exposed directly as GATT characteristics.
data FunctionCall
  = StartDemo
  | CancelDemo
  | StartSearch { objectId :: Text, searchParams :: SearchParameters }
  | ModifySearchParameters { searchParams :: SearchParameters }
  | CancelSearch
  | UpdateObject { objectId :: Text, photoCount :: Int }
  | DeleteObject { objectId :: Text }
  | GetObjectPhotos { objectId :: Text }
  | PowerOff

data Msg s a where
  FunctionCall :: FunctionCall -> Msg 'Phone FunctionCall

data SMsg (a :: Type) where
  SFunctionCall :: SMsg (Msg 'Phone FunctionCall)

$(genSingletons [''Side])
$(singDecideInstance ''Side)

type instance Sing = SMsg

instance SingI (Msg 'Phone FunctionCall) where
  sing = SFunctionCall
