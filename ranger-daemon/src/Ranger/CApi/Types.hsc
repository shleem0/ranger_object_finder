{-# LANGUAGE CPP, ForeignFunctionInterface, TemplateHaskell, NamedFieldPuns, RecordWildCards, PatternSynonyms #-}
module Ranger.CApi.Types (ObjectId(..), SearchParameters(..), RangerCallbacks(..), RangerSignals(..)) where

import Foreign
import Foreign.C.Types
import Data.ByteString (ByteString)
import qualified Data.ByteString as B
import qualified Data.ByteString.Char8 as BC
import Data.Foldable
import Data.Traversable
import qualified Data.Serialize as S
import Control.Monad

#include <ranger_types.h>

pattern OBJECT_ID_LEN :: (Eq a, Num a) => a
pattern OBJECT_ID_LEN = #const OBJECT_ID_LEN

-- | 16-byte (8-bit chars/unicode code points 0-255)
-- name for the object, padded with '\NUL' at the end.
newtype ObjectId = ObjectId { idBytes :: ByteString } deriving Show

instance Storable ObjectId where
  alignment _ = #alignment object_id_t
  sizeOf _ = #size object_id_t
  peek ptr = fmap (ObjectId . B.pack) . for [0..OBJECT_ID_LEN - 1] $ \i ->
    peekByteOff (#{ptr object_id_t, value} ptr) i
  poke ptr (ObjectId{idBytes}) =
    for_ [0..OBJECT_ID_LEN - 1] $ \i ->
      pokeByteOff (#{ptr object_id_t, value} ptr) i (B.index idBytes i)

data SearchParameters = SearchParameters
  { timeout :: Word8
  , maxRadius :: Word16
  } deriving Show

instance Storable SearchParameters where
  alignment _ = #alignment search_parameters_t
  sizeOf _ = #size search_parameters_t
  peek ptr = do
    timeout <- #{peek search_parameters_t, timeout} ptr
    maxRadius <- #{peek search_parameters_t, max_radius} ptr
    pure SearchParameters{..}
  poke ptr (SearchParameters{timeout, maxRadius}) = do
    #{poke search_parameters_t, timeout} ptr timeout
    #{poke search_parameters_t, max_radius} ptr maxRadius

instance S.Serialize SearchParameters where
  put SearchParameters{timeout, maxRadius} = S.put timeout >> S.put maxRadius
  get = do
    timeout <- S.get
    maxRadius <- S.get
    pure SearchParameters{timeout, maxRadius}

instance S.Serialize ObjectId where
  put (ObjectId bs) = mapM_ S.put (BC.unpack bs)
  get = ObjectId . B.pack <$> replicateM 16 (S.get :: S.Get Word8)

data RangerCallbacks = RangerCallbacks
  { onDemoStart :: FunPtr (IO ())
  , onDemoCancel :: FunPtr (IO ())
  , onSearchStart :: FunPtr (Ptr ObjectId -> Ptr SearchParameters -> IO CBool)
  , onSearchCancel :: FunPtr (IO ())
  , onSearchUpdate :: FunPtr (Ptr SearchParameters -> IO ())
  , onUserPhotoResponse :: FunPtr (CBool -> IO ())
  }

instance Storable RangerCallbacks where
  alignment _ = #alignment ranger_callbacks_t
  sizeOf _ = #size ranger_callbacks_t
  peek ptr = do
    onDemoStart <- #{peek ranger_callbacks_t, on_demo_start} ptr
    onDemoCancel <- #{peek ranger_callbacks_t, on_demo_cancel} ptr
    onSearchStart <- #{peek ranger_callbacks_t, on_search_start} ptr
    onSearchCancel <- #{peek ranger_callbacks_t, on_search_cancel} ptr
    onSearchUpdate <- #{peek ranger_callbacks_t, on_search_update} ptr
    onUserPhotoResponse <- #{peek ranger_callbacks_t, on_user_photo_response} ptr
    pure RangerCallbacks{..}
  poke ptr (RangerCallbacks{..}) = do
    #{poke ranger_callbacks_t, on_demo_start} ptr onDemoStart
    #{poke ranger_callbacks_t, on_demo_cancel} ptr onDemoCancel
    #{poke ranger_callbacks_t, on_search_start} ptr onSearchStart
    #{poke ranger_callbacks_t, on_search_cancel} ptr onSearchCancel
    #{poke ranger_callbacks_t, on_search_update} ptr onSearchUpdate
    #{poke ranger_callbacks_t, on_user_photo_response} ptr onUserPhotoResponse

data RangerSignals = RangerSignals
 { demoEnded :: FunPtr (IO ())
 , photoPrompt :: FunPtr (Ptr ByteString -> IO ())
 , searchFailed :: FunPtr (IO ())
 , retrievingObject :: FunPtr (IO ())
 , objectReturnSuccess :: FunPtr (IO ())
 , objectReturnFail :: FunPtr (IO ())
 }

instance Storable RangerSignals where
  alignment _ = #alignment ranger_signals_t
  sizeOf _ = #size ranger_signals_t
  peek ptr = do
    demoEnded <- #{peek ranger_signals_t, demo_ended} ptr
    photoPrompt <- #{peek ranger_signals_t, photo_prompt} ptr
    searchFailed <- #{peek ranger_signals_t, search_failed} ptr
    retrievingObject <- #{peek ranger_signals_t, retrieving_object} ptr
    objectReturnSuccess <- #{peek ranger_signals_t, object_return_success} ptr
    objectReturnFail <- #{peek ranger_signals_t, object_return_fail} ptr
    pure RangerSignals{..}
  poke ptr (RangerSignals{..}) = do
    #{poke ranger_signals_t, demo_ended} ptr demoEnded
    #{poke ranger_signals_t, photo_prompt} ptr photoPrompt
    #{poke ranger_signals_t, search_failed} ptr searchFailed
    #{poke ranger_signals_t, retrieving_object} ptr retrievingObject
    #{poke ranger_signals_t, object_return_success} ptr objectReturnSuccess
    #{poke ranger_signals_t, object_return_fail} ptr objectReturnFail
