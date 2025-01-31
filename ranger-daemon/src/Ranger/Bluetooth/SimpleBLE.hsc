{-# LANGUAGE CPP, ForeignFunctionInterface, RecordWildCards #-}
-- | SimpleBLE library bindings
module Ranger.Bluetooth.SimpleBLE where

import Foreign.C
import Foreign.Ptr
import GHC.Stack
import qualified Data.Text as T
import qualified Data.Text.Foreign as T
import Foreign.Storable
import Data.Vector.Storable (Vector)
import qualified Data.Vector.Storable as V

#include <simpleble_c/simpleble.h>


--------------------------------------------------------------------------------
-- types.h
--------------------------------------------------------------------------------


uuidStringLength, characteristicMaxCount, descriptorMaxCount :: Int
-- 37 originally, but that includes the null terminator
uuidStringLength = (#const SIMPLEBLE_UUID_STR_LEN) - 1
-- 16
characteristicMaxCount = #const SIMPLEBLE_CHARACTERISTIC_MAX_COUNT
-- 16
descriptorMaxCount = #const SIMPLEBLE_DESCRIPTOR_MAX_COUNT


newtype SimpleBleResult = SimpleBleResult CInt deriving Eq

#{enum SimpleBleResult, SimpleBleResult
 , simpleBle_Success = SIMPLEBLE_SUCCESS
 , simpleBle_Failure = SIMPLEBLE_FAILURE
 }


newtype SimpleBleUuid = SimpleBleUuid T.Text deriving Eq

-- | Length of the UUID string in bytes must equal `simpleBleUuidStringLength`: throws otherwise
mkSimpleBleUuid :: HasCallStack => T.Text -> SimpleBleUuid
mkSimpleBleUuid t
  | T.lengthWord8 t == uuidStringLength = SimpleBleUuid t
  | otherwise = error "SimpleBleUuid: invalid length"

instance Storable SimpleBleUuid where
  alignment _ = #alignment simpleble_uuid_t
  sizeOf _ = #size simpleble_uuid_t
  peek ptr =
    let valPtr = (#{ptr simpleble_uuid_t, value} ptr)
    in SimpleBleUuid <$> T.fromPtr0 valPtr
  poke ptr (SimpleBleUuid t) = T.withCString t $ \cstr ->
    #{poke simpleble_uuid_t, value} ptr cstr


newtype SimpleBleDescriptor = SimpleBleDescriptor SimpleBleUuid deriving Eq

instance Storable SimpleBleDescriptor where
  alignment _ = #alignment simpleble_descriptor_t
  sizeOf _ = #size simpleble_descriptor_t
  peek ptr = SimpleBleDescriptor <$> #{peek simpleble_descriptor_t, uuid} ptr
  poke ptr (SimpleBleDescriptor u) = #{poke simpleble_descriptor_t, uuid} ptr u


data SimpleBleCharacteristic = SimpleBleCharacteristic
  { characteristicUuid :: SimpleBleUuid
  , canRead :: CBool
  , canWriteRequest :: CBool
  , canWriteCommand :: CBool
  , canNotify :: CBool
  , canIndicate :: CBool
  , descriptors :: Vector SimpleBleDescriptor
  }

instance Storable SimpleBleCharacteristic where
  alignment _ = #alignment simpleble_characteristic_t
  sizeOf _ = #size simpleble_characteristic_t
  peek ptr = do
    characteristicUuid <- #{peek simpleble_characteristic_t, uuid} ptr
    canRead <- #{peek simpleble_characteristic_t, can_read} ptr
    canWriteRequest <- #{peek simpleble_characteristic_t, can_write_request} ptr
    canWriteCommand <- #{peek simpleble_characteristic_t, can_write_command} ptr
    canNotify <- #{peek simpleble_characteristic_t, can_notify} ptr
    canIndicate <- #{peek simpleble_characteristic_t, can_indicate} ptr
    let descPtr = (#{ptr simpleble_characteristic_t, descriptors} ptr)
    descCount <- #{peek simpleble_characteristic_t, descriptor_count} ptr
    descriptors <- V.generateM descCount $ \i -> peekElemOff descPtr i
    return $ SimpleBleCharacteristic{..}
  poke ptr SimpleBleCharacteristic{..} = do
    #{poke simpleble_characteristic_t, uuid} ptr characteristicUuid
    #{poke simpleble_characteristic_t, can_read} ptr canRead
    #{poke simpleble_characteristic_t, can_write_request} ptr canWriteRequest
    #{poke simpleble_characteristic_t, can_write_command} ptr canWriteCommand
    #{poke simpleble_characteristic_t, can_notify} ptr canNotify
    #{poke simpleble_characteristic_t, can_indicate} ptr canIndicate
    let descCount = V.length descriptors
    #{poke simpleble_characteristic_t, descriptor_count} ptr descCount
    V.iforM_ descriptors $ \i desc ->
      pokeElemOff (#{ptr simpleble_characteristic_t, descriptors} ptr) i desc
