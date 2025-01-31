{-# LANGUAGE CPP, ForeignFunctionInterface, RecordWildCards #-}
module Ranger.Bluetooth.SimpleBLE.Types
  ( uuidStringLength, characteristicMaxCount, descriptorMaxCount, maxManufacturerData
  , SimpleBleResult, simpleBle_Success, simpleBle_Failure
  , SimpleBleUuid, mkSimpleBleUuid
  , SimpleBleDescriptor(..)
  , SimpleBleCharacteristic(..)
  , SimpleBleService(..)
  , SimpleBleAdapter(..), SimpleBlePeripheral(..)
  , SimpleBleOS, simpleBleOS_Linux, simpleBleOS_Windows, simpleBleOS_MacOS
  , SimpleBleAddressType, simpleBleAddressType_Public, simpleBleAddressType_Random, simpleBleAddressType_Unspecified
  ) where

import Foreign.C
import Foreign.Ptr
import GHC.Stack
import qualified Data.Text as T
import qualified Data.Text.Foreign as T
import Foreign.Storable
import Data.Vector.Storable (Vector)
import qualified Data.Vector.Storable as V

#include <simpleble_c/types.h>


uuidStringLength, characteristicMaxCount, descriptorMaxCount, maxManufacturerData :: Int
-- 37 originally, but that includes the null terminator
uuidStringLength = (#const SIMPLEBLE_UUID_STR_LEN) - 1
-- 16
characteristicMaxCount = #const SIMPLEBLE_CHARACTERISTIC_MAX_COUNT
-- 16
descriptorMaxCount = #const SIMPLEBLE_DESCRIPTOR_MAX_COUNT
maxManufacturerData = 27


newtype SimpleBleResult = SimpleBleResult Int deriving Eq

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
  -- ^ Maximum length is `descriptorMaxCount`
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
    let descPtr = #{ptr simpleble_characteristic_t, descriptors} ptr
    (descCount :: CSize) <- #{peek simpleble_characteristic_t, descriptor_count} ptr
    descriptors <- V.generateM (fromIntegral descCount) $ \i -> peekElemOff descPtr i
    return $ SimpleBleCharacteristic{..}
  poke ptr SimpleBleCharacteristic{..} = do
    #{poke simpleble_characteristic_t, uuid} ptr characteristicUuid
    #{poke simpleble_characteristic_t, can_read} ptr canRead
    #{poke simpleble_characteristic_t, can_write_request} ptr canWriteRequest
    #{poke simpleble_characteristic_t, can_write_command} ptr canWriteCommand
    #{poke simpleble_characteristic_t, can_notify} ptr canNotify
    #{poke simpleble_characteristic_t, can_indicate} ptr canIndicate
    let descCount = fromIntegral $ V.length descriptors :: CSize
    #{poke simpleble_characteristic_t, descriptor_count} ptr descCount
    V.iforM_ descriptors $ \i desc ->
      pokeElemOff (#{ptr simpleble_characteristic_t, descriptors} ptr) i desc


data SimpleBleService = SimpleBleService
  { serviceUuid :: SimpleBleUuid
  , manufacturerData :: Vector CUChar
  -- ^ Maximum length is `maxManufacturerData`
  , characteristics :: Vector SimpleBleCharacteristic
  -- ^ Maximum length is `characteristicMaxCount`
  }

instance Storable SimpleBleService where
  alignment _ = #alignment simpleble_service_t
  sizeOf _ = #size simpleble_service_t
  peek ptr = do
    serviceUuid <- #{peek simpleble_service_t, uuid} ptr

    let manufDataPtr = #{ptr simpleble_service_t, data} ptr
    (dataLength :: CSize) <- #{peek simpleble_service_t, data_length} ptr
    manufacturerData <- V.generateM (fromIntegral dataLength) $ \i -> peekElemOff manufDataPtr i

    let charPtr = #{ptr simpleble_service_t, characteristics} ptr
    (charCount :: CSize) <- #{peek simpleble_service_t, characteristic_count} ptr
    characteristics <- V.generateM (fromIntegral charCount) $ \i -> peekElemOff charPtr i

    return $ SimpleBleService{..}
  poke ptr SimpleBleService{..} = do
    #{poke simpleble_service_t, uuid} ptr serviceUuid

    let dataLength = fromIntegral $ V.length manufacturerData :: CSize
    #{poke simpleble_service_t, data_length} ptr dataLength

    V.iforM_ manufacturerData $ \i d ->
      pokeElemOff (#{ptr simpleble_service_t, data} ptr) i d

    let charCount = fromIntegral $ V.length characteristics :: CSize
    #{poke simpleble_service_t, characteristic_count} ptr charCount

    V.iforM_ characteristics $ \i c ->
      pokeElemOff (#{ptr simpleble_service_t, characteristics} ptr) i c


newtype SimpleBleAdapter = SimpleBleAdapter (FunPtr (IO ()))
newtype SimpleBlePeripheral = SimpleBlePeripheral (FunPtr (IO ()))


newtype SimpleBleOS = SimpleBleOS Int

#{enum SimpleBleOS, SimpleBleOS
 , simpleBleOS_Linux = SIMPLEBLE_OS_LINUX
 , simpleBleOS_Windows = SIMPLEBLE_OS_WINDOWS
 , simpleBleOS_MacOS = SIMPLEBLE_OS_MACOS
 }


newtype SimpleBleAddressType = SimpleBleAddressType Int

#{enum SimpleBleAddressType, SimpleBleAddressType
 , simpleBleAddressType_Public = SIMPLEBLE_ADDRESS_TYPE_PUBLIC
 , simpleBleAddressType_Random = SIMPLEBLE_ADDRESS_TYPE_RANDOM
 , simpleBleAddressType_Unspecified = SIMPLEBLE_ADDRESS_TYPE_UNSPECIFIED
 }
