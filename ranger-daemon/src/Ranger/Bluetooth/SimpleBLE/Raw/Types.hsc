{-# LANGUAGE CPP, ForeignFunctionInterface, RecordWildCards, LambdaCase, PatternSynonyms, TemplateHaskell #-}
module Ranger.Bluetooth.SimpleBLE.Raw.Types
  ( pattern UUID_STR_LENGTH, pattern SIMPLEBLE_CHARACTERISTIC_MAX_COUNT, pattern SIMPLEBLE_DESCRIPTOR_MAX_COUNT, pattern MAX_MANUFACTURER_DATA_BYTES
  , SimpleBleResult(..), pattern SIMPLEBLE_SUCCESS, pattern SIMPLEBLE_FAILURE
  , SimpleBleUuid(..), mkSimpleBleUuid
  , SimpleBleDescriptor(..)
  , SimpleBleCharacteristic(..)
  , SimpleBleService(..)
  , SimpleBleAdapter(..), SimpleBlePeripheral(..)
  , SimpleBleManufacturerData(..)
  , SimpleBleOS(..), pattern SIMPLEBLE_OS_LINUX, pattern SIMPLEBLE_OS_WINDOWS, pattern SIMPLEBLE_OS_MACOS
  , SimpleBleAddressType(..), pattern SIMPLEBLE_ADDRESS_TYPE_PUBLIC, pattern SIMPLEBLE_ADDRESS_TYPE_RANDOM, pattern SIMPLEBLE_ADDRESS_TYPE_UNSPECIFIED
  ) where

import Foreign.C
import Foreign.Ptr
import GHC.Stack
import qualified Data.Text as T
import qualified Data.Text.Foreign as T
import Foreign.Storable
import Data.Vector.Storable (Vector)
import qualified Data.Vector.Storable as V
import Language.Haskell.TH
import Language.Haskell.TH.Syntax

#include <simpleble_c/types.h>


pattern UUID_STR_LENGTH, SIMPLEBLE_CHARACTERISTIC_MAX_COUNT, SIMPLEBLE_DESCRIPTOR_MAX_COUNT, MAX_MANUFACTURER_DATA_BYTES :: Int
-- | 37 originally, but that includes the null terminator, so we subtract 1
pattern UUID_STR_LENGTH = $(fmap (\case (LitE l) -> LitP l; _ -> error "not a lit") . lift $ (#const SIMPLEBLE_UUID_STR_LEN) - (1 :: Int))
-- | 16
pattern SIMPLEBLE_CHARACTERISTIC_MAX_COUNT = #const SIMPLEBLE_CHARACTERISTIC_MAX_COUNT
-- | 16
pattern SIMPLEBLE_DESCRIPTOR_MAX_COUNT = #const SIMPLEBLE_DESCRIPTOR_MAX_COUNT
-- | 27
pattern MAX_MANUFACTURER_DATA_BYTES = 27


newtype SimpleBleResult = SimpleBleResult CInt deriving (Eq, Ord)

pattern SIMPLEBLE_SUCCESS :: SimpleBleResult
pattern SIMPLEBLE_SUCCESS = SimpleBleResult (#const SIMPLEBLE_SUCCESS)
pattern SIMPLEBLE_FAILURE :: SimpleBleResult
pattern SIMPLEBLE_FAILURE = SimpleBleResult (#const SIMPLEBLE_FAILURE)
{-# COMPLETE SIMPLEBLE_SUCCESS, SIMPLEBLE_FAILURE #-}

instance Show SimpleBleResult where
  show SIMPLEBLE_SUCCESS = "SIMPLEBLE_SUCCESS"
  show SIMPLEBLE_FAILURE = "SIMPLEBLE_FAILURE"

instance Storable SimpleBleResult where
  alignment _ = #alignment simpleble_err_t
  sizeOf _ = #size simpleble_err_t
  peek ptr = SimpleBleResult <$> peek (castPtr ptr)
  poke ptr (SimpleBleResult x) = poke (castPtr ptr :: Ptr CInt) x


newtype SimpleBleUuid = SimpleBleUuid T.Text deriving (Eq, Show)

-- | Length of the UUID string in bytes must equal `UUID_STR_LENGTH`: throws otherwise
mkSimpleBleUuid :: HasCallStack => T.Text -> SimpleBleUuid
mkSimpleBleUuid t
  | T.lengthWord8 t == UUID_STR_LENGTH = SimpleBleUuid t
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
  -- ^ Maximum length is `SIMPLEBLE_DESCRIPTOR_MAX_COUNT`, anything extra is UB
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
  , serviceManufacturerData :: Vector CUChar
  -- ^ Maximum length is `MAX_MANUFACTURER_DATA_BYTES`, anything extra is UB
  , characteristics :: Vector SimpleBleCharacteristic
  -- ^ Maximum length is `SIMPLEBLE_CHARACTERISTIC_MAX_COUNT`, anything extra is UB
  }

instance Storable SimpleBleService where
  alignment _ = #alignment simpleble_service_t
  sizeOf _ = #size simpleble_service_t
  peek ptr = do
    serviceUuid <- #{peek simpleble_service_t, uuid} ptr

    let manufDataPtr = #{ptr simpleble_service_t, data} ptr
    (dataLength :: CSize) <- #{peek simpleble_service_t, data_length} ptr
    serviceManufacturerData <- V.generateM (fromIntegral dataLength) $ \i -> peekElemOff manufDataPtr i

    let charPtr = #{ptr simpleble_service_t, characteristics} ptr
    (charCount :: CSize) <- #{peek simpleble_service_t, characteristic_count} ptr
    characteristics <- V.generateM (fromIntegral charCount) $ \i -> peekElemOff charPtr i

    return $ SimpleBleService{..}
  poke ptr SimpleBleService{..} = do
    #{poke simpleble_service_t, uuid} ptr serviceUuid

    let dataLength = fromIntegral $ V.length serviceManufacturerData :: CSize
    #{poke simpleble_service_t, data_length} ptr dataLength

    V.iforM_ serviceManufacturerData $ \i d ->
      pokeElemOff (#{ptr simpleble_service_t, data} ptr) i d

    let charCount = fromIntegral $ V.length characteristics :: CSize
    #{poke simpleble_service_t, characteristic_count} ptr charCount

    V.iforM_ characteristics $ \i c ->
      pokeElemOff (#{ptr simpleble_service_t, characteristics} ptr) i c


data SimpleBleManufacturerData = SimpleBleManufacturerData
  { manufacturerId :: CUShort
  , manufacturerData :: Vector CUChar
  -- ^ Maximum length is `MAX_MANUFACTURER_DATA_BYTES`, anything extra is UB
  }


instance Storable SimpleBleManufacturerData where
  alignment _ = #alignment simpleble_manufacturer_data_t
  sizeOf _ = #size simpleble_manufacturer_data_t
  peek ptr = do
    manufacturerId <- #{peek simpleble_manufacturer_data_t, manufacturer_id} ptr
    (dataLength :: CSize) <- #{peek simpleble_manufacturer_data_t, data_length} ptr
    let dataPtr = #{ptr simpleble_manufacturer_data_t, data} ptr
    manufacturerData <- V.generateM (fromIntegral dataLength) $ \i -> peekElemOff dataPtr i

    pure SimpleBleManufacturerData{..}

  poke ptr SimpleBleManufacturerData{..} = do
    #{poke simpleble_manufacturer_data_t, manufacturer_id} ptr manufacturerId
    let dataLength = fromIntegral $ V.length manufacturerData :: CSize
    #{poke simpleble_manufacturer_data_t, data_length} ptr dataLength
    V.iforM_ manufacturerData $ \i d ->
      pokeElemOff (#{ptr simpleble_manufacturer_data_t, data} ptr) i d


newtype SimpleBleAdapter = SimpleBleAdapter (FunPtr (IO ()))
newtype SimpleBlePeripheral = SimpleBlePeripheral (FunPtr (IO ()))


newtype SimpleBleOS = SimpleBleOS CInt deriving (Eq, Ord)

pattern SIMPLEBLE_OS_LINUX, SIMPLEBLE_OS_WINDOWS, SIMPLEBLE_OS_MACOS :: SimpleBleOS
pattern SIMPLEBLE_OS_LINUX = SimpleBleOS (#const SIMPLEBLE_OS_LINUX)
pattern SIMPLEBLE_OS_WINDOWS = SimpleBleOS (#const SIMPLEBLE_OS_WINDOWS)
pattern SIMPLEBLE_OS_MACOS = SimpleBleOS (#const SIMPLEBLE_OS_MACOS)
{-# COMPLETE SIMPLEBLE_OS_LINUX, SIMPLEBLE_OS_WINDOWS, SIMPLEBLE_OS_MACOS #-}

instance Show SimpleBleOS where
  show SIMPLEBLE_OS_LINUX = "SIMPLEBLE_OS_LINUX"
  show SIMPLEBLE_OS_WINDOWS = "SIMPLEBLE_OS_WINDOWS"
  show SIMPLEBLE_OS_MACOS = "SIMPLEBLE_OS_MACOS"

instance Storable SimpleBleOS where
  alignment _ = #alignment simpleble_os_t
  sizeOf _ = #size simpleble_os_t
  peek ptr = SimpleBleOS <$> peek (castPtr ptr)
  poke ptr (SimpleBleOS x) = poke (castPtr ptr :: Ptr CInt) x


newtype SimpleBleAddressType = SimpleBleAddressType CInt

pattern SIMPLEBLE_ADDRESS_TYPE_PUBLIC, SIMPLEBLE_ADDRESS_TYPE_RANDOM, SIMPLEBLE_ADDRESS_TYPE_UNSPECIFIED :: SimpleBleAddressType
pattern SIMPLEBLE_ADDRESS_TYPE_PUBLIC = SimpleBleAddressType (#const SIMPLEBLE_ADDRESS_TYPE_PUBLIC)
pattern SIMPLEBLE_ADDRESS_TYPE_RANDOM = SimpleBleAddressType (#const SIMPLEBLE_ADDRESS_TYPE_RANDOM)
pattern SIMPLEBLE_ADDRESS_TYPE_UNSPECIFIED = SimpleBleAddressType (#const SIMPLEBLE_ADDRESS_TYPE_UNSPECIFIED)
{-# COMPLETE SIMPLEBLE_ADDRESS_TYPE_PUBLIC, SIMPLEBLE_ADDRESS_TYPE_RANDOM, SIMPLEBLE_ADDRESS_TYPE_UNSPECIFIED #-}

instance Show SimpleBleAddressType where
  show SIMPLEBLE_ADDRESS_TYPE_PUBLIC = "SIMPLEBLE_ADDRESS_TYPE_PUBLIC"
  show SIMPLEBLE_ADDRESS_TYPE_RANDOM = "SIMPLEBLE_ADDRESS_TYPE_RANDOM"
  show SIMPLEBLE_ADDRESS_TYPE_UNSPECIFIED = "SIMPLEBLE_ADDRESS_TYPE_UNSPECIFIED"

instance Storable SimpleBleAddressType where
  alignment _ = #alignment simpleble_address_type_t
  sizeOf _ = #size simpleble_address_type_t
  peek ptr = SimpleBleAddressType <$> peek (castPtr ptr)
  poke ptr (SimpleBleAddressType x) = poke (castPtr ptr :: Ptr CInt) x
