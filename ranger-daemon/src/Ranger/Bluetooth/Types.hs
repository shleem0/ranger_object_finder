{-# LANGUAGE NamedFieldPuns #-}
{-# LANGUAGE ApplicativeDo #-}
{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE StandaloneKindSignatures #-}
{-# LANGUAGE DataKinds #-}
{-# LANGUAGE GADTs #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE EmptyCase #-}
module Ranger.Bluetooth.Types
  ( ObjectId(idBytes)
  , PhotoFragment(fragmentBytes)
  , SearchParameters(..)
  , Side(..)
  , SSide(..)
  , PhoneSym0
  , RangerSym0
  , nBytesToNFragments
  , objectName
  , toObjectId
  , toFragments
  , fromFragments
  ) where

import Data.Text (Text)
import qualified Data.Text as T
import qualified Data.Text.Encoding as T
import Data.Word
import qualified Data.Serialize as S
import Data.ByteString (ByteString)
import qualified Data.ByteString as B
import qualified Data.ByteString.Char8 as BC
import Control.Monad
import Data.Vector (Vector)
import qualified Data.Vector as V
import Data.Singletons.TH
import Ranger.CApi.Types

data Side = Ranger | Phone

$(genSingletons [''Side])
$(singDecideInstance ''Side)

-- | 19-byte photo fragment, padded with 0 at the end if necessary.
newtype PhotoFragment = PhotoFragment { fragmentBytes :: ByteString } deriving Show


objectName :: ObjectId -> Text
objectName (ObjectId bytes) = T.dropWhileEnd (=='\NUL') . T.pack $ BC.unpack bytes

toObjectId :: Text -> ObjectId
toObjectId = ObjectId . padNul 16 . B.take 16 . T.encodeUtf8

padNul :: Int -> ByteString -> ByteString
padNul l bs = bs <> B.replicate (l - B.length bs) 0

toFragments :: ByteString -> Vector PhotoFragment
toFragments bs
  | B.null bs = V.empty
  | B.length bs < 19 = V.singleton (PhotoFragment $ padNul 19 bs)
  | otherwise =
      let (fragment, rest) = B.splitAt 19 bs
       in V.cons (PhotoFragment fragment) (toFragments rest)

fromFragments :: Int -> [PhotoFragment] -> ByteString
fromFragments size = B.take size . B.concat . map fragmentBytes

nBytesToNFragments :: Integral a => a -> a
nBytesToNFragments nBytes = (nBytes `div` 19) + if nBytes `mod` 19 == 0 then 0 else 1

instance S.Serialize PhotoFragment where
  put (PhotoFragment bs) = mapM_ S.put (B.unpack bs)
  get = PhotoFragment . B.pack <$> replicateM 19 (S.get :: S.Get Word8)