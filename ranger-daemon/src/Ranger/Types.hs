{-# LANGUAGE NamedFieldPuns #-}
{-# LANGUAGE ApplicativeDo #-}
module Ranger.Types
  ( ObjectId(idBytes)
  , PhotoFragment(fragmentBytes)
  , SearchParameters(..)
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

-- | 16-byte (8-bit chars/unicode code points 0-255)
-- name for the object, padded with '\NUL' at the end.
newtype ObjectId = ObjectId { idBytes :: ByteString }

-- | 20-byte photo fragment, padded with 0 at the end if necessary.
newtype PhotoFragment = PhotoFragment { fragmentBytes :: ByteString }


objectName :: ObjectId -> Text
objectName (ObjectId bytes) = T.dropWhileEnd (=='\NUL') . T.pack $ BC.unpack bytes

toObjectId :: Text -> ObjectId
toObjectId = ObjectId . padNul 16 . B.take 16 . T.encodeUtf8

padNul :: Int -> ByteString -> ByteString
padNul l bs = bs <> B.replicate (l - B.length bs) 0

toFragments :: ByteString -> Vector PhotoFragment
toFragments bs
  | B.null bs = V.empty
  | B.length bs < 20 = V.singleton (PhotoFragment $ padNul 20 bs)
  | otherwise =
      let (fragment, rest) = B.splitAt 20 bs
       in V.cons (PhotoFragment fragment) (toFragments rest)

fromFragments :: [PhotoFragment] -> ByteString
fromFragments = B.dropWhileEnd (== 0) . B.concat . map fragmentBytes

nBytesToNFragments :: Integral a => a -> a
nBytesToNFragments nBytes = (nBytes `div` 20) + if nBytes `mod` 20 == 0 then 0 else 1

data SearchParameters = SearchParameters
  { timeout :: Word8 -- ^ minutes, 0 interpreted as nothing
  , maxRadius :: Word16 -- ^ centimetres, 0 interpreted as nothing
  }

instance S.Serialize SearchParameters where
  put SearchParameters{timeout, maxRadius} = S.put timeout >> S.put maxRadius
  get = do
    timeout <- S.get
    maxRadius <- S.get
    pure SearchParameters{timeout, maxRadius}

instance S.Serialize ObjectId where
  put (ObjectId bs) = mapM_ S.put (BC.unpack bs)
  get = ObjectId . B.pack <$> replicateM 16 (S.get :: S.Get Word8)

instance S.Serialize PhotoFragment where
  put (PhotoFragment bs) = mapM_ S.put (B.unpack bs)
  get = PhotoFragment . B.pack <$> replicateM 20 (S.get :: S.Get Word8)