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
{-# LANGUAGE OverloadedStrings #-}
{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# OPTIONS_GHC -Wno-orphans #-}
module Ranger.Bluetooth.Protocol (test, RPSync, SideM(..)) where

import Control.Monad.Sync
import Data.Singletons
import Ranger.Bluetooth.Msg
import Data.Type.Bool
import Data.Type.Equality
import Ranger.Effect (RangerControl)
import Effectful
import Ranger.Bluetooth.Types
import qualified Data.Vector as V
import Data.Word
import qualified Data.ByteString as B
import Ranger.CApi.Types
import System.Random
import System.Random.Stateful

-- | Monad that only exists if the side is 'Ranger'.
newtype SideM m (s :: Side) a = SideM { unSide :: If (s == 'Ranger) m IO a }

deriving instance Functor m => Functor (SideM m 'Ranger)
deriving instance Applicative m => Applicative (SideM m 'Ranger)
deriving instance Monad m => Monad (SideM m 'Ranger)
deriving instance Functor (SideM m 'Phone)
deriving instance Applicative (SideM m 'Phone)
deriving instance Monad (SideM m 'Phone)

-- | Ranger-Phone sync
type RPSync s m = Sync s Msg (SideM m s)

privateP :: IO a -> RPSync s m (Private s 'Phone a)
privateP a = private SPhone Proxy (SideM a)

syncP :: Msg 'Phone a -> IO a -> RPSync s m a
syncP msg a = sync SPhone msg (SideM a)

test :: (RangerControl :> es) => Word16 -> RPSync s (Eff es) Bool
test imgSize = do
  _fn <- syncP FunctionCall $ do
    pure $ UpdateObject { objectId = ObjectId "IMGTEST", photoCount = 1}
  nBytes <- syncP AnnounceSizeBytes (pure imgSize)
  let nFragments = nBytesToNFragments nBytes
  frags <- privateP . pure $ toFragments undefined
  frags' <- V.generateM (fromIntegral nFragments) $ \i -> sync SPhone SendPhotoFragment $ pure (fromPrivate frags V.! i)
  let img' = fromFragments (fromIntegral nBytes) (V.toList frags')
  pure $ img == img'
  where
    seed = 69
    img = B.pack . fst $ runStateGen (mkStdGen seed) (uniformListM (fromIntegral imgSize))
