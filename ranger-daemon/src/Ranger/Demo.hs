module Ranger.Demo
  ( demoProcedure
  , getDemoState
  , waitDemoStateChange
  , cancelDemoProcedure
  ) where
import System.IO.Unsafe
import Control.Concurrent.STM

{-# NOINLINE demoState #-}
-- | ugly temporary testing trick, represents data provided by other ROS nodes
demoState :: TVar Bool
demoState = unsafePerformIO $ newTVarIO False

-- | TODO
demoProcedure :: IO ()
demoProcedure = do
  atomically $ writeTVar demoState True
  putStrLn "Demo started"

-- | TODO
getDemoState :: IO Bool
getDemoState = readTVarIO demoState

-- | TODO
waitDemoStateChange :: (Bool -> IO a) -> IO a
waitDemoStateChange f = do
  s1 <- readTVarIO demoState
  s2 <- atomically $ do
    s <- readTVar demoState
    check (s /= s1)
    pure s
  f s2 

-- | TODO
cancelDemoProcedure :: IO ()
cancelDemoProcedure = do
  atomically $ writeTVar demoState False
  putStrLn "Demo cancelled"