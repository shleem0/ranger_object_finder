{-# LANGUAGE LambdaCase #-}
{-# LANGUAGE NamedFieldPuns #-}
module Ranger.Demo
  ( demoProcedure
  , getDemoState
  , waitDemoStateChange
  , cancelDemoProcedure
  ) where
import System.IO.Unsafe
import Control.Concurrent.STM
import System.Process
import Data.Maybe
import System.Directory
import System.FilePath
import Data.Functor
import System.Exit
import Control.Concurrent.Async

-- | System.Process's ProcessHandle seems to have internal race conditions for
-- getProcessExitCode/waitForProcess, so this just wraps the demo process inside
data DemoHandle = DemoHandle
  { exitCode :: TVar (Maybe ExitCode)
  , processHandle :: ProcessHandle
  }

demoScript :: IO CreateProcess
demoScript = do
  d <- getHomeDirectory
  let demoDir = d </> "DEMO"
  createDirectoryIfMissing True demoDir
  pure (shell $ demoDir </> "demo_script.sh") { cwd = Just demoDir}

{-# NOINLINE demoScriptHandle #-}
demoScriptHandle :: TVar (Int, Maybe DemoHandle)
demoScriptHandle = unsafePerformIO $ newTVarIO (0, Nothing)

demoProcedure :: IO ()
demoProcedure = do
  alreadyRunning <- getDemoState
  if alreadyRunning
    then putStrLn "Demo already running!"
    else do
      s <- demoScript
      (_,_,_,processHandle) <- createProcess s
      exitCode <- newTVarIO Nothing
      let demoHandle = DemoHandle { exitCode, processHandle }

      a <- async $ do
        e <- waitForProcess processHandle
        atomically $ writeTVar exitCode (Just e)
      link a

      atomically $ modifyTVar' demoScriptHandle (\(i, _) -> (i + 1, Just demoHandle))
      putStrLn "Demo started"

getDemoState :: IO Bool
getDemoState = readTVarIO demoScriptHandle >>= maybe (pure False) (fmap isNothing . readTVarIO . exitCode) . snd

waitDemoStateChange :: (Bool -> IO a) -> IO a
waitDemoStateChange f = do
  mh <- readTVarIO demoScriptHandle

  case mh of
    (_, Nothing) -> do
      atomically $ do
        (_, mh') <- readTVar demoScriptHandle
        check (isJust mh')
      f True
    (i, Just h) -> do
      me <- readTVarIO $ exitCode h
      case me of
        Nothing -> do
          atomically $ do
            me' <- readTVar (exitCode h)
            check (isJust me')
          f False
        Just _ -> do
          atomically $ do
            i' <- fst <$> readTVar demoScriptHandle
            check (i /= i')
          f True

cancelDemoProcedure :: IO ()
cancelDemoProcedure = do
  (_, mh) <- readTVarIO demoScriptHandle
  runningH <- maybe (pure Nothing) (\h -> readTVarIO (exitCode h) <&> (\case Nothing -> Just h; Just _ -> Nothing) ) mh
  case runningH of
    Nothing -> putStrLn "Cancel request: Demo already not running!"
    Just h -> do
      cleanupProcess (Nothing, Nothing, Nothing, processHandle h)
      putStrLn "Demo cancelled"