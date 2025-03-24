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

demoScript :: CreateProcess
demoScript = (shell "~/DEMO/demo_script.sh") { cwd = Just "~/DEMO" }

{-# NOINLINE demoScriptHandle #-}
demoScriptHandle :: TVar (Int, Maybe ProcessHandle)
demoScriptHandle = unsafePerformIO $ newTVarIO (0, Nothing)

demoProcedure :: IO ()
demoProcedure = do
  alreadyRunning <- getDemoState
  if alreadyRunning
    then putStrLn "Demo already running!"
    else do
      (_,_,_,h) <- createProcess demoScript
      earlyExit <- isJust <$> getProcessExitCode h
      if earlyExit
        then putStrLn "(!!!) Failed to start demo"
        else do
          atomically $ modifyTVar' demoScriptHandle (\(i, _) -> (i + 1, Just h))
          putStrLn "Demo started"

getDemoState :: IO Bool
getDemoState = readTVarIO demoScriptHandle >>= maybe (pure False) (fmap isNothing . getProcessExitCode) . snd

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
      me <- getProcessExitCode h
      case me of
        Nothing -> do
          _ <- waitForProcess h
          f False
        Just _ -> do
          atomically $ do
            i' <- fst <$> readTVar demoScriptHandle
            check (i /= i')
          f True

cancelDemoProcedure :: IO ()
cancelDemoProcedure = do
  (_, mh) <- readTVarIO demoScriptHandle
  runningH <- maybe (pure Nothing) (\h -> getProcessExitCode h >>= \e -> pure (h <$ e)) mh
  case runningH of
    Nothing -> putStrLn "Cancel request: Demo already not running!"
    Just h -> do
      cleanupProcess (Nothing, Nothing, Nothing, h)
      putStrLn "Demo cancelled"