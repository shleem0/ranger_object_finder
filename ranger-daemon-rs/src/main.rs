use bluer::gatt::{local::ReqError, CharacteristicReader};
use futures::{future, StreamExt};
use tokio::io::AsyncReadExt;
use clap::Parser;

mod ranger_bluetooth;

#[derive(Parser)]
struct Cli {
    bluetooth_adapter: Option<String>,
}

#[tokio::main(flavor = "multi_thread")]
async fn main() -> bluer::Result<()> {
    env_logger::init();

    let cli = Cli::parse();

    let mut bh = ranger_bluetooth::start_bluetooth(&cli.bluetooth_adapter).await?;
    
    let mut read_buf = Vec::new();
    let mut current_demo_reader: Option<CharacteristicReader> = None;
    
    loop {
        tokio::select! {
            evt = bh.is_demo_active_writes.next() => {
                match evt {
                    Some(req) => {
                        log::debug!("Write event from {}", req.device_address());
                        if current_demo_reader.is_none() {
                            log::info!("Accepting write event with MTU {} from {}", req.mtu(), req.device_address());
                            read_buf = vec![0; req.mtu()];
                            current_demo_reader = Some(req.accept()?);
                        } else {
                            log::warn!("Rejecting write event with MTU {} from {}: already being written to", req.mtu(), req.device_address());
                            req.reject(ReqError::InProgress);
                        }
                    },
                    None => {
                        log::info!("No more writes");
                        break;
                    }
                }
            }
            length_read = async {
                match &mut current_demo_reader {
                    Some(reader) => reader.read(&mut read_buf).await,
                    None => future::pending().await,
                }
            } => {
                match length_read {
                    Ok(0) => {
                        log::info!("Write stream ended");
                        current_demo_reader = None;
                    }
                    Ok(n) => {
                        let v: Option<bool> = ranger_bluetooth::RangerType::from_bytes(&read_buf[0..n].to_vec());
                        match v {
                            Some(demo_state_to_set) => {
                                bh.modify_state(|s| {
                                    ranger_bluetooth::RangerState {
                                        is_demo_active: demo_state_to_set,
                                        ..s.clone()
                                    }
                                }).await;
                            }
                            None => log::error!("Value parse fail: {:x?}", &read_buf[0..n]),
                        }
                    }
                    Err(err) => {
                        log::error!("Write stream error: {}", &err);
                        current_demo_reader = None;
                    }
                }
            }
        }
    }

    Ok(())
    }