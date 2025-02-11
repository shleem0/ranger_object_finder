use std::{pin::Pin, sync::{Arc, LazyLock, Mutex, MutexGuard}};

use bluer::{adv::{Advertisement, AdvertisementHandle}, gatt::local::{characteristic_control, Application, ApplicationHandle, Characteristic, CharacteristicControl, CharacteristicControlEvent, CharacteristicNotifier, CharacteristicNotify, CharacteristicNotifyFun, CharacteristicNotifyMethod, CharacteristicRead, CharacteristicReadFun, CharacteristicWrite, CharacteristicWriteIoRequest, CharacteristicWriteMethod, Service}};
use futures::{future::{self}, FutureExt, Stream, StreamExt};
use uuid::Uuid;

#[derive(Debug, Clone)]
pub struct RangerState {
    pub is_demo_active: bool,
}

struct RangerStateNotifiers {
    is_demo_active_notifier: Option<CharacteristicNotifier>,
}

pub struct RangerBluetoothHandle {
    _adv_handle: AdvertisementHandle,
    _app_handle: ApplicationHandle,
    notifiers: Arc<Mutex<RangerStateNotifiers>>,
    state: Arc<Mutex<RangerState>>,
    pub is_demo_active_writes: Pin<Box<dyn Stream<Item = CharacteristicWriteIoRequest> + Send>>,
}

impl RangerBluetoothHandle {
    pub async fn modify_state<F>(&self, f: F)
    where
        F: FnOnce(&RangerState) -> RangerState,
    {
        let mut s = force_lock(&self.state);
        let mut ns = force_lock(&self.notifiers);

        let s2 = f(&s);
        if s.is_demo_active != s2.is_demo_active {
            if let Some(notifier) = &mut ns.is_demo_active_notifier {
                let r = notifier.notify(s2.is_demo_active.to_bytes()).await;
                log::warn!("Failed to notify: {:?}", r);
            }
        };

        *s = s2;
    }
}

fn force_lock<T>(m: &Mutex<T>) -> MutexGuard<T> {
    match m.lock() {
        Ok(t) => t,
        Err(e) => panic!("unrecoverable: lock poisoned: {:?}", e),
    }
}

pub trait RangerType: Sized {
    fn from_bytes(bytes: &[u8]) -> Option<Self>;
    fn to_bytes(&self) -> Vec<u8>;
}

impl RangerType for bool {
    fn from_bytes(bytes: &[u8]) -> Option<bool> {
        if bytes == [0] {
            Some(false)
        } else if bytes == [1] {
            Some(true)
        } else {
            None
        }
    }

    fn to_bytes(&self) -> Vec<u8> {
        if *self {
            vec![1]
        } else {
            vec![0]
        }
    }
}

// using the following namespace with sha1 (uuidv5)
// 5a772388-abcf-43a7-9691-6598ab86b2f6

// RangerDemo
// fbb876fb-3ee3-5315-9716-01ede2358aab
const RANGER_DEMO_SERVICE_UUID: Uuid = Uuid::from_bytes(
    [ 0xfb, 0xb8, 0x87, 0xfb
    , 0x3e, 0xe3
    , 0x53, 0x15
    , 0x97, 0x16
    , 0x01, 0xed, 0xe2, 0x35, 0x8a, 0xab
    ]);

// IsDemoActive
// 82e761bc-8508-5f80-90ee-9b3455444798
const IS_DEMO_ACTIVE_UUID: Uuid = Uuid::from_bytes(
    [ 0x82, 0xe7, 0x61, 0xbc
    , 0x85, 0x08
    , 0x5f, 0x80
    , 0x90, 0xee
    , 0x9b, 0x34, 0x55, 0x44, 0x47, 0x98
    ]);

static LE_ADVERTISEMENT: LazyLock<Advertisement> = LazyLock::new(||Advertisement {
    service_uuids: vec![RANGER_DEMO_SERVICE_UUID].into_iter().collect(),
    discoverable: Some(true),
    local_name: Some("Ranger".to_string()),
    ..Default::default()
});

fn on_notify_request(rn: Arc<Mutex<RangerStateNotifiers>>) -> CharacteristicNotifyFun {
    Box::new(move |req| {
        log::info!("Received notify request");
        let mut s = force_lock(&rn);
        s.is_demo_active_notifier = Some(req);
        Box::pin(future::ready(()))
    }
    )
}

fn on_read_request(rs: Arc<Mutex<RangerState>>) -> CharacteristicReadFun {
    Box::new(move |r| {
        let rs1 = rs.clone();
        async move {
            log::info!("Received read request with MTU {} from {}", r.mtu, r.device_address);
            match rs1.lock() {
                Ok(s) => if s.is_demo_active {
                    Ok(vec![1])
                } else {
                    Ok(vec![0])
                },
                Err(e) => panic!("unrecoverable: state lock poisoned: {:?}", e),
            }
        }.boxed()
    })
}

fn demo_service(rs: Arc<Mutex<RangerState>>, rn: Arc<Mutex<RangerStateNotifiers>>) -> (Service, CharacteristicControl) {
    let (demo_active_control, demo_active_handle) = characteristic_control();

    let s = Service {
        uuid: RANGER_DEMO_SERVICE_UUID,
        primary: true,
        characteristics: vec![
            // IsDemoActive
            Characteristic {
                uuid: IS_DEMO_ACTIVE_UUID,
                write: Some(CharacteristicWrite {
                    write: true,
                    write_without_response: true,
                    encrypt_write: true,
                    encrypt_authenticated_write: true,
                    method: CharacteristicWriteMethod::Io,
                    ..Default::default()
                }),
                notify: Some(CharacteristicNotify {
                    notify: true,
                    method: CharacteristicNotifyMethod::Fun(on_notify_request(rn)),
                    ..Default::default()
                }),
                read: Some(CharacteristicRead {
                    read: true,
                    fun: on_read_request(rs),
                    ..Default::default()
                }),
                control_handle: demo_active_handle,
                ..Default::default()
            }
        ],
        ..Default::default()
    };
    (s, demo_active_control)    
}


/// Start the Bluetooth adapter, initialise GATT services and advertise them.
///
/// The Bluetooth application stops when the returned handle is dropped.
pub async fn start_bluetooth(adapter_name: &Option<String>) -> bluer::Result<RangerBluetoothHandle> {
    let ranger_state = Arc::new(Mutex::new(RangerState {
        is_demo_active: false,
    }));
    let ranger_notifiers = Arc::new(Mutex::new(RangerStateNotifiers {
        is_demo_active_notifier: None,
    }));
    let (ds, is_demo_active_ctl) = demo_service(ranger_state.clone(), ranger_notifiers.clone());

    let session = bluer::Session::new().await?;
    let adapter = match adapter_name.as_deref() {
        Some(name) => session.adapter(name)?,
        None => session.default_adapter().await?,
    };

    adapter.set_powered(true).await?;

    log::info!("Serving GATT service on Bluetooth adapter {}", adapter.name());

    let app_handle = adapter.serve_gatt_application(Application {
        services: vec![ds],
        ..Default::default()
    }).await?;

    log::info!("Advertising on Bluetooth adapter {} with address {}", adapter.name(), adapter.address().await?);

    let adv_handle = adapter.advertise(LE_ADVERTISEMENT.clone()).await?;

    log::debug!("IsDemoActive characteristic handle is 0x{:x}", is_demo_active_ctl.handle()?);

    let write_requests = is_demo_active_ctl.filter_map(|evt| match evt {
        CharacteristicControlEvent::Write(req) => future::ready(Some(req)),
        CharacteristicControlEvent::Notify(_) => unreachable!("Using callbacks for notify")
    }).boxed();

    Ok(RangerBluetoothHandle {
        _adv_handle: adv_handle,
        _app_handle: app_handle,
        state: ranger_state,
        notifiers: ranger_notifiers,
        is_demo_active_writes: write_requests,
    })
}
