use crate::rte::rte_dto;
use tokio::sync::broadcast;

pub type VfbSender = broadcast::Sender<rte_dto::VfbEvent>;
pub type VfbReceiver = broadcast::Receiver<rte_dto::VfbEvent>;

pub type DebugSender = broadcast::Sender<rte_dto::VfbEvent>;
pub type DebugReceiver = broadcast::Receiver<rte_dto::VfbEvent>;



const VFB_CHANNEL_CAPACITY: usize = 1;
const DEBUG_CHANNEL_CAPACITY: usize = 1000;

pub fn init() -> VfbSender {
    let (tx, _) = broadcast::channel(VFB_CHANNEL_CAPACITY);
    tx
}

pub fn debug_init() -> DebugSender {
    let (tx, _) = broadcast::channel(DEBUG_CHANNEL_CAPACITY);
    tx
}
