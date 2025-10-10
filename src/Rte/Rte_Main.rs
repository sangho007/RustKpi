use std::sync::Arc;
use tokio::sync::broadcast;
use crate::Rte::Rte_Dto;

pub type VfbSender = broadcast::Sender<Rte_Dto::VfbEvent>;
pub type VfbReceiver = broadcast::Receiver<Rte_Dto::VfbEvent>;

pub type DebugSender = broadcast::Sender<Rte_Dto::VfbEvent>;
pub type DebugReceiver = broadcast::Receiver<Rte_Dto::VfbEvent>;



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