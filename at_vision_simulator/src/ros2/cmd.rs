use crate::ros2::topic::{FytCmdTopic, RosTopic};
use bevy::prelude::Resource;
use futures::StreamExt;
use r2r::std_msgs::msg::String as RosString;
use r2r::Node;
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};
use std::thread;

#[derive(Debug, Clone, Copy, Default)]
pub struct FytCommand {
    pub received: bool,
    pub control: bool,
    pub shoot: bool,
    pub yaw: f32,
    pub pitch: f32,
}

#[derive(Resource, Clone)]
pub struct FytCommandState(pub Arc<Mutex<FytCommand>>);

fn parse_bool(s: &str) -> Option<bool> {
    match s.trim() {
        "1" | "true" | "True" | "TRUE" => Some(true),
        "0" | "false" | "False" | "FALSE" => Some(false),
        _ => None,
    }
}

fn parse_cmd(data: &str) -> Option<FytCommand> {
    let parts: Vec<_> = data.split(',').collect();
    if parts.len() != 4 {
        return None;
    }

    Some(FytCommand {
        received: true,
        control: parse_bool(parts[0])?,
        shoot: parse_bool(parts[1])?,
        yaw: parts[2].trim().parse().ok()?,
        pitch: parts[3].trim().parse().ok()?,
    })
}

pub fn spawn_fyt_cmd_listener(node: &mut Node, stop_signal: Arc<AtomicBool>) -> FytCommandState {
    let state = Arc::new(Mutex::new(FytCommand::default()));
    let state_for_thread = state.clone();

    let mut subscriber = node
        .subscribe::<RosString>(FytCmdTopic::TOPIC, FytCmdTopic::QOS)
        .unwrap();

    thread::spawn(move || {
        futures::executor::block_on(async move {
            while let Some(msg) = subscriber.next().await {
                if stop_signal.load(Ordering::Acquire) {
                    break;
                }
                if let Some(cmd) = parse_cmd(&msg.data) {
                    if let Ok(mut guard) = state_for_thread.lock() {
                        *guard = cmd;
                    }
                }
            }
        });
    });

    FytCommandState(state)
}