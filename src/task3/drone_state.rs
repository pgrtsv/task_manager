use crate::{msgs::task_manager::DroneStatus, topic_publishers::*};
use lazy_static::lazy_static;
use rosrust::ros_info;
use std::{
    fmt::Debug,
    sync::{Arc, Mutex},
};

lazy_static! {
    /// Состояние дрона
    pub static ref DRONE_STATE: Arc<Mutex<Task3DroneState>> = {
        handle_set_state("Drone is waiting for commands...", false);
        Arc::new(Mutex::new(Task3DroneState::WaitingForCommand(
            WaitingForCommand {},
        )))
    };
}

machine!(
    #[derive(Debug, Clone, PartialEq)]
    enum Task3DroneState {
        WaitingForCommand,
        FollowingLine,
        // Landing,
    }
);

fn handle_set_state(info_text: &str, is_error: bool) {
    ros_info!("{}", info_text);
    DRONE_STATUS_PUBLISHER
        .send(DroneStatus {
            state: info_text.to_string(),
            is_error,
        })
        .unwrap();
}

pub fn set_drone_state(state: Task3DroneState) {
    handle_set_state(
        match &state {
            Task3DroneState::Error => "Drone state is invalid due to wrong transition!",
            Task3DroneState::WaitingForCommand(_) => "Drone is waiting for commands...",
            // Task3DroneState::Landing(_) => "Drone is landing...",
            Task3DroneState::FollowingLine(_) => "Drone is following line...",
        },
        match &state {
            Task3DroneState::Error => true,
            _ => false,
        },
    );
    *DRONE_STATE.lock().unwrap() = state;
}

pub fn get_drone_state() -> Task3DroneState {
    DRONE_STATE.lock().unwrap().clone()
}
