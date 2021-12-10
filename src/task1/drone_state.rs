use crate::{msgs::task_manager, topic_publishers::*};
use lazy_static::lazy_static;
use rosrust::ros_info;
use std::{
    fmt::Debug,
    sync::{Arc, Mutex},
};

lazy_static! {
    static ref DRONE_STATE: Arc<Mutex<DroneState>> = {
        handle_set_state("Drone is waiting for commands...", false);
        Arc::new(Mutex::new(DroneState::WaitingForCommand(
            WaitingForCommand {},
        )))
    };
}

machine!(
    #[derive(Debug, Clone, PartialEq)]
    enum DroneState {
        WaitingForCommand {},
        Landing,
        FlyingInside,
        Exploring,
        ReturningToStartPoint,
        LookingForEntry,
    }
);

pub fn get_drone_state() -> DroneState {
    DRONE_STATE.lock().unwrap().clone()
}

fn handle_set_state(info_text: &str, is_error: bool) {
    ros_info!("{}", info_text);
    DRONE_STATUS_PUBLISHER
        .send(task_manager::DroneStatus {
            state: info_text.to_string(),
            is_error,
        })
        .unwrap();
}

pub fn set_drone_state(drone_state: DroneState) {
    handle_set_state(
        match &drone_state {
            DroneState::Error => "Drone state is invalid due to wrong transition!",
            DroneState::WaitingForCommand(_) => "Drone is waiting for commands...",
            DroneState::Landing(_) => "Drone is landing...",
            DroneState::FlyingInside(_) => {
                "Drone is flying into the building to start exploring..."
            }
            DroneState::Exploring(_) => "Drone is exploring...",
            DroneState::ReturningToStartPoint(_) => "Drone is returning to the start point...",
            DroneState::LookingForEntry(_) => "Drone is looking for entry to building...",
        },
        match &drone_state {
            DroneState::Error => true,
            _ => false,
        },
    );
    *DRONE_STATE.lock().unwrap() = drone_state;
}
