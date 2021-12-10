#[macro_use]
extern crate machine;

pub mod task1 {
    pub mod commands;
    pub mod drone_state;
    pub mod events;
    pub mod transitions;
}
pub mod task2 {
    pub mod commands;
    pub mod drone_state;
    pub mod events;
    pub mod transitions;
}
pub mod task3 {
    pub mod commands;
    pub mod drone_state;
    pub mod events;
    pub mod transitions;
}
pub mod common_ros_utils;
pub mod events;
pub mod geometry;
pub mod msgs;
pub mod rviz;
pub mod service_clients;
pub mod task_manager;
pub mod topic_publishers;
pub mod topic_subscribers;

/// Название узла.
pub const NODE_NAME: &'static str = "task_manager";
