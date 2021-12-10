use env_logger;
use rosrust::ros_info;
use task_manager::msgs::{self, std_msgs};
use task_manager::topic_publishers::*;
use task_manager::{task_manager::*, NODE_NAME};

fn main() {
    env_logger::init();
    rosrust::init(NODE_NAME);
    DRONE_EVENT_PUBLISHER
        .send(std_msgs::String {
            data: String::default(),
        })
        .unwrap();
    NODES_MONITOR_PUBLISHER
        .send(msgs::nodes_monitor_msgs::Status {
            status: msgs::nodes_monitor_msgs::Status::INITIALIZED,
        })
        .unwrap();
    let _task_manager = TaskManager::new();
    ros_info!("{} is initialized.", NODE_NAME);
    rosrust::spin();
}
