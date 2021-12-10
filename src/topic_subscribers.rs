use std::sync::Mutex;

use crate::{
    common_ros_utils::wait_for_topic, msgs::geometry_msgs::*, msgs::sensor_msgs::BatteryState,
    task_manager::TaskType,
};

use lazy_static::{initialize, lazy_static};
use rosrust::Subscriber;

lazy_static! {
    static ref DRONE_POSE: Mutex<Option<PoseStamped>> = Mutex::new(None);
    pub static ref DRONE_POSE_SUBSCRIBER: Subscriber = {
        wait_for_topic("/mavros/local_position/pose");
        rosrust::subscribe("/mavros/local_position/pose", 3, |pose: PoseStamped| {
            *DRONE_POSE.lock().unwrap() = Some(pose);
        })
        .unwrap()
    };
    static ref BATTERY_VOLTAGE: Mutex<Option<f32>> = Mutex::new(None);
    pub static ref BATTERY_VOLTAGE_SUBSCRIBER: Subscriber = {
        wait_for_topic("/mavros/battery");
        rosrust::subscribe("/mavros/battery", 1, |state: BatteryState| {
            *BATTERY_VOLTAGE.lock().unwrap() = Some(state.voltage);
        })
        .unwrap()
    };
}

/// Инициализирует подписчиков на топики
pub fn init(_: TaskType) {
    initialize(&DRONE_POSE_SUBSCRIBER);
    initialize(&BATTERY_VOLTAGE_SUBSCRIBER);
}

/// Возвращает текущую позу дрона в СК map
/// Блокирует текущий поток, пока не придет хотя бы одно сообщение из топика с позой.
pub fn get_current_drone_pose() -> PoseStamped {
    if let Some(drone_pose) = &*DRONE_POSE.lock().unwrap() {
        return drone_pose.clone();
    }
    initialize(&DRONE_POSE_SUBSCRIBER);
    let rate = rosrust::rate(10.0);
    while DRONE_POSE.lock().unwrap().is_none() {
        rate.sleep();
    }
    DRONE_POSE.lock().unwrap().as_ref().unwrap().clone()
}

/// Возвращает текущий вольтаж аккумулятора.
/// Блокирует текущий поток, пока не придет хотя бы одно сообщение из топика с вольтажом.
pub fn get_current_battery_voltage() -> f32 {
    if let Some(battery_voltage) = *BATTERY_VOLTAGE.lock().unwrap() {
        return battery_voltage;
    }
    let rate = rosrust::rate(10.0);
    while BATTERY_VOLTAGE.lock().unwrap().is_none() {
        rate.sleep();
    }
    (*BATTERY_VOLTAGE.lock().unwrap()).unwrap()
}
