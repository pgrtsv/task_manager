use crate::{
    common_ros_utils::get_param,
    events::Failure,
    msgs::{nodes_monitor_msgs::Status, task_manager::*},
    service_clients, task1, task2, task3,
    topic_publishers::{self, *},
    topic_subscribers::{self, *},
    NODE_NAME,
};
use lazy_static::{initialize, lazy_static};
use rosrust::Service;
use std::{
    sync::{Arc, Mutex},
    thread,
};

lazy_static! {
    pub static ref OPTIONS: TaskManagerOptions = TaskManagerOptions::new();
    pub static ref IS_OK: Arc<Mutex<bool>> = Arc::new(Mutex::new(true));
}

/// Параметры
#[derive(Clone, Copy)]
pub struct TaskManagerOptions {
    pub operating_altitude: f32,
    pub low_altitude: f32,
    pub linear_velocity: f32,
    pub linear_acceleration: f32,
    pub angular_velocity: f32,
    pub angular_acceleration: f32,
    pub flying_into_hole_detection_distance: f32,
    pub flying_into_hole_pass_distance: f64,
    pub flying_into_hole_detection_pass_distance: f64,
    pub min_battery_voltage: f32,
    pub task1: Task1Options,
    pub task2: Task2Options,
}

/// Параметры, необходимые для 1 задания.
#[derive(Clone, Copy)]
pub struct Task1Options {
    /// Количество кубов, которое необходимо найти.
    pub cubes_count: usize,
    /// Время (мин), которое отводится на поиск кубов.
    pub max_timer_minutes: f32,
}

/// Параметры, необходимые для 2 задания.
#[derive(Clone, Copy)]
pub struct Task2Options {
    /// Наибольшее значение координаты z, ниже которой найденные QR-коды считаются расположенными на полу.
    pub max_floor_z: f32,
    /// Наибольшее расстояние между центрами QR-кодов (м), при котором они считаются одним обнаруженным кодом.
    pub max_qr_distance_tolerance: f32,
    /// Наибольшее расстояние между центром проёма и центром QR-кода (м), при котором они считаются связанными.
    pub max_association_distance: f32,
}

impl TaskManagerOptions {
    pub fn new() -> TaskManagerOptions {
        TaskManagerOptions {
            operating_altitude: get_param("~operating_altitude", 1.5),
            low_altitude: get_param("~low_altitude", 0.5),
            linear_velocity: get_param("~linear_velocity", 0.1),
            linear_acceleration: get_param("~linear_acceleration", 0.1),
            angular_velocity: get_param("~angular_velocity", 0.1),
            angular_acceleration: get_param("~angular_acceleration", 0.1),
            flying_into_hole_detection_distance: get_param(
                "~flying_into_hole_detection_distance",
                0.3,
            ),
            flying_into_hole_pass_distance: get_param("~flying_into_hole_pass_distance", 0.5),
            flying_into_hole_detection_pass_distance: get_param(
                "~flying_into_hole_detection_pass_distance",
                0.3,
            ),
            min_battery_voltage: get_param("~min_battery_voltage", 10.0),
            task1: Task1Options {
                cubes_count: get_param("~task1_cubes_count", 5),
                max_timer_minutes: get_param("~task1_max_timer_minutes", 9.0),
            },
            task2: Task2Options {
                max_floor_z: get_param("~task2_max_floor_z", 0.2),
                max_qr_distance_tolerance: get_param("~task2_max_qr_distance_tolerance", 0.2),
                max_association_distance: get_param("~task2_max_association_distance", 0.6),
            },
        }
    }
}

pub struct TaskManager {
    pub start_service: Service,
}

#[derive(Debug, Clone, Copy)]
pub enum TaskType {
    One,
    Two,
    Three,
}

impl TaskManager {
    pub fn new() -> TaskManager {
        initialize(&OPTIONS);
        let start_service =
            rosrust::service::<Start, _>(format!("{}/start", NODE_NAME).as_str(), |start| {
                let task_type = match start.task {
                    1 => TaskType::One,
                    2 => TaskType::Two,
                    3 => TaskType::Three,
                    _ => return Err("Wrong task number is specified".to_string()),
                };
                thread::spawn(move || {
                    let rate = rosrust::rate(1.0);
                    while rosrust::is_ok() {
                        if get_current_battery_voltage() <= OPTIONS.min_battery_voltage {
                            match task_type {
                                TaskType::One => task1::drone_state::set_drone_state(
                                    task1::drone_state::get_drone_state()
                                        .on_failure(Failure::new_low_voltage_detected()),
                                ),
                                TaskType::Two => task2::drone_state::set_drone_state(
                                    task2::drone_state::get_drone_state()
                                        .on_failure(Failure::new_low_voltage_detected()),
                                ),
                                TaskType::Three => todo!(),
                            };
                            break;
                        }
                        rate.sleep();
                    }
                });
                thread::spawn(move || {
                    let start_time = rosrust::now();
                    let rate = rosrust::rate(0.2);
                    while rosrust::is_ok() {
                        let minutes_passed = (rosrust::now() - start_time).sec as f32 / 60.0;
                        if minutes_passed > 1.0 && minutes_passed - minutes_passed.floor() < 0.08 {
                            rosrust::ros_info!("{} minute(s) has passed!", minutes_passed.floor());
                        }
                        if minutes_passed > OPTIONS.task1.max_timer_minutes {
                            match task_type {
                                TaskType::One => task1::drone_state::set_drone_state(
                                    task1::drone_state::get_drone_state()
                                        .on_failure(Failure::new_timeout()),
                                ),
                                TaskType::Two => task2::drone_state::set_drone_state(
                                    task2::drone_state::get_drone_state()
                                        .on_failure(Failure::new_timeout()),
                                ),
                                TaskType::Three => todo!(),
                            };
                            break;
                        }
                        rate.sleep();
                    }
                });
                service_clients::init(task_type);
                topic_publishers::init(task_type);
                topic_subscribers::init(task_type);
                match task_type {
                    TaskType::One => task1::drone_state::set_drone_state(
                        task1::drone_state::get_drone_state().on_start(task1::commands::Start {}),
                    ),
                    TaskType::Two => {
                        task2::drone_state::set_drone_state(
                            task2::drone_state::get_drone_state()
                                .on_start(task2::commands::Start {}),
                        );
                    }
                    TaskType::Three => {
                        task3::drone_state::set_drone_state(
                            task3::drone_state::get_drone_state()
                                .on_start(task3::commands::Start {}),
                        );
                    }
                }
                NODES_MONITOR_PUBLISHER
                    .send(Status {
                        status: Status::STARTED,
                    })
                    .unwrap();
                Ok(StartRes {})
            })
            .unwrap();
        TaskManager { start_service }
    }
}
