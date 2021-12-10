use crate::{msgs::std_msgs, task_manager, topic_publishers::DRONE_EVENT_PUBLISHER};
use rosrust::ros_info;

/// Рассылает сообщения о создании события
pub fn log_event(event_name: &str) {
    DRONE_EVENT_PUBLISHER
        .send(std_msgs::String {
            data: event_name.to_string(),
        })
        .unwrap();
    ros_info!("Event {} is created.", event_name);
}

/// События, предотвращающие дальнейшее успешное выполнение задания
#[derive(Debug, Clone, PartialEq)]
pub enum Failure {
    /// Событие происходит, когда вольтаж аккумулятора дрона становится ниже допустимого предела
    LowVoltageDetected,
    /// Событие происходит, когда время на выполнение активной части задания истекло
    Timeout,
}

impl Failure {
    pub fn new_low_voltage_detected() -> Failure {
        log_event("LowVoltageDetected");
        *task_manager::IS_OK.lock().unwrap() = false;
        Failure::LowVoltageDetected {}
    }

    pub fn new_timeout() -> Failure {
        log_event("Timeout");
        *task_manager::IS_OK.lock().unwrap() = false;
        Failure::Timeout {}
    }
}
