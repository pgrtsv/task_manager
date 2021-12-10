use rosrust::ros_info;
use rosrust_actionlib::{Action, SimpleActionClient};
use serde::Deserialize;

use crate::NODE_NAME;

/// Ожидает указанный сервис. Пока сервис не станет доступен, каждые 10 секунд публикует
/// сообщение об ожидании.
pub fn wait_for_service(service_name: &str) {
    while rosrust::is_ok()
        && rosrust::wait_for_service(service_name, Some(std::time::Duration::from_secs(10)))
            .is_err()
    {
        ros_info!(
            "{}: waiting for required service \"{}\"...",
            NODE_NAME,
            service_name
        );
    }
    if rosrust::is_ok() {
        ros_info!(
            "{}: required service \"{}\" is up.",
            NODE_NAME,
            service_name
        );
    } else {
        ros_info!(
            "{}: stopped waiting for service \"{}\" due to ROS shutdown.",
            NODE_NAME,
            service_name
        );
    }
}

/// Ожидает указанный топик. Пока топик не станет доступен, каждые 10 секунд публикует
/// сообщение об ожидании.
pub fn wait_for_topic(topic_name: &str) {
    let rate = rosrust::rate(0.1);
    let normalized_topic_name = if topic_name.starts_with("/") {
        topic_name.to_string()
    } else {
        format!("/{}", topic_name)
    };
    while rosrust::is_ok()
        && rosrust::topics()
            .unwrap()
            .into_iter()
            .all(|topic| topic.name != normalized_topic_name)
    {
        ros_info!(
            "{}: waiting for required topic \"{}\"...",
            NODE_NAME,
            topic_name
        );
        rate.sleep();
    }
    if rosrust::is_ok() {
        ros_info!("{}: required topic \"{}\" is up.", NODE_NAME, topic_name);
    } else {
        ros_info!(
            "{}: stopped waiting for topic \"{}\" due to ROS shutdown.",
            NODE_NAME,
            topic_name
        );
    }
}

/// Ожидает указанный сервер actionlib. Пока сервер не станет доступен, каждые 10 секунд публикует
/// сообщение об ожидании.
pub fn wait_for_actionlib_server<T: Action>(client: &SimpleActionClient<T>, client_name: &str) {
    while rosrust::is_ok() && !client.wait_for_server(Some(rosrust::Duration::from_seconds(10))) {
        ros_info!(
            "{}: waiting for required actionlib server \"{}\"...",
            NODE_NAME,
            client_name
        );
    }
    if rosrust::is_ok() {
        ros_info!(
            "{}: required actionlib server \"{}\" is up.",
            NODE_NAME,
            client_name
        );
    } else {
        ros_info!(
            "{}: stopped waiting for actionlib server \"{}\" due to ROS shutdown.",
            NODE_NAME,
            client_name
        );
    }
}

/// Возвращает параметр `param_name`, переданный через ROS. Если параметр не передан, либо при его получении произошла ошибка,
/// возвращается `default_value`.
pub fn get_param<'b, T: Deserialize<'b>>(param_name: &str, default_value: T) -> T {
    match rosrust::param(param_name) {
        Some(param) => match param.get() {
            Ok(param) => {
                rosrust::ros_info!("Recieved param \"{}\" from ROS.", param_name);
                param
            }
            Err(error) => {
                rosrust::ros_warn!("Could not recieve param \"{}\" from ROS. Error message: {} Default value is set.", param_name, error);
                default_value
            }
        },
        None => {
            rosrust::ros_warn!(
                "Could not recieve param \"{}\" from ROS. Default value is set.",
                param_name
            );
            default_value
        }
    }
}
