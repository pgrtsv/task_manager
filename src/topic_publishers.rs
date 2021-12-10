use crate::{
    msgs::{geometry_msgs::Point, nodes_monitor_msgs::Status, std_msgs, task_manager::DroneStatus},
    task_manager::TaskType,
};
use lazy_static::{initialize, lazy_static};
use rosrust::{self, Publisher};

lazy_static! {
    /// В топик публикуются названия событий, которые происходят с дроном
    pub static ref DRONE_EVENT_PUBLISHER: Publisher<std_msgs::String> =
        rosrust::publish::<std_msgs::String>("task_manager/events", 20).unwrap();

    /// В топик публикуются изменения статуса дрона
    pub static ref DRONE_STATUS_PUBLISHER: Publisher<DroneStatus> = rosrust::publish::<DroneStatus>("task_manager/status", 20).unwrap();

    /// В топик публикуются данные мониторинга узла
    pub static ref NODES_MONITOR_PUBLISHER: Publisher<Status> = rosrust::publish("nodes_monitor", 1).unwrap();

    /// В топик публикуются координаты точек найденных в 1 задании кубов
    pub static ref CUBES_OUTPUT_PUBLISHER: Publisher<Point> = rosrust::publish("object_cordinates", 1).unwrap();
}

/// Инициализирует паблишеров в топики, необходимые для задания `task_type`
pub fn init(task_type: TaskType) {
    initialize(&DRONE_EVENT_PUBLISHER);
    initialize(&DRONE_STATUS_PUBLISHER);
    initialize(&NODES_MONITOR_PUBLISHER);
    match task_type {
        TaskType::One => {
            initialize(&CUBES_OUTPUT_PUBLISHER);
        }
        TaskType::Two => {}
        TaskType::Three => {}
    }
}

/// Публикует точку - центр обнаруженного куба
pub fn publish_new_detected_cube(position: Point) {
    CUBES_OUTPUT_PUBLISHER.send(position).unwrap();
}
