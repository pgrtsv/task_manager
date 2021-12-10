use crate::{
    geometry,
    msgs::{detection_msgs::*, geometry_msgs::*, task_manager::DroneStatus},
    task_manager,
    topic_publishers::*,
};
use lazy_static::lazy_static;
use rosrust::ros_info;
use std::{
    collections::HashMap,
    fmt::Debug,
    sync::{Arc, Mutex},
};

lazy_static! {
    /// Состояние дрона
    pub static ref DRONE_STATE: Arc<Mutex<Task2DroneState>> = {
        handle_set_state("Drone is waiting for commands...", false);
        Arc::new(Mutex::new(Task2DroneState::WaitingForCommand(
            WaitingForCommand {},
        )))
    };

    /// Обнаруженные в комнате QR-коды
    pub static ref DETECTED_QR_CODES: Arc<Mutex<Vec<Qr>>> = Arc::new(Mutex::new(Vec::new()));

    /// Актуальные обнаруженные проёмы из pos_collector
    pub static ref DETECTED_HOLES: Arc<Mutex<DetectedObjects>> = Arc::new(Mutex::new(DetectedObjects {
        header: geometry::new_header("map"),
        detected_objects: Vec::default(),
    }));

    /// Связи между обнаруженными проёмами и QR-кодами. Ключ - ID проёма, возвращаемого pos_collector-ом,
    /// значение - индекс QR-кода в DETECTED_QR_CODES
    pub static ref DETECTED_HOLE_QR_CONNECTIONS: Arc<Mutex<HashMap<usize, usize>>> =
        Arc::new(Mutex::new(HashMap::new()));

    /// Вектор, содержащий номера последовательно пройденных дроном комнат
    pub static ref PASSED_ROOMS_NUMBERS: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
}

machine!(
    #[derive(Debug, Clone, PartialEq)]
    enum Task2DroneState {
        WaitingForCommand,
        Exploring,
        FlyingIntoHole,
        FlyingToLandingPoint,
        Landing,
    }
);

pub fn is_qr_already_detected(qr: &Qr) -> bool {
    let detected_qr_codes = &*DETECTED_QR_CODES.lock().unwrap();
    if qr.position == Point::default() {
        return qr.content.chars().count() > 2
            || detected_qr_codes
                .iter()
                .any(|detected_qr| detected_qr.is_on_floor && detected_qr.content == qr.content);
    }
    detected_qr_codes.iter().all(|detected_qr| {
        geometry::get_distance_between_points(&detected_qr.position, &qr.position)
            > (task_manager::OPTIONS.task2.max_qr_distance_tolerance as f64)
    })
}

pub fn add_qr(qr: Qr) -> usize {
    let codes = &mut *DETECTED_QR_CODES.lock().unwrap();
    codes.push(qr);
    codes.len() - 1
}

pub fn add_hole_qr_connection(hole_id: usize, qr_index: usize) {
    DETECTED_HOLE_QR_CONNECTIONS
        .lock()
        .unwrap()
        .insert(hole_id, qr_index);
}

pub fn find_connected_hole(qr: &Qr) -> Option<DetectedObject> {
    match DETECTED_HOLES
        .lock()
        .unwrap()
        .detected_objects
        .iter()
        .find(|hole| {
            geometry::get_distance_between_points(&hole.pose.position, &qr.position)
                <= (task_manager::OPTIONS.task2.max_association_distance as f64)
        }) {
        Some(object) => Some(object.clone()),
        None => None,
    }
}

pub fn find_connected_qr(hole: &DetectedObject) -> Option<usize> {
    DETECTED_QR_CODES.lock().unwrap().iter().position(|qr| {
        geometry::get_distance_between_points(&hole.pose.position, &qr.position)
            <= (task_manager::OPTIONS.task2.max_association_distance as f64)
    })
}

/// Находит и возвращает проём, связанный с QR-кодом с индексом qr_index
pub fn match_qr_index_with_hole(qr_index: usize) -> Option<DetectedObject> {
    rosrust::ros_info!("match_qr_index_with_hole");
    let detected_hole_qr_connections = &*DETECTED_HOLE_QR_CONNECTIONS.lock().unwrap();
    match detected_hole_qr_connections.get(&qr_index) {
        Some(hole_id) => Some(
            (*DETECTED_HOLES.lock().unwrap())
                .detected_objects
                .iter()
                .find(|hole| &(hole.id as usize) == hole_id)
                .unwrap()
                .clone(),
        ),
        None => None,
    }
}

/// Проверяет, имеет ли нарисованный на полу QR-код с индексом qr_index то же содержимое,
/// что и последовательное объединение содержимого QR-кодов в пройденных комнатах.
/// Если QR-код имеет то же содержимое, возвращается он сам, иначе - None
pub fn match_qr_index_with_passed_rooms(qr_index: usize) -> Option<Qr> {
    let detected_qr_codes = &*DETECTED_QR_CODES.lock().unwrap();
    let qr = detected_qr_codes.iter().nth(qr_index).unwrap();
    let passed_rooms_number: String = (*PASSED_ROOMS_NUMBERS.lock().unwrap()).join("");
    if qr.content == passed_rooms_number {
        Some(qr.clone())
    } else {
        None
    }
}

/// Находит и возвращает индекс QR-кода, нарисованного на полу и имеющего то же содержимое, что и
/// QR-код с индексом qr_index
pub fn match_qr_index_with_qr_on_floor(qr_index: usize) -> Option<usize> {
    rosrust::ros_info!("match_qr_index_with_qr_on_floor");
    let detected_qr_codes = &*DETECTED_QR_CODES.lock().unwrap();
    let qr_content = &detected_qr_codes[qr_index].content;
    detected_qr_codes
        .iter()
        .position(|qr| qr.is_on_floor && &qr.content == qr_content)
}

/// Находит QR-код, связанный с дверью с ID hole_id. Если такой QR-код найден, находит и возвращает
/// индекс QR-кода, нарисованного на полу и имеющего то же содержимое, что QR-код двери
pub fn match_hole_id_with_qr_on_floor(hole_id: usize) -> Option<usize> {
    let detected_hole_qr_connections = &*DETECTED_HOLE_QR_CONNECTIONS.lock().unwrap();
    match detected_hole_qr_connections
        .iter()
        .find_map(|(qr_index, hole_id_)| {
            if *hole_id_ == hole_id {
                Some(qr_index)
            } else {
                None
            }
        }) {
        Some(qr_index) => match_qr_index_with_qr_on_floor(*qr_index),
        None => None,
    }
}

/// QR-код
#[derive(Debug, Clone, PartialEq)]
pub struct Qr {
    /// Позиция кода в СК map
    pub position: Point,
    /// Содержание кода
    pub content: String,
    /// true, если QR расположен на полу
    pub is_on_floor: bool,
}

impl Qr {
    pub fn new(position: Point, content: String) -> Qr {
        let z = position.z;
        Qr {
            position: position,
            content,
            is_on_floor: z < (task_manager::OPTIONS.task2.max_floor_z as f64),
        }
    }
}

fn handle_set_state(info_text: &str, is_error: bool) {
    ros_info!("{}", info_text);
    DRONE_STATUS_PUBLISHER
        .send(DroneStatus {
            state: info_text.to_string(),
            is_error,
        })
        .unwrap();
}

pub fn set_drone_state(state: Task2DroneState) {
    handle_set_state(
        match &state {
            Task2DroneState::FlyingIntoHole(_) => "Drone is flying into the hole...",
            Task2DroneState::FlyingToLandingPoint(_) => "Drone is flying to landing point...",
            Task2DroneState::Error => "Drone state is invalid due to wrong transition!",
            Task2DroneState::WaitingForCommand(_) => "Drone is waiting for commands...",
            Task2DroneState::Landing(_) => "Drone is landing...",
            Task2DroneState::Exploring(_) => "Drone is exploring...",
        },
        match &state {
            Task2DroneState::Error => true,
            _ => false,
        },
    );
    *DRONE_STATE.lock().unwrap() = state;
}

pub fn get_drone_state() -> Task2DroneState {
    DRONE_STATE.lock().unwrap().clone()
}
