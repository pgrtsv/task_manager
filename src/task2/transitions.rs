use std::thread;

use crate::{
    common_ros_utils::wait_for_topic,
    events::Failure,
    geometry::{self, new_header},
    msgs::{
        detection_msgs::DetectedObject,
        fast_planner_server::FastPlannerGoal,
        geometry_msgs::{Point, PointStamped, Pose},
        nodes_monitor_msgs::Status,
        qr_detector_msgs::QRCodeArray,
    },
    service_clients::*,
    task2::{
        commands::Start,
        drone_state::{self, *},
        events::*,
    },
    task_manager,
    topic_publishers::*,
    topic_subscribers::get_current_drone_pose,
};
use lazy_static::{initialize, lazy_static};

transitions!(Task2DroneState, [
    (WaitingForCommand, Start) => Exploring,
    (WaitingForCommand, QrFound) => WaitingForCommand,
    (WaitingForCommand, HoleFound) => WaitingForCommand,
    (Exploring, QrFound) => [Exploring, FlyingIntoHole, FlyingToLandingPoint],
    (Exploring, HoleFound) => [Exploring, FlyingIntoHole],
    (FlyingIntoHole, QrFound) => FlyingIntoHole,
    (FlyingIntoHole, HoleFound) => FlyingIntoHole,
    (FlyingIntoHole, FlewThroughHole) => Exploring,
    (FlyingToLandingPoint, QrFound) => FlyingToLandingPoint,
    (FlyingToLandingPoint, HoleFound) => FlyingToLandingPoint,
    (FlyingToLandingPoint, FlewNearLandingPoint) => Landing,
    (Landing, QrFound) => Landing,
    (Landing, HoleFound) => Landing,

    (Exploring, Failure) => Landing,
    (FlyingIntoHole, Failure) => Landing,
    (FlyingToLandingPoint, Failure) => FlyingToLandingPoint,
    (Landing, Failure) => Landing
]);

lazy_static! {
    pub static ref QR_CODES_SUBSCRIBER: rosrust::Subscriber = {
        wait_for_topic("vision/qr_codes");
        rosrust::subscribe("vision/qr_codes", 1, |qrs: QRCodeArray| {
            if !*task_manager::IS_OK.lock().unwrap() || qrs.qr_codes.is_empty() {
                return;
            }
            for detected_qr in qrs.qr_codes {
                let qr_point = transform_point(
                    PointStamped {
                        header: detected_qr.header,
                        point: Point {
                            x: detected_qr.position.x,
                            y: detected_qr.position.y,
                            z: detected_qr.position.z,
                        },
                    },
                    "map",
                )
                .point;
                let qr = Qr::new(qr_point, detected_qr.data);
                if !drone_state::is_qr_already_detected(&qr) {
                    let qr_index = drone_state::add_qr(qr.clone());
                    let connected_hole = drone_state::find_connected_hole(&qr);
                    if let Some(connected_hole) = connected_hole {
                        drone_state::add_hole_qr_connection(connected_hole.id as usize, qr_index);
                    }
                    set_drone_state(get_drone_state().on_qr_found(QrFound::new(qr, qr_index)));
                }
            }
        })
        .unwrap()
    };
}

impl WaitingForCommand {
    pub fn on_start(self, command: Start) -> Exploring {
        NODES_MONITOR_PUBLISHER
            .send(Status {
                status: Status::STARTED,
            })
            .unwrap();

        // Подписчик следит за обнаруженными QR-кодами, и при получении новых добавляет их в drone_state::DETECTED_QR_CODES.
        // Также по возможности обнаруживает и добавляет связь с ранее найденным проёмом в
        // drone_state::DETECTED_HOLE_QR_CONNECTIONS. Вызывает событие on_qr_found()
        initialize(&QR_CODES_SUBSCRIBER);

        // Поток следит за изменением количества обнаруженных проёмов в pos_collector и при добавлении новых
        // автоматически добавляет их в drone_state::DETECTED_HOLES. Также по возможности обнаруживает и добавляет связь с
        // ранее найденным QR-кодом в drone_state::DETECTED_HOLE_QR_CONNECTIONS. Вызывает событие on_hole_found()
        thread::Builder::new()
            .name("watch_pos_collector_holes_changes".to_string())
            .spawn(|| {
                let mut count = 0;
                let rate = rosrust::rate(1.0);
                while rosrust::is_ok() && *task_manager::IS_OK.lock().unwrap() {
                    let new_count = count_holes();
                    if count == new_count {
                        rate.sleep();
                        continue;
                    }
                    // TODO: гарантировать, что pos_collector возвращает упорядоченные по id объекты
                    let holes = get_holes();
                    for new_hole in holes.detected_objects.iter().skip(count) {
                        let connected_qr = drone_state::find_connected_qr(new_hole);
                        if let Some(connected_qr) = connected_qr {
                            drone_state::add_hole_qr_connection(new_hole.id as usize, connected_qr);
                        }
                        set_drone_state(
                            get_drone_state().on_hole_found(HoleFound::new(new_hole.clone())),
                        );
                    }
                    *DETECTED_HOLES.lock().unwrap() = holes;
                    count = new_count;
                    rate.sleep();
                }
            })
            .unwrap();

        Exploring::start()
    }

    pub fn on_qr_found(self, qr_found: QrFound) -> Self {
        self
    }

    pub fn on_hole_found(self, hole_found: HoleFound) -> Self {
        self
    }
}

impl Exploring {
    pub fn start() -> Exploring {
        thread::Builder::new()
            .name("start_exploring".to_string())
            .spawn(|| {
                takeoff(task_manager::OPTIONS.operating_altitude);
                if !*task_manager::IS_OK.lock().unwrap()
                    || !matches!(
                        drone_state::get_drone_state(),
                        Task2DroneState::Exploring(_)
                    )
                {
                    return;
                }
                spin_and_wait(
                    1,
                    task_manager::OPTIONS.low_altitude,
                    task_manager::OPTIONS.angular_velocity,
                );
                if !*task_manager::IS_OK.lock().unwrap()
                    || !matches!(
                        drone_state::get_drone_state(),
                        Task2DroneState::Exploring(_)
                    )
                {
                    return;
                }
                spin_and_wait(
                    1,
                    task_manager::OPTIONS.operating_altitude,
                    task_manager::OPTIONS.angular_velocity,
                );
                if !*task_manager::IS_OK.lock().unwrap()
                    || !matches!(
                        drone_state::get_drone_state(),
                        Task2DroneState::Exploring(_)
                    )
                {
                    return;
                }
                start_exploration();
            })
            .unwrap();

        Exploring {}
    }

    pub fn go_on(qr_index: usize) -> Exploring {
        {
            let detected_qr_codes = &mut *DETECTED_QR_CODES.lock().unwrap();
            DETECTED_HOLE_QR_CONNECTIONS.lock().unwrap().clear();
            PASSED_ROOMS_NUMBERS.lock().unwrap().push(
                detected_qr_codes
                    .iter()
                    .nth(qr_index)
                    .unwrap()
                    .content
                    .clone(),
            );
            detected_qr_codes.clear();
        }
        enable_virtual_walls();
        thread::Builder::new()
            .name("go_on_exploring".to_string())
            .spawn(|| {
                spin_and_wait(
                    1,
                    task_manager::OPTIONS.low_altitude,
                    task_manager::OPTIONS.angular_velocity,
                );
                if !*task_manager::IS_OK.lock().unwrap()
                    || !matches!(
                        drone_state::get_drone_state(),
                        Task2DroneState::Exploring(_)
                    )
                {
                    return;
                }
                spin_and_wait(
                    1,
                    task_manager::OPTIONS.operating_altitude,
                    task_manager::OPTIONS.angular_velocity,
                );
                if !*task_manager::IS_OK.lock().unwrap()
                    || !matches!(
                        drone_state::get_drone_state(),
                        Task2DroneState::Exploring(_)
                    )
                {
                    return;
                }
                start_exploration();
            })
            .unwrap();
        Exploring {}
    }

    pub fn on_qr_found(self, qr_found: QrFound) -> Task2DroneState {
        if qr_found.qr.is_on_floor {
            if let Some(hole) = match_qr_index_with_hole(qr_found.index) {
                return Task2DroneState::FlyingIntoHole(FlyingIntoHole::new(hole, qr_found.index));
            }
            if let Some(qr) = match_qr_index_with_passed_rooms(qr_found.index) {
                return Task2DroneState::FlyingToLandingPoint(FlyingToLandingPoint::new(
                    qr.position,
                ));
            }
            return Task2DroneState::Exploring(self);
        }
        rosrust::ros_info!("MATHCING!");
        if let Some(hole_qr_index) = match_qr_index_with_qr_on_floor(qr_found.index) {
            if let Some(hole) = match_qr_index_with_hole(hole_qr_index) {
                return Task2DroneState::FlyingIntoHole(FlyingIntoHole::new(hole, hole_qr_index));
            }
        }
        Task2DroneState::Exploring(self)
    }

    pub fn on_hole_found(self, hole_found: HoleFound) -> Task2DroneState {
        if let Some(qr_index) = match_hole_id_with_qr_on_floor(hole_found.hole.id as usize) {
            return Task2DroneState::FlyingIntoHole(FlyingIntoHole::new(hole_found.hole, qr_index));
        }
        Task2DroneState::Exploring(self)
    }

    pub fn on_failure(self, failure: Failure) -> Landing {
        Landing::new()
    }
}

impl FlyingIntoHole {
    pub fn new(hole: DetectedObject, qr_index: usize) -> FlyingIntoHole {
        cancel_all_goals();
        pause_exploration();
        let drone_position = get_current_drone_pose().pose.position;
        FAST_PLANNER_SERVER_CLIENT
            .lock()
            .unwrap()
            .build_goal_sender(FastPlannerGoal {
                header: new_header("map"),
                pose: geometry::get_entry_in_hole(
                    &hole,
                    &drone_position,
                    task_manager::OPTIONS.flying_into_hole_pass_distance,
                ),
            })
            .send();
        thread::spawn(move || {
            let rate = rosrust::rate(4.0);
            let drone_position = get_current_drone_pose().pose.position;
            let mut has_drone_flew_through_hole = geometry::has_drone_flew_through_hole(
                geometry::DroneFlewThroughHoleResultParams::New {
                    hole_position: &hole.pose.position,
                    hole_orientation: &hole.pose.orientation,
                    flying_into_hole_pass_distance: task_manager::OPTIONS
                        .flying_into_hole_detection_distance
                        as f64,
                    flying_into_hole_detection_pass_distance: task_manager::OPTIONS
                        .flying_into_hole_detection_pass_distance,
                    drone_position: &drone_position,
                },
            );

            while rosrust::is_ok() && *task_manager::IS_OK.lock().unwrap() {
                let drone_position = get_current_drone_pose().pose.position;
                has_drone_flew_through_hole = geometry::has_drone_flew_through_hole(
                    geometry::DroneFlewThroughHoleResultParams::PreviousResult {
                        previous_result: has_drone_flew_through_hole,
                        drone_position: &drone_position,
                    },
                );
                if has_drone_flew_through_hole.flew_through {
                    add_virtual_wall(hole);
                    set_drone_state(
                        get_drone_state().on_flew_through_hole(FlewThroughHole::new(qr_index)),
                    );
                    return;
                }
                rate.sleep();
            }
        });
        FlyingIntoHole {}
    }

    pub fn on_qr_found(self, qr_found: QrFound) -> FlyingIntoHole {
        self
    }

    pub fn on_hole_found(self, hole_found: HoleFound) -> FlyingIntoHole {
        self
    }

    pub fn on_flew_through_hole(self, flew_through_hole: FlewThroughHole) -> Exploring {
        Exploring::go_on(flew_through_hole.qr_index)
    }

    pub fn on_failure(self, failure: Failure) -> Landing {
        Landing::new()
    }
}

impl FlyingToLandingPoint {
    pub fn new(landing_point: Point) -> FlyingToLandingPoint {
        cancel_all_goals();
        let orientation = geometry::get_orientation_towards_point(&landing_point);
        FAST_PLANNER_SERVER_CLIENT
            .lock()
            .unwrap()
            .build_goal_sender(FastPlannerGoal {
                header: new_header("map"),
                pose: Pose {
                    position: landing_point,
                    orientation,
                },
            })
            .on_done(|_, _| {
                set_drone_state(
                    get_drone_state().on_flew_near_landing_point(FlewNearLandingPoint::new()),
                )
            })
            .send();
        FlyingToLandingPoint {}
    }

    pub fn on_flew_near_landing_point(
        self,
        flew_near_landing_point: FlewNearLandingPoint,
    ) -> Landing {
        Landing::new()
    }

    pub fn on_qr_found(self, qr_found: QrFound) -> Self {
        self
    }

    pub fn on_hole_found(self, hole_found: HoleFound) -> Self {
        self
    }

    pub fn on_failure(self, failure: Failure) -> Self {
        self
    }
}

impl Landing {
    pub fn new() -> Landing {
        cancel_all_goals();
        land();
        Landing {}
    }

    pub fn on_qr_found(self, qr_found: QrFound) -> Self {
        self
    }

    pub fn on_hole_found(self, hole_found: HoleFound) -> Self {
        self
    }

    pub fn on_failure(self, failure: Failure) -> Self {
        self
    }
}
