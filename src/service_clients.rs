use crate::{
    common_ros_utils::*,
    geometry::{self, default_point, default_quaternion, new_header},
    msgs::{
        autotakeoff::*,
        detection_msgs::*,
        fast_planner_server::FastPlannerAction,
        geometry_msgs::{PointStamped, Pose, PoseStamped},
        motion_controller::*,
        plan_env::*,
        pos_collector_msgs::*,
        std_srvs::{Empty, EmptyReq, SetBool, SetBoolReq},
        transform_services::*,
    },
    task_manager::TaskType,
};
use lazy_static::{initialize, lazy_static};
use rosrust::{self, Client, Duration};
use rosrust_actionlib::SimpleActionClient;
use std::sync::Mutex;

lazy_static! {
    pub static ref FAST_PLANNER_SERVER_CLIENT: Mutex<SimpleActionClient<FastPlannerAction>> = {
        let fast_planner_client = SimpleActionClient::new("fast_planner_server").unwrap();
        wait_for_actionlib_server(&fast_planner_client, "fast_planner_server");
        Mutex::new(fast_planner_client)
    };
    pub static ref USE_FUEL_CLIENT: Client<SetBool> = {
        wait_for_service("fuel/use_fuel");
        rosrust::client("fuel/use_fuel").unwrap()
    };
    pub static ref GET_NEAREST_HOLE: Client<NearestPos> = {
        wait_for_service("vision/holes/pos_collector/get_nearest");
        rosrust::client("vision/holes/pos_collector/get_nearest").unwrap()
    };
    pub static ref RESET_FUEL_CLIENT: Client<Empty> = {
        wait_for_service("fuel/reset");
        rosrust::client("fuel/reset").unwrap()
    };
    pub static ref SPIN_CLIENT: Client<Spin> = {
        wait_for_service("motion_controller/spin");
        rosrust::client("motion_controller/spin").unwrap()
    };
    pub static ref STOP_SPIN_CLIENT: Client<Empty> = {
        wait_for_service("motion_controller/stop");
        rosrust::client("motion_controller/stop").unwrap()
    };
    pub static ref TAKEOFF_CLIENT: Client<Takeoff> = {
        wait_for_service("takeoff_landing");
        rosrust::client("takeoff_landing").unwrap()
    };
    pub static ref ADD_VIRTUAL_WALL_CLIENT: Client<AddWalls> = {
        wait_for_service("sdf_map/add_wall");
        rosrust::client("sdf_map/add_wall").unwrap()
    };
    pub static ref SET_ARE_WALLS_ENABLED_CLIENT: Client<SetBool> = {
        wait_for_service("sdf_map/set_are_walls_enabled");
        rosrust::client("sdf_map/set_are_walls_enabled").unwrap()
    };
    pub static ref COUNT_CUBES_CLIENT: Client<Count> = {
        wait_for_service("vision/cubes/pos_collector/count");
        rosrust::client("vision/cubes/pos_collector/count").unwrap()
    };
    pub static ref GET_ALL_CUBES_CLIENT: Client<GetAll> = {
        wait_for_service("vision/cubes/pos_collector/get_all");
        rosrust::client("vision/cubes/pos_collector/get_all").unwrap()
    };
    pub static ref TRANSFORM_POINT_CLIENT: Client<TransformPoint> = {
        wait_for_service("transform/point");
        rosrust::client::<TransformPoint>("transform/point").unwrap()
    };
    pub static ref TRANSFORM_POSE_CLIENT: Client<TransformPose> = {
        wait_for_service("transform/pose");
        rosrust::client::<TransformPose>("transform/pose").unwrap()
    };
    pub static ref COUNT_HOLES_CLIENT: Client<Count> = {
        wait_for_service("vision/holes/pos_collector/count");
        rosrust::client("vision/holes/pos_collector/count").unwrap()
    };
    pub static ref GET_ALL_HOLES_CLIENT: Client<GetAll> = {
        wait_for_service("vision/holes/pos_collector/get_all");
        rosrust::client("vision/holes/pos_collector/get_all").unwrap()
    };
}

/// Ожидает и инициализирует всех клиентов сервисов, использующихся узлом.
pub fn init(task_type: TaskType) {
    match task_type {
        TaskType::One => {
            initialize(&FAST_PLANNER_SERVER_CLIENT);
            initialize(&USE_FUEL_CLIENT);
            initialize(&GET_NEAREST_HOLE);
            initialize(&RESET_FUEL_CLIENT);
            initialize(&SPIN_CLIENT);
            initialize(&STOP_SPIN_CLIENT);
            initialize(&TAKEOFF_CLIENT);
            initialize(&ADD_VIRTUAL_WALL_CLIENT);
            initialize(&SET_ARE_WALLS_ENABLED_CLIENT);
            initialize(&COUNT_CUBES_CLIENT);
            initialize(&GET_ALL_CUBES_CLIENT);
        }
        TaskType::Two => {
            initialize(&FAST_PLANNER_SERVER_CLIENT);
            initialize(&USE_FUEL_CLIENT);
            initialize(&GET_NEAREST_HOLE);
            initialize(&RESET_FUEL_CLIENT);
            initialize(&SPIN_CLIENT);
            initialize(&STOP_SPIN_CLIENT);
            initialize(&TAKEOFF_CLIENT);
            initialize(&ADD_VIRTUAL_WALL_CLIENT);
            initialize(&SET_ARE_WALLS_ENABLED_CLIENT);
            initialize(&COUNT_HOLES_CLIENT);
            initialize(&GET_ALL_HOLES_CLIENT);
        }
        TaskType::Three => {
            initialize(&TAKEOFF_CLIENT);
        }
    }
}

/// Подаёт дрону команду на взлёт. Блокирует вызывающий поток, пока дрон не достигнет высоты `height`.
pub fn takeoff(height: f32) {
    TAKEOFF_CLIENT
        .req(&TakeoffReq {
            height,
            land: false,
        })
        .unwrap()
        .unwrap();
}

/// Подаёт дрону команду на приземление в текущей точке. Блокирует вызывающий поток, пока дрон не приземлится.
pub fn land() {
    TAKEOFF_CLIENT
        .req(&TakeoffReq {
            height: 0.0,
            land: true,
        })
        .unwrap()
        .unwrap();
}

/// Сбрасывает карту и ранее построенные маршруты FUEL.
pub fn reset_fuel() {
    RESET_FUEL_CLIENT.req(&EmptyReq {}).unwrap().unwrap();
}

/// Отменяет все цели FastPlanner.
pub fn cancel_all_goals() {
    FAST_PLANNER_SERVER_CLIENT
        .lock()
        .unwrap()
        .cancel_all_goals()
        .unwrap();
    rosrust::sleep(Duration::from_seconds(1));
}

/// Вращает дрона вокруг оси Z по часовой стрелке `laps_count` полных оборотов на высоте `altitude` м с угловой скоростью
/// `angular_velocity` м/с. Не блокирует вызывающий поток.
pub fn spin(laps_count: i32, altitude: f32, angular_velocity: f32) {
    SPIN_CLIENT.req_async(SpinReq {
        laps_count,
        altitude,
        angular_velocity,
    });
}

/// См. `service_clients::spin`. Блокирует вызывающий поток, пока дрон не прекратит вращение.
pub fn spin_and_wait(laps_count: i32, altitude: f32, angular_velocity: f32) {
    SPIN_CLIENT
        .req(&SpinReq {
            laps_count,
            altitude,
            angular_velocity,
        })
        .unwrap()
        .unwrap();
}

/// Останавливает вращение.
pub fn stop_spinning() {
    STOP_SPIN_CLIENT.req(&EmptyReq {}).unwrap().unwrap();
}

/// Начинает автономное исследование, переключает FastPlanner на FUEL.
pub fn start_exploration() {
    USE_FUEL_CLIENT
        .req(&SetBoolReq { data: true })
        .unwrap()
        .unwrap();
}

/// Останавливает автономное исследование, переключая FUEL на FastPlanner.
pub fn pause_exploration() {
    USE_FUEL_CLIENT
        .req(&SetBoolReq { data: false })
        .unwrap()
        .unwrap();
}

/// Возвращает ближайший к дрону обнаруженный проём. Если ещё не найден ни один проём, возвращает `None`.
pub fn get_closest_hole() -> Option<DetectedObject> {
    match GET_NEAREST_HOLE.req(&NearestPosReq {
        myPose: PoseStamped {
            header: new_header("base_link"),
            pose: Pose {
                position: default_point(),
                orientation: default_quaternion(),
            },
        },
    }) {
        Ok(result) => match result {
            Ok(mut result) => {
                let mut result = result.nearestObj.detected_objects.swap_remove(0);
                geometry::fix_hole(&mut result);
                Some(result)
            }
            Err(_) => None,
        },
        Err(_) => None,
    }
}

/// Возвращает все обнаруженные кубы.
pub fn get_cubes() -> DetectedObjects {
    GET_ALL_CUBES_CLIENT
        .req(&GetAllReq {})
        .unwrap()
        .unwrap()
        .objects
}

/// Возвращает количество обнаруженных кубов.
pub fn count_cubes() -> usize {
    COUNT_CUBES_CLIENT.req(&CountReq {}).unwrap().unwrap().count as usize
}

/// Возвращает все обнаруженные проёмы.
pub fn get_holes() -> DetectedObjects {
    let mut holes = GET_ALL_HOLES_CLIENT
        .req(&GetAllReq {})
        .unwrap()
        .unwrap()
        .objects;
    for hole in holes.detected_objects.iter_mut() {
        geometry::fix_hole(hole);
    }
    holes
}

/// Возвращает количество обнаруженных проёмов.
pub fn count_holes() -> usize {
    COUNT_HOLES_CLIENT.req(&CountReq {}).unwrap().unwrap().count as usize
}

/// Добавляет виртуальную стену `wall` в FastPlanner и FUEL. `wall` должен подаваться в СК "map".
pub fn add_virtual_wall(wall: DetectedObject) {
    ADD_VIRTUAL_WALL_CLIENT
        .req(&AddWallsReq {
            objects: DetectedObjects {
                header: new_header("map"),
                detected_objects: vec![wall],
            },
        })
        .unwrap()
        .unwrap();
}

/// Включает виртуальные стены в FastPlanner и FUEL.
pub fn enable_virtual_walls() {
    SET_ARE_WALLS_ENABLED_CLIENT
        .req(&SetBoolReq { data: true })
        .unwrap()
        .unwrap();
}

/// Отключает виртуальные стены в FastPlanner и FUEL.
pub fn disable_virtual_walls() {
    SET_ARE_WALLS_ENABLED_CLIENT
        .req(&SetBoolReq { data: false })
        .unwrap()
        .unwrap();
}

/// Возвращает точку `point` в указанной СК `target_frame_id`.
/// Вызывает панику, если трансформация невозможна.
pub fn transform_point(point: PointStamped, target_frame_id: &str) -> PointStamped {
    TRANSFORM_POINT_CLIENT
        .req(&TransformPointReq {
            point,
            target_frame_id: target_frame_id.to_string(),
        })
        .unwrap()
        .unwrap()
        .point
}

/// Возвращает позу `pose` в указанной СК `target_frame_id`.
/// Вызывает панику, если трансформация невозможна.
pub fn transform_pose(pose: PoseStamped, target_frame_id: &str) -> PoseStamped {
    TRANSFORM_POSE_CLIENT
        .req(&TransformPoseReq {
            pose,
            target_frame_id: target_frame_id.to_string(),
        })
        .unwrap()
        .unwrap()
        .pose
}
