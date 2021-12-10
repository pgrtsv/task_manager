use std::{
    sync::{Arc, Mutex},
    thread,
};

use crate::{
    common_ros_utils::wait_for_topic,
    geometry::{self, new_header},
    msgs::{
        geometry_msgs::{Point, PoseStamped, Vector3},
        mavros_msgs::PositionTarget,
        nav_msgs::Path,
        nodes_monitor_msgs::Status,
    },
    service_clients::*,
    task3::{commands::Start, drone_state::*},
    task_manager,
    topic_publishers::*,
    topic_subscribers::get_current_drone_pose,
};
use lazy_static::{initialize, lazy_static};

transitions!(Task3DroneState, [
    (WaitingForCommand, Start) => FollowingLine
]);

lazy_static! {
    static ref POSES: Arc<Mutex<Vec<PoseStamped>>> = Arc::new(Mutex::new(Vec::new()));
    static ref LINE_DETECTOR_SUBSCRIBER: rosrust::Subscriber = {
        wait_for_topic("/line_detector_node/line_points");
        rosrust::subscribe("/line_detector_node/line_points", 4, |path: Path| {
            let mut drone_position = get_current_drone_pose().pose.position;
            drone_position.z = 0.0;
            for pose in path.poses {
                let mut pose = transform_pose(pose, "map");
                pose.pose.position.z = 0.0;
                if get_poses_len() == 0 {
                    add_pose(pose);
                    continue;
                }
                if get_poses_len() == 1 {
                    let last_point = get_last_pose().pose.position;
                    if geometry::get_distance_between_points(&drone_position, &last_point)
                        < geometry::get_distance_between_points(
                            &drone_position,
                            &pose.pose.position,
                        )
                    {
                        add_pose(pose);
                    } else {
                        add_start_pose(pose);
                    }
                    continue;
                }
                let previous_pose = get_previous_pose();
                let last_pose = get_last_pose();
                rosrust::ros_warn!(
                    "SIMILAR ORIENTATION: {}, HAS POINT: {}",
                    geometry::has_similar_orientation(
                        &previous_pose.pose.position,
                        &last_pose.pose.position,
                        &pose.pose.position,
                    ),
                    has_point(&pose.pose.position)
                );
                if geometry::has_similar_orientation(
                    &previous_pose.pose.position,
                    &last_pose.pose.position,
                    &pose.pose.position,
                ) && geometry::get_distance_between_points(&drone_position, &pose.pose.position)
                    < 3.5
                    && !has_point(&pose.pose.position)
                {
                    add_pose(pose);
                }
            }
            GLOBAL_PATH_PUBLISHER
                .send(Path {
                    header: new_header("map"),
                    poses: get_poses(),
                })
                .unwrap();
        })
        .unwrap()
    };
    static ref RAW_POINT_PUBLISHER: rosrust::Publisher<PositionTarget> =
        rosrust::publish("/mavros/setpoint_raw/local", 10).unwrap();
    static ref GLOBAL_PATH_PUBLISHER: rosrust::Publisher<Path> =
        rosrust::publish("global_path", 1).unwrap();
}

fn add_pose(pose: PoseStamped) {
    POSES.lock().unwrap().push(pose);
}

fn add_start_pose(pose: PoseStamped) {
    POSES.lock().unwrap().insert(0, pose);
}

fn get_poses_len() -> usize {
    POSES.lock().unwrap().len()
}

fn get_poses() -> Vec<PoseStamped> {
    POSES.lock().unwrap().clone()
}

fn get_last_pose() -> PoseStamped {
    POSES.lock().unwrap().last().unwrap().clone()
}

fn get_previous_pose() -> PoseStamped {
    POSES.lock().unwrap().iter().nth_back(1).unwrap().clone()
}

fn has_point(point: &Point) -> bool {
    POSES
        .lock()
        .unwrap()
        .iter()
        .any(|pose| geometry::get_distance_between_points(&pose.pose.position, point) < 0.5)
}

impl WaitingForCommand {
    pub fn on_start(self, _: Start) -> FollowingLine {
        NODES_MONITOR_PUBLISHER
            .send(Status {
                status: Status::STARTED,
            })
            .unwrap();
        initialize(&RAW_POINT_PUBLISHER);

        FollowingLine::new()
    }
}

impl FollowingLine {
    pub fn new() -> FollowingLine {
        thread::Builder::new()
            .name("follow_line".to_string())
            .spawn(|| {
                initialize(&LINE_DETECTOR_SUBSCRIBER);
                takeoff(task_manager::OPTIONS.operating_altitude);
                let rate = rosrust::rate(20.0);
                let mut index = 0;
                loop {
                    if let Some(pose) = POSES.lock().unwrap().iter().nth(index) {
                        let drone_position = &mut get_current_drone_pose().pose.position;
                        drone_position.z = 0.0;
                        RAW_POINT_PUBLISHER
                            .send(PositionTarget {
                                header: new_header("map"),
                                coordinate_frame: PositionTarget::FRAME_LOCAL_NED,
                                type_mask: PositionTarget::IGNORE_VX
                                    | PositionTarget::IGNORE_VY
                                    | PositionTarget::IGNORE_VZ
                                    | PositionTarget::IGNORE_AFX
                                    | PositionTarget::IGNORE_AFY
                                    | PositionTarget::IGNORE_AFZ,
                                position: {
                                    let mut position = pose.pose.position.clone();
                                    position.z = 1.0;
                                    position
                                },
                                velocity: Vector3::default(),
                                acceleration_or_force: Vector3::default(),
                                yaw: geometry::get_yaw_between_points(
                                    drone_position,
                                    &pose.pose.position,
                                ),
                                yaw_rate: 0.0,
                            })
                            .unwrap();
                        if geometry::get_distance_between_points(
                            drone_position,
                            &pose.pose.position,
                        ) <= 0.2
                        {
                            index += 1;
                        }
                    }
                    rate.sleep();
                }
            })
            .unwrap();
        FollowingLine {}
    }
}
