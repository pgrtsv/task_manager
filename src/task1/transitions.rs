use std::thread;

use crate::{
    events::Failure,
    geometry::{self, default_point, default_quaternion, new_header},
    msgs::{
        detection_msgs::DetectedObject, fast_planner_server::FastPlannerGoal, geometry_msgs::Pose,
        nodes_monitor_msgs::Status,
    },
    service_clients::*,
    task1::{commands::Start, drone_state::*, events::*},
    task_manager,
    topic_publishers::*,
    topic_subscribers::get_current_drone_pose,
};

transitions!(DroneState, [
    (WaitingForCommand, Start) => LookingForEntry,
    (LookingForEntry, EntryFound) => FlyingInside,
    (FlyingInside, FlewInsideBuilding) => Exploring,
    (Exploring, FoundAllCubes) => ReturningToStartPoint,
    (ReturningToStartPoint, FlewNearStartPoint) => Landing,

    (LookingForEntry, Failure) => Landing,
    (FlyingInside, Failure) => ReturningToStartPoint,
    (Exploring, Failure) => ReturningToStartPoint,
    (ReturningToStartPoint, Failure) => ReturningToStartPoint,
    (Landing, Failure) => Landing
]);

impl WaitingForCommand {
    pub fn on_start(self, command: Start) -> LookingForEntry {
        NODES_MONITOR_PUBLISHER
            .send(Status {
                status: Status::STARTED,
            })
            .unwrap();
        thread::spawn(|| {
            let mut cubes = Vec::new();
            let rate = rosrust::rate(1.0);
            while rosrust::is_ok() {
                let old_len = cubes.len();
                let new_len = count_cubes();
                if old_len == new_len {
                    rate.sleep();
                    continue;
                }
                cubes = get_cubes().detected_objects;
                cubes.sort_by(|x, y| x.id.cmp(&y.id));
                for new_cube in cubes.iter().skip(old_len) {
                    publish_new_detected_cube(new_cube.pose.position.clone());
                }

                rate.sleep();
            }
        });
        LookingForEntry::new()
    }
}

impl LookingForEntry {
    pub fn new() -> LookingForEntry {
        thread::spawn(|| {
            let mut entry = get_closest_hole();
            while entry.is_none() && *task_manager::IS_OK.lock().unwrap() {
                rosrust::sleep(rosrust::Duration::from_seconds(1));
                entry = get_closest_hole();
            }
            stop_spinning();
            if !*task_manager::IS_OK.lock().unwrap() {
                return;
            }
            set_drone_state(get_drone_state().on_entry_found(EntryFound::new(entry.unwrap())));
        });
        thread::spawn(|| {
            takeoff(task_manager::OPTIONS.operating_altitude);
        });

        LookingForEntry {}
    }

    pub fn on_entry_found(self, entry_found: EntryFound) -> FlyingInside {
        FlyingInside::new(entry_found.entry)
    }

    pub fn on_failure(self, _: Failure) -> Landing {
        Landing::new()
    }
}

impl Exploring {
    pub fn new() -> Exploring {
        thread::spawn(|| {
            enable_virtual_walls();
            spin_and_wait(
                1,
                task_manager::OPTIONS.operating_altitude,
                task_manager::OPTIONS.angular_velocity,
            );
            start_exploration();
        });
        Exploring {}
    }

    pub fn on_found_all_cubes(self, _: FoundAllCubes) -> ReturningToStartPoint {
        pause_exploration();
        ReturningToStartPoint::new()
    }

    pub fn on_failure(self, _: Failure) -> ReturningToStartPoint {
        pause_exploration();
        ReturningToStartPoint::new()
    }
}

impl FlyingInside {
    pub fn new(entry: DetectedObject) -> FlyingInside {
        cancel_all_goals();
        FAST_PLANNER_SERVER_CLIENT
            .lock()
            .unwrap()
            .build_goal_sender(FastPlannerGoal {
                header: new_header("map"),
                pose: geometry::get_entry_in_hole(
                    &entry,
                    &get_current_drone_pose().pose.position,
                    task_manager::OPTIONS.flying_into_hole_pass_distance,
                ),
            })
            .send();
        thread::spawn(|| {
            let rate = rosrust::rate(4.0);
            let drone_position = get_current_drone_pose().pose.position;
            let mut has_drone_flew_through_hole = geometry::has_drone_flew_through_hole(
                geometry::DroneFlewThroughHoleResultParams::New {
                    hole_position: &entry.pose.position,
                    hole_orientation: &entry.pose.orientation,
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
                    add_virtual_wall(entry);
                    set_drone_state(
                        get_drone_state().on_flew_inside_building(FlewInsideBuilding::new()),
                    );
                    return;
                }
                rate.sleep();
            }
        });

        FlyingInside {}
    }

    pub fn on_flew_inside_building(self, _: FlewInsideBuilding) -> Exploring {
        Exploring::new()
    }

    pub fn on_failure(self, _: Failure) -> ReturningToStartPoint {
        ReturningToStartPoint::new()
    }
}

impl ReturningToStartPoint {
    pub fn new() -> ReturningToStartPoint {
        disable_virtual_walls();
        cancel_all_goals();
        stop_spinning();
        FAST_PLANNER_SERVER_CLIENT
            .lock()
            .unwrap()
            .build_goal_sender(FastPlannerGoal {
                header: new_header("map"),
                pose: Pose {
                    position: default_point(),
                    orientation: default_quaternion(),
                },
            })
            .on_done({
                move |_, _| {
                    set_drone_state(
                        get_drone_state().on_flew_near_start_point(FlewNearStartPoint::new()),
                    );
                }
            })
            .send();
        ReturningToStartPoint {}
    }

    pub fn on_flew_near_start_point(self, event: FlewNearStartPoint) -> Landing {
        Landing::new()
    }

    pub fn on_failure(self, _: Failure) -> ReturningToStartPoint {
        self
    }
}

impl Landing {
    pub fn new() -> Landing {
        land();
        Landing {}
    }

    pub fn on_failure(self, _: Failure) -> Landing {
        self
    }
}
