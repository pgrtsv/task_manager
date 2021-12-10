use lazy_static::initialize;
use rosrust;
use std::thread;
use task_manager::{
    common_ros_utils::*, geometry, msgs::fast_planner_server::FastPlannerGoal, service_clients::*,
    topic_subscribers::*,
};

/// Тестирует алгоритм влёта в проём. Использует сервисы FastPlanner, hole_hunter, pos_collector и motion_controller
/// для обнаружения окна и пролёта через него.
/// ### Пример запуска:
/// ```bash
/// roslaunch task_manager test_fly_into_hole.launch
/// ```
fn main() {
    rosrust::init("test_fly_into_hole");
    let altitude = get_param("~altitude", 1.7);
    let angular_velocity = get_param("~angular_velocity", 0.2);
    let flying_into_hole_detection_distance =
        get_param("~flying_into_hole_detection_distance", 0.3);
    let flying_into_hole_detection_pass_distance =
        get_param("~flying_into_hole_detection_pass_distance", 0.3);
    initialize(&FAST_PLANNER_SERVER_CLIENT);
    initialize(&GET_NEAREST_HOLE);
    initialize(&STOP_SPIN_CLIENT);
    initialize(&SPIN_CLIENT);
    initialize(&TAKEOFF_CLIENT);
    initialize(&DRONE_POSE_SUBSCRIBER);
    initialize(&ADD_VIRTUAL_WALL_CLIENT);

    thread::spawn(move || {
        rosrust::ros_info!("Looking for entry...");
        let mut entry = get_closest_hole();
        while entry.is_none() {
            rosrust::sleep(rosrust::Duration::from_seconds(1));
            entry = get_closest_hole();
        }
        rosrust::ros_info!("Found entry.");
        let entry = entry.unwrap();
        stop_spinning();
        cancel_all_goals();
        rosrust::ros_info!("Flying through entry...");
        FAST_PLANNER_SERVER_CLIENT
            .lock()
            .unwrap()
            .build_goal_sender(FastPlannerGoal {
                header: geometry::new_header("map"),
                pose: geometry::get_entry_in_hole(
                    &entry,
                    &get_current_drone_pose().pose.position,
                    0.3,
                ),
            })
            .send();
        thread::spawn(move || {
            rosrust::ros_info!("Detecting that drone has flown through entry...");
            let rate = rosrust::rate(4.0);
            let drone_position = get_current_drone_pose().pose.position;
            let mut has_drone_flew_through_hole = geometry::has_drone_flew_through_hole(
                geometry::DroneFlewThroughHoleResultParams::New {
                    hole_position: &entry.pose.position,
                    hole_orientation: &entry.pose.orientation,
                    flying_into_hole_pass_distance: flying_into_hole_detection_distance,
                    drone_position: &drone_position,
                    flying_into_hole_detection_pass_distance,
                },
            );

            while rosrust::is_ok() {
                let drone_position = get_current_drone_pose().pose.position;
                has_drone_flew_through_hole = geometry::has_drone_flew_through_hole(
                    geometry::DroneFlewThroughHoleResultParams::PreviousResult {
                        previous_result: has_drone_flew_through_hole,
                        drone_position: &drone_position,
                    },
                );
                if has_drone_flew_through_hole.flew_through {
                    rosrust::ros_info!("Detected that drone has flown through entry.");
                    rosrust::ros_info!("Adding virtual wall and landing...");
                    add_virtual_wall(entry);
                    land();
                    return;
                }
                rate.sleep();
            }
        });
    });
    thread::spawn(move || {
        rosrust::ros_info!("Taking off...");
        takeoff(altitude);
        rosrust::ros_info!("Spinning (looking for entry)...");
        spin(1, altitude, angular_velocity);
    });
    rosrust::spin();
}
