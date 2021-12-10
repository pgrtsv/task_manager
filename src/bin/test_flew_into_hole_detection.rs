use rosrust::*;
use task_manager::{
    common_ros_utils::*,
    geometry::{self, *},
    msgs::{detection_msgs::*, geometry_msgs::*},
    rviz::*,
    topic_subscribers::{get_current_drone_pose, DRONE_POSE_SUBSCRIBER},
};

/// Тестирует алгоритм определения факта влёта дрона в проём. Команды для движения дрона не подаются. При запуске задаётся:
/// - положение окна (аргументы **hole_**);
/// - расстояние влёта **entry_distance** (т. е. на какое расстояние от окна дрон должен влететь внутрь);
/// - расстояние детекции влёта **flying_into_hole_detection_distance** (на каком наибольшем расстоянии от окна дрон должен
/// считать свою позицию стартовой для влета в окно);
/// - минимальное расстояние, которое дрон должен пролететь от проёма в сторону, противоположную стороне влёта, чтобы
/// считать влёт в проём успешным, **flying_into_hole_detection_pass_distance**;
/// - флаг **fix_hole**. Если true, применяется преобразование [hole_dimensions_x, hole_dimensions_y, hole_dimensions_z] =
/// [hole_dimensions_z, hole_dimensions_x, hole_dimensions_y], а hole_orientation поворачивается на 90 градусов по оси y.
/// ### Пример запуска:
/// ```bash
/// roslaunch task_manager test_flew_into_hole_detection
/// ```
fn main() {
    rosrust::init("test_flew_into_hole_detection");
    let hole_position_x = get_param("~hole_position_x", 2.0);
    let hole_position_y = get_param("~hole_position_y", 0.0);
    let hole_position_z = get_param("~hole_position_z", 1.5);
    let hole_orientation_x = get_param("~hole_orientation_x", 0.0);
    let hole_orientation_y = get_param("~hole_orientation_y", 0.0);
    let hole_orientation_z = get_param("~hole_orientation_z", 0.0);
    let hole_orientation_w = get_param("~hole_orientation_w", 1.0);
    let hole_dimensions_x = get_param("~hole_dimensions_x", 0.05);
    let hole_dimensions_y = get_param("~hole_dimensions_y", 1.0);
    let hole_dimensions_z = get_param("~hole_dimensions_z", 1.0);
    let entry_distance = get_param("~entry_distance", 0.3);
    let flying_into_hole_detection_distance =
        get_param("~flying_into_hole_detection_distance", 0.3);
    let flying_into_hole_detection_pass_distance =
        get_param("~flying_into_hole_detection_pass_distance", 0.3);
    let fix_hole = get_param("~fix_hole", false);

    let hole_marker_publisher = publish("/test_flew_into_hole_detection/hole", 1).unwrap();
    let entry_goal_publisher = publish("/test_flew_into_hole_detection/entry", 1).unwrap();

    let mut hole = DetectedObject {
        id: 0,
        pose: Pose {
            position: Point {
                x: hole_position_x,
                y: hole_position_y,
                z: hole_position_z,
            },
            orientation: Quaternion {
                x: hole_orientation_x,
                y: hole_orientation_y,
                z: hole_orientation_z,
                w: hole_orientation_w,
            },
        },
        dimensions: Vector3 {
            x: hole_dimensions_x,
            y: hole_dimensions_y,
            z: hole_dimensions_z,
        },
    };
    if fix_hole {
        geometry::fix_hole(&mut hole);
    }

    lazy_static::initialize(&DRONE_POSE_SUBSCRIBER);
    let rate = rosrust::rate(4.0);
    let drone_position = get_current_drone_pose().pose.position;
    let mut has_drone_flew_through_hole =
        geometry::has_drone_flew_through_hole(DroneFlewThroughHoleResultParams::New {
            hole_position: &hole.pose.position,
            hole_orientation: &hole.pose.orientation,
            flying_into_hole_pass_distance: flying_into_hole_detection_distance,
            drone_position: &drone_position,
            flying_into_hole_detection_pass_distance,
        });

    while rosrust::is_ok() {
        let drone_position = get_current_drone_pose().pose.position;
        has_drone_flew_through_hole = geometry::has_drone_flew_through_hole(
            DroneFlewThroughHoleResultParams::PreviousResult {
                previous_result: has_drone_flew_through_hole,
                drone_position: &drone_position,
            },
        );
        hole_marker_publisher.send(create_clear_marker()).unwrap();
        hole_marker_publisher
            .send(create_cube_marker(
                hole.pose.clone(),
                "map".to_string(),
                hole.dimensions.clone(),
                0,
            ))
            .unwrap();
        let entry = get_entry_in_hole(
            &hole,
            &get_current_drone_pose().pose.position,
            entry_distance,
        );
        entry_goal_publisher
            .send(PoseStamped {
                header: new_header("map"),
                pose: entry,
            })
            .unwrap();
        ros_info!("Flying through state: {:?}", has_drone_flew_through_hole);
        if has_drone_flew_through_hole.flew_through {
            ros_info!("Drone flew through the hole!");
            break;
        }
        rate.sleep();
    }
}
