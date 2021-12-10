extern crate nalgebra as na;

use std::fmt::Debug;

use na::{Quaternion, Rotation3, UnitQuaternion, Vector3, Vector4};

use crate::msgs::{
    detection_msgs::DetectedObject,
    geometry_msgs::{self, Point, PointStamped, Pose},
    std_msgs::Header,
};

impl From<Quaternion<f64>> for geometry_msgs::Quaternion {
    fn from(quaternion: Quaternion<f64>) -> Self {
        geometry_msgs::Quaternion {
            x: quaternion.i,
            y: quaternion.j,
            z: quaternion.k,
            w: quaternion.w,
        }
    }
}

impl From<&geometry_msgs::Quaternion> for UnitQuaternion<f64> {
    fn from(quaternion: &geometry_msgs::Quaternion) -> Self {
        UnitQuaternion::new_unchecked(Quaternion::new(
            quaternion.w,
            quaternion.x,
            quaternion.y,
            quaternion.z,
        ))
    }
}

impl From<&geometry_msgs::Point> for Vector3<f64> {
    fn from(point: &geometry_msgs::Point) -> Self {
        Vector3::new(point.x, point.y, point.z)
    }
}

impl From<Vector3<f64>> for Point {
    fn from(point: Vector3<f64>) -> Self {
        Point {
            x: point.x,
            y: point.y,
            z: point.z,
        }
    }
}

/// Возвращает расстояние между двумя точками
pub fn get_distance_between_points(point1: &Point, point2: &Point) -> f64 {
    ((point1.x - point2.x).powi(2) + (point1.y - point2.y).powi(2) + (point1.z - point2.z).powi(2))
        .sqrt()
}

/// Возвращает точку, находящуюся на прямой, образованной точками (0, 0, 0) и point,
/// и отстоящую от point на safe_distance. Если safe_distance > 0, точка находится ближе к (0, 0, 0),
/// если safe_distance == 0, точка равна point, если safe_distance < 0, точка находится дальше от (0, 0, 0)
pub fn get_point_with_safe_distance(point: &Point, safe_distance: f64) -> Point {
    let angle = (point.x / point.y).atan();
    let distance = get_distance_between_points(&default_point(), point) - safe_distance;
    new_point(distance * angle.sin(), distance * angle.cos(), point.z)
}

/// Возвращает новый PointStamped
pub fn new_point_stamped(frame_id: &str, x: f64, y: f64, z: f64) -> PointStamped {
    PointStamped {
        header: new_header(frame_id),
        point: new_point(x, y, z),
    }
}

/// Создаёт новый Point
pub fn new_point(x: f64, y: f64, z: f64) -> Point {
    Point { x, y, z }
}

/// Аналог na::UnitQuaternion::face_towards, но вперёд всегда смотрит ось x, а вверх - ось z.
fn face_towards(dir: &Vector3<f64>) -> UnitQuaternion<f64> {
    let xaxis = dir.normalize();
    let zaxis = Vector3::z();
    let yaxis = zaxis.cross(&xaxis).normalize();

    let rotation: Rotation3<f64> = Rotation3::from_matrix_unchecked(na::SMatrix::<f64, 3, 3>::new(
        xaxis.x.clone(),
        yaxis.x.clone(),
        zaxis.x.clone(),
        xaxis.y.clone(),
        yaxis.y.clone(),
        zaxis.y.clone(),
        xaxis.z.clone(),
        yaxis.z.clone(),
        zaxis.z.clone(),
    ));
    UnitQuaternion::from_rotation_matrix(&rotation)
}

/// Возвращает кватернион - направление к указанной точке
pub fn get_orientation_towards_point(point: &Point) -> geometry_msgs::Quaternion {
    (*face_towards(&point.into()).quaternion()).into()
}

/// Возвращает точку для влета в проём hole на расстояние distance
pub fn get_entry_in_hole(
    hole: &DetectedObject,
    drone_position: &geometry_msgs::Point,
    distance: f64,
) -> Pose {
    let orientation: UnitQuaternion<f64> = (&hole.pose.orientation).into();
    let vector = orientation.transform_vector(&Vector3::new(distance, 0.0, 0.0));
    let hole_center: Vector3<f64> = (&hole.pose.position).into();
    let drone_position: Vector3<f64> = drone_position.into();
    let first_perpendicular_point = hole_center + vector;
    let second_perpendicular_point = hole_center - vector;
    let is_first_point = drone_position.metric_distance(&first_perpendicular_point)
        > drone_position.metric_distance(&second_perpendicular_point);
    Pose {
        position: if is_first_point {
            first_perpendicular_point.into()
        } else {
            second_perpendicular_point.into()
        },
        orientation: {
            let vector = if is_first_point {
                Vector3::new(
                    first_perpendicular_point.index(0) - hole_center.index(0),
                    first_perpendicular_point.index(1) - hole_center.index(1),
                    0.0,
                )
            } else {
                Vector3::new(
                    second_perpendicular_point.index(0) - hole_center.index(0),
                    second_perpendicular_point.index(1) - hole_center.index(1),
                    0.0,
                )
            };
            (*face_towards(&vector).quaternion()).into()
        },
    }
}

/// Возвращает нулевой PointStamped
pub fn default_point_stamped(frame_id: &str) -> PointStamped {
    PointStamped {
        header: new_header(frame_id),
        point: default_point(),
    }
}

/// Возвращает нулевой Quaternion
pub fn default_quaternion() -> geometry_msgs::Quaternion {
    geometry_msgs::Quaternion {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    }
}

/// Возвращает нулевой Point
pub fn default_point() -> Point {
    Point {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }
}

/// Создаёт новый Header
pub fn new_header(frame_id: &str) -> Header {
    Header {
        seq: 0,
        stamp: rosrust::now(),
        frame_id: frame_id.to_string(),
    }
}

#[derive(Debug)]
/// Результат выполнения has_drone_flew_through_hole
pub struct DroneFlewThroughHoleResult {
    /// true, если дрон влетел в окно
    pub flew_through: bool,
    /// None, если расстояние до окна больше distance, иначе - первая точка, с которой дрон начинает влет в окно
    start_point: Option<Vector3<f64>>,
    /// Коэффициенты уравнения плоскости проёма
    plane_vector: Vector4<f64>,
    /// Центр проёма
    hole_center: Vector3<f64>,
    /// Ориентация проёма
    hole_orientation: UnitQuaternion<f64>,
    /// Расстояние до проёма, на котором необходимо выполнять расчёты
    flying_into_hole_pass_distance: f64,
    /// Расстояние от проёма, на котором дрон считается пролетевшим в проём
    flying_into_hole_detection_pass_distance: f64,
}

pub enum DroneFlewThroughHoleResultParams<'a> {
    New {
        hole_position: &'a Point,
        hole_orientation: &'a geometry_msgs::Quaternion,
        flying_into_hole_pass_distance: f64,
        drone_position: &'a Point,
        flying_into_hole_detection_pass_distance: f64,
    },
    PreviousResult {
        previous_result: DroneFlewThroughHoleResult,
        drone_position: &'a Point,
    },
}

/// Возвращает расстояние (высоту) между точкой `point` и плоскостью `plane`.
pub fn get_distance_between_point_and_plane(point: &Vector3<f64>, plane: &na::Vector4<f64>) -> f64 {
    f64::abs(
        plane.index(0) * point.index(0)
            + plane.index(1) * point.index(1)
            + plane.index(2) * point.index(2)
            + plane.index(3),
    ) / f64::sqrt(plane.index(0).powi(2) + plane.index(1).powi(2) + plane.index(2).powi(2))
}

/// Итерационная функция, определяющая, влетел ли дрон в указанный проём.
///
/// ## Пример использования
/// ```rust
/// let rate = rosrust::rate(4.0);
/// let mut has_drone_flew_through_hole = geometry::has_drone_flew_through_hole(
///     geometry::DroneFlewThroughHoleResultParams::New {
///         hole_position: &entry.pose.position,
///         hole_orientation: &entry.pose.orientation,
///         flying_into_hole_pass_distance: flying_into_hole_detection_distance,
///         drone_position: &drone_position,
///         flying_into_hole_detection_pass_distance,
///     },
/// );
/// while rosrust::is_ok() {
///     let drone_position = get_current_drone_pose().pose.position;
///     has_drone_flew_through_hole = geometry::has_drone_flew_through_hole(
///         geometry::DroneFlewThroughHoleResultParams::PreviousResult {
///             previous_result: has_drone_flew_through_hole,
///             drone_position: &drone_position,
///         },
///     );
///     if has_drone_flew_through_hole.flew_through {
///         rosrust::ros_info!("Detected that drone has flown through entry.");
///         break;
///     }
///     rate.sleep();
/// }
/// rosrust::ros_info!("Adding virtual wall and landing...");
/// add_virtual_wall(entry);
/// land();
/// ```
pub fn has_drone_flew_through_hole(
    params: DroneFlewThroughHoleResultParams,
) -> DroneFlewThroughHoleResult {
    let (mut params, drone_position) = match params {
        DroneFlewThroughHoleResultParams::New {
            hole_position,
            hole_orientation,
            flying_into_hole_pass_distance,
            drone_position,
            flying_into_hole_detection_pass_distance,
        } => (
            {
                let hole_orientation: UnitQuaternion<f64> = hole_orientation.into();
                let hole_center: Vector3<f64> = hole_position.into();
                let plane_vector = get_plane(&hole_center, &hole_orientation);
                let drone_position: Vector3<f64> = drone_position.into();
                let distance_to_plane =
                    get_distance_between_point_and_plane(&drone_position, &plane_vector);
                DroneFlewThroughHoleResult {
                    flew_through: false,
                    start_point: if distance_to_plane > flying_into_hole_pass_distance {
                        None
                    } else {
                        Some(drone_position)
                    },
                    plane_vector,
                    hole_center: hole_position.into(),
                    hole_orientation,
                    flying_into_hole_pass_distance,
                    flying_into_hole_detection_pass_distance,
                }
            },
            drone_position.into(),
        ),
        DroneFlewThroughHoleResultParams::PreviousResult {
            mut previous_result,
            drone_position,
        } => {
            let drone_position = drone_position.into();
            if previous_result.start_point.is_none()
                && get_distance_between_point_and_plane(
                    &drone_position,
                    &previous_result.plane_vector,
                ) <= previous_result.flying_into_hole_pass_distance
            {
                previous_result.start_point = Some(drone_position);
            }
            (previous_result, drone_position)
        }
    };

    if let Some(start_point) = params.start_point {
        params.flew_through =
            are_points_opposite(&start_point, &drone_position, &params.plane_vector)
                && get_distance_between_point_and_plane(&drone_position, &params.plane_vector)
                    >= params.flying_into_hole_detection_pass_distance
    };

    params
}

/// Применяет к указанному проёму `hole` следующие преобразования:
/// * [`hole.dimensions.x`, `hole.dimensions.y`, `hole.dimensions.z`] = [`hole.dimensions.z`, `hole.dimensions.x`, `hole.dimensions.y`];
/// * `hole.orientation` поворачивается на 90 градусов по оси y.
///
/// Этот метод необходимо применять к проёмам, полученным из *hole_hunter*, либо из соответствующего ему *pos_collector*,
/// так как вместо нормали к плоскости проёма *hole_hunter* в качестве ориентации передаёт коллинеарный плоскости кватернион,
/// а вместо принятой в ROS координатной системы (x - вперед/глубина проёма, y - влево/ширина проёма, z - вверх/высота проёма)
/// размеры окна передаются в виде x - ширина проёма, y - высота проёма, z - глубина проёма.
pub fn fix_hole(hole: &mut DetectedObject) {
    hole.dimensions = geometry_msgs::Vector3 {
        x: hole.dimensions.z,
        y: hole.dimensions.x,
        z: hole.dimensions.y,
    };
    let orientation: UnitQuaternion<f64> = (&hole.pose.orientation).into();
    let orientation = orientation * UnitQuaternion::from_euler_angles(0.0, 1.57, 0.0);
    hole.pose.orientation = (*orientation.quaternion()).into();
}

/// Возвращает коэффициенты общего уравнения плоскости (Ax + By + Cz + D = 0).
/// * `point` - точка на плоскости;
/// * `normal` - нормаль к плоскости.
fn get_plane(point: &Vector3<f64>, normal: &UnitQuaternion<f64>) -> Vector4<f64> {
    na::Vector4::new(
        normal[0],
        normal[1],
        normal[2],
        -normal[0] * point[0] - normal[1] * point[1] - normal[2] * point[2],
    )
}

/// Возвращает `true`, если точки `point_1` и `point_2` лежат по разные стороны плоскости `plane`.  
fn are_points_opposite(
    point_1: &Vector3<f64>,
    point_2: &Vector3<f64>,
    plane: &Vector4<f64>,
) -> bool {
    let point_1_scalar = plane.index(0) * point_1.index(0)
        + plane.index(1) * point_1.index(1)
        + plane.index(2) * point_1.index(2)
        + plane.index(3);
    let point_2_scalar = plane.index(0) * point_2.index(0)
        + plane.index(1) * point_2.index(1)
        + plane.index(2) * point_2.index(2)
        + plane.index(3);
    (point_1_scalar > 0.0 && point_2_scalar < 0.0) || (point_1_scalar < 0.0 && point_2_scalar > 0.0)
}

/// Возвращает `true`, если вектора `previous_point`->`current_point` и `current_point`->`next_point` имеют схожее
/// направление (ориентацию).
///
/// *Детали реализации*: из `previous_point` в `current_point` строится нормаль. По этой нормали в точке `current_point`
/// строится плоскость. Если точки `previous_point` и `next_point` лежат по разные стороны этой плоскости, возвращается true.
pub fn has_similar_orientation(
    previous_point: &Point,
    current_point: &Point,
    next_point: &Point,
) -> bool {
    let previous_point: Vector3<f64> = previous_point.into();
    let current_point: Vector3<f64> = current_point.into();
    let next_point: Vector3<f64> = next_point.into();
    let previous_orientation = UnitQuaternion::rotation_between(
        &Vector3::new(1.0, 0.0, 0.0),
        &(current_point - previous_point),
    );
    if let Some(previous_orientation) = previous_orientation {
        let plane = get_plane(&current_point, &previous_orientation);
        are_points_opposite(&previous_point, &next_point, &plane)
    } else {
        false
    }
}

/// Возвращает угол поворота по оси Z из точки `from_point` в точку `to_point`.
pub fn get_yaw_between_points(from_point: &Point, to_point: &Point) -> f32 {
    let from_point: Vector3<f64> = from_point.into();
    let to_point: Vector3<f64> = to_point.into();
    match UnitQuaternion::rotation_between(&Vector3::new(1.0, 0.0, 0.0), &(to_point - from_point)) {
        Some(rotation) => rotation.euler_angles().2 as f32,
        None => 3.14,
    }
}
