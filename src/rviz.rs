use crate::{
    geometry::new_header,
    msgs::{
        geometry_msgs::*,
        std_msgs::{ColorRGBA, Header},
        visualization_msgs::Marker,
    },
};

pub fn create_clear_marker() -> Marker {
    Marker {
        header: new_header(""),
        ns: String::default(),
        id: 0,
        type_: Marker::CUBE as i32,
        action: Marker::DELETEALL as i32,
        pose: Pose::default(),
        scale: Vector3::default(),
        color: ColorRGBA {
            r: 0.0,
            g: 0.0,
            b: 0.0,
            a: 0.0,
        },
        lifetime: rosrust::Duration::default(),
        frame_locked: false,
        points: Vec::default(),
        colors: Vec::default(),
        text: String::default(),
        mesh_resource: String::default(),
        mesh_use_embedded_materials: false,
    }
}

pub fn create_cube_marker(pose: Pose, frame_id: String, scale: Vector3, id: i32) -> Marker {
    Marker {
        header: Header {
            seq: 0,
            stamp: rosrust::now(),
            frame_id: frame_id,
        },
        ns: String::default(),
        id,
        type_: Marker::CUBE as i32,
        action: Marker::ADD as i32,
        pose: pose,
        scale: scale,
        color: ColorRGBA {
            r: 0.0,
            g: 1.0,
            b: 1.0,
            a: 1.0,
        },
        lifetime: rosrust::Duration::default(),
        frame_locked: false,
        points: Vec::default(),
        colors: Vec::default(),
        text: String::default(),
        mesh_resource: String::default(),
        mesh_use_embedded_materials: false,
    }
}
