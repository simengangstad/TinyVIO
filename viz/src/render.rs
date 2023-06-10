use glam::{Quat, Vec3};
use raylib::prelude::*;

pub fn pose(
    draw_handle: &mut RaylibMode3D<RaylibDrawHandle>,
    position: Vec3,
    orientation: Quat,
    scale: f32,
    colors: [Color; 3],
) {
    // Convert to raylib's types
    let orientation = Quaternion::new(orientation.x, orientation.y, orientation.z, orientation.w);
    let position = Vector3::new(position.x, position.y, position.z);

    draw_handle.draw_line_3D(
        position,
        position + Vector3::new(scale, 0.0, 0.0).rotate_by(orientation),
        colors[0],
    );

    draw_handle.draw_line_3D(
        position,
        position + Vector3::new(0.0, scale, 0.0).rotate_by(orientation),
        colors[1],
    );

    draw_handle.draw_line_3D(
        position,
        position + Vector3::new(0.0, 0.0, scale).rotate_by(orientation),
        colors[2],
    );
}
