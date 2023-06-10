mod data_interface;
mod data_point;
mod euroc_loader;
mod header;
mod render;
mod serialisable;

use clap::Parser;
use data_interface::DataInterface;
use raylib::{core::texture::*, prelude::*};
use std::{
    net::{IpAddr, Ipv4Addr, SocketAddr},
    sync::{mpsc::channel, Arc, Mutex},
    thread,
};

use glam::{Quat, Vec3};

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    dataset_path: String,
}

fn main() {
    let args = Args::parse();

    let orig_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |panic_info| {
        // invoke the default handler and exit the process
        orig_hook(panic_info);
        std::process::exit(1);
    }));

    env_logger::init();

    let (image_data_point_transmitter, image_data_point_receiver) = channel();
    let (pose_data_point_transmitter, pose_data_point_receiver) = channel();
    let (ground_truth_data_point_transmitter, ground_truth_data_point_receiver) = channel();

    let (_, message_receiver) = channel();

    let keypoint_data = Arc::new(Mutex::new(Vec::new()));

    let message_receiver_mutex = Arc::new(Mutex::new(message_receiver));

    let data_interface = DataInterface::new(
        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(192, 168, 0, 100)), 5001),
        image_data_point_transmitter,
        pose_data_point_transmitter,
        ground_truth_data_point_transmitter,
        message_receiver_mutex,
        Arc::clone(&keypoint_data),
        &args.dataset_path,
    );

    thread::spawn(move || data_interface.start());

    let width = 1080;
    let height = 720;

    let (mut raylib_handle, thread) = raylib::init().size(width, height).title("Iron").build();

    let mut camera = Camera3D::perspective(
        Vector3::new(-10.0, -10.0, -10.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, -1.0, 0.0),
        45.0,
    );

    raylib_handle.set_camera_mode(camera, CameraMode::CAMERA_FREE);
    raylib_handle.set_camera_alt_control(KeyboardKey::KEY_LEFT_SHIFT);
    raylib_handle.set_target_fps(60);

    let default_pose_colors: [Color; 3] = [
        Color::new(255, 0, 0, 255),
        Color::new(0, 255, 0, 255),
        Color::new(0, 0, 255, 255),
    ];

    let ground_truth_pose_colors: [Color; 3] = [
        Color::new(0, 0, 0, 255),
        Color::new(0, 0, 0, 255),
        Color::new(0, 0, 0, 255),
    ];

    let mut texture: Option<Texture2D> = None;

    let mut estimated_position = Vec3::ZERO;
    let mut estimated_orientation = Quat::IDENTITY;

    let mut ground_truth_position = Vec3::ZERO;
    let mut ground_truth_orientation = Quat::IDENTITY;

    let q_worldaxes_eurocreferenceaxes =
        glam::Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), PI as f32 / 2.0);

    let q_worldaxes_eurocbodyaxes =
        glam::Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), -PI as f32 / 2.0);

    while !raylib_handle.window_should_close() {
        // For the following channels, if the application has been in the background the update
        // loop wouldn't have triggered, so elements have piled up in the channels. We loop through
        // them to receive the latest element

        loop {
            match image_data_point_receiver.try_recv() {
                Ok(image_data_point) => {
                    if texture.is_none() {
                        let mut image = Image::gen_image_color(752, 480, Color::new(0, 0, 0, 255));
                        image.set_format(PixelFormat::PIXELFORMAT_UNCOMPRESSED_GRAYSCALE);

                        texture = Some(
                            raylib_handle
                                .load_texture_from_image(&thread, &image)
                                .expect("Failed to load texture"),
                        );
                    }

                    texture
                        .as_mut()
                        .unwrap()
                        .update_texture(image_data_point.image.as_bytes());
                }
                Err(_) => break,
            }
        }

        loop {
            match pose_data_point_receiver.try_recv() {
                Ok(pose) => {
                    estimated_position = pose.position;
                    estimated_orientation = pose.orientation;
                }
                Err(_) => break,
            }
        }

        loop {
            match ground_truth_data_point_receiver.try_recv() {
                Ok(ground_truth) => {
                    ground_truth_position = q_worldaxes_eurocreferenceaxes * ground_truth.position;
                    ground_truth_orientation = q_worldaxes_eurocreferenceaxes
                        * ground_truth.orientation
                        * q_worldaxes_eurocbodyaxes.conjugate();
                }
                Err(_) => break,
            }
        }

        raylib_handle.update_camera(&mut camera);

        let mut draw_handle = raylib_handle.begin_drawing(&thread);

        draw_handle.clear_background(Color::WHITE);

        // 3D context
        {
            let mut mode = draw_handle.begin_mode3D(camera);
            mode.draw_grid(80, 1.0);

            // Draw the coordinate frame
            render::pose(
                &mut mode,
                Vec3::new(0.0, 0.0, 0.0),
                Quat::IDENTITY,
                40.0,
                default_pose_colors,
            );

            // Draw the estimated pose
            render::pose(
                &mut mode,
                estimated_position,
                estimated_orientation,
                1.0,
                default_pose_colors,
            );

            // Draw the estimated pose
            render::pose(
                &mut mode,
                ground_truth_position,
                ground_truth_orientation,
                1.0,
                ground_truth_pose_colors,
            );
        }

        if let Some(texture) = &texture {
            draw_handle.draw_texture_ex(
                texture,
                Vector2::new((width - texture.width / 2) as f32, 0.0),
                0.0,
                0.5,
                Color::WHITE,
            );
        }
    }
}
