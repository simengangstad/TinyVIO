use crate::serialisable::{Deserialise, Serialise};
use image::{DynamicImage, ImageBuffer, Luma};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DataPointType {
    Imu,
    Image,
    GroundTruth,
    None,
}

#[derive(Debug, Clone)]
pub struct ImuDataPoint {
    pub timestamp: f64,
    pub linear_acceleration: glam::Vec3,
    pub angular_velocity: glam::Vec3,
}

#[derive(Debug, Clone)]
pub struct ImageDataPoint {
    pub timestamp: f64,
    pub image: DynamicImage,
}

#[derive(Debug, Clone)]
pub struct GroundTruthDataPoint {
    pub timestamp: f64,
    pub position: glam::Vec3,
    pub orientation: glam::Quat,
    pub velocity: glam::Vec3,
    pub angular_velocity_bias: glam::Vec3,
    pub linear_acceleration_bias: glam::Vec3,
}

#[derive(Debug, Clone)]
pub struct PoseDataPoint {
    pub position: glam::Vec3,
    pub orientation: glam::Quat,
}

impl Serialise for ImuDataPoint {
    fn serialise(&self) -> Vec<u8> {
        let mut payload: Vec<u8> = Vec::new();

        payload.append(&mut self.timestamp.to_bits().to_le_bytes().to_vec());

        payload.append(
            &mut (self.linear_acceleration.x as f32)
                .to_bits()
                .to_le_bytes()
                .to_vec(),
        );
        payload.append(
            &mut (self.linear_acceleration.y as f32)
                .to_bits()
                .to_le_bytes()
                .to_vec(),
        );
        payload.append(
            &mut (self.linear_acceleration.z as f32)
                .to_bits()
                .to_le_bytes()
                .to_vec(),
        );

        payload.append(
            &mut (self.angular_velocity.x as f32)
                .to_bits()
                .to_le_bytes()
                .to_vec(),
        );
        payload.append(
            &mut (self.angular_velocity.y as f32)
                .to_bits()
                .to_le_bytes()
                .to_vec(),
        );
        payload.append(
            &mut (self.angular_velocity.z as f32)
                .to_bits()
                .to_le_bytes()
                .to_vec(),
        );

        return payload;
    }
}

impl Serialise for ImageDataPoint {
    fn serialise(&self) -> Vec<u8> {
        let mut payload: Vec<u8> = Vec::new();

        payload.append(&mut self.timestamp.to_bits().to_le_bytes().to_vec());

        let image = self.image.to_luma8();

        payload.append(&mut (image.width() as u32).to_le_bytes().to_vec());
        payload.append(&mut (image.height() as u32).to_le_bytes().to_vec());
        payload.append(&mut image.to_vec());

        return payload;
    }
}

impl Deserialise<ImageDataPoint> for Vec<u8> {
    fn deserialise(&self) -> Option<ImageDataPoint> {
        let timestamp = unsafe {
            std::mem::transmute::<[u8; 8], f64>([
                self[0], self[1], self[2], self[3], self[4], self[5], self[6], self[7],
            ])
        };

        let width =
            unsafe { std::mem::transmute::<[u8; 4], u32>([self[8], self[9], self[10], self[11]]) }
                .to_le();

        let height = unsafe {
            std::mem::transmute::<[u8; 4], u32>([self[12], self[13], self[14], self[15]])
        }
        .to_le();

        if (width * height) as usize != self.len() - 16 {
            return None;
        }

        let image_data: Option<ImageBuffer<Luma<u8>, _>> =
            ImageBuffer::from_vec(width, height, self[16..].to_vec());

        if let Some(image_data) = image_data {
            return Some(ImageDataPoint {
                timestamp,
                image: DynamicImage::ImageLuma8(image_data),
            });
        }

        return None;
    }
}

impl Serialise for GroundTruthDataPoint {
    fn serialise(&self) -> Vec<u8> {
        let mut payload: Vec<u8> = Vec::new();

        payload.append(&mut self.timestamp.to_bits().to_le_bytes().to_vec());

        payload.append(&mut (self.position.x as f32).to_bits().to_le_bytes().to_vec());
        payload.append(&mut (self.position.y as f32).to_bits().to_le_bytes().to_vec());
        payload.append(&mut (self.position.z as f32).to_bits().to_le_bytes().to_vec());

        payload.append(&mut (self.orientation.w as f32).to_bits().to_le_bytes().to_vec());
        payload.append(&mut (self.orientation.x as f32).to_bits().to_le_bytes().to_vec());
        payload.append(&mut (self.orientation.y as f32).to_bits().to_le_bytes().to_vec());
        payload.append(&mut (self.orientation.z as f32).to_bits().to_le_bytes().to_vec());

        payload.append(&mut (self.velocity.x as f32).to_bits().to_le_bytes().to_vec());
        payload.append(&mut (self.velocity.y as f32).to_bits().to_le_bytes().to_vec());
        payload.append(&mut (self.velocity.z as f32).to_bits().to_le_bytes().to_vec());

        payload.append(
            &mut (self.linear_acceleration_bias.x as f32)
                .to_bits()
                .to_le_bytes()
                .to_vec(),
        );
        payload.append(
            &mut (self.linear_acceleration_bias.y as f32)
                .to_bits()
                .to_le_bytes()
                .to_vec(),
        );
        payload.append(
            &mut (self.linear_acceleration_bias.z as f32)
                .to_bits()
                .to_le_bytes()
                .to_vec(),
        );

        payload.append(
            &mut (self.angular_velocity_bias.x as f32)
                .to_bits()
                .to_le_bytes()
                .to_vec(),
        );
        payload.append(
            &mut (self.angular_velocity_bias.y as f32)
                .to_bits()
                .to_le_bytes()
                .to_vec(),
        );
        payload.append(
            &mut (self.angular_velocity_bias.z as f32)
                .to_bits()
                .to_le_bytes()
                .to_vec(),
        );

        return payload;
    }
}

impl Deserialise<PoseDataPoint> for Vec<u8> {
    fn deserialise(&self) -> Option<PoseDataPoint> {
        let mut index: usize = 0;

        let x = unsafe {
            std::mem::transmute::<[u8; 4], f32>([
                self[index],
                self[index + 1],
                self[index + 2],
                self[index + 3],
            ])
        };
        index += 4;

        let y = unsafe {
            std::mem::transmute::<[u8; 4], f32>([
                self[index],
                self[index + 1],
                self[index + 2],
                self[index + 3],
            ])
        };
        index += 4;

        let z = unsafe {
            std::mem::transmute::<[u8; 4], f32>([
                self[index],
                self[index + 1],
                self[index + 2],
                self[index + 3],
            ])
        };
        index += 4;

        let qx = unsafe {
            std::mem::transmute::<[u8; 4], f32>([
                self[index],
                self[index + 1],
                self[index + 2],
                self[index + 3],
            ])
        };
        index += 4;

        let qy = unsafe {
            std::mem::transmute::<[u8; 4], f32>([
                self[index],
                self[index + 1],
                self[index + 2],
                self[index + 3],
            ])
        };
        index += 4;

        let qz = unsafe {
            std::mem::transmute::<[u8; 4], f32>([
                self[index],
                self[index + 1],
                self[index + 2],
                self[index + 3],
            ])
        };
        index += 4;

        let qw = unsafe {
            std::mem::transmute::<[u8; 4], f32>([
                self[index],
                self[index + 1],
                self[index + 2],
                self[index + 3],
            ])
        };

        Some(PoseDataPoint {
            position: glam::Vec3::new(x, y, z),
            orientation: glam::Quat::from_xyzw(qx, qy, qz, qw),
        })
    }
}
