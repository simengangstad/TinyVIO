use crate::data_interface::Packet;
use crate::data_point::{DataPointType, GroundTruthDataPoint, ImageDataPoint, ImuDataPoint};
use crate::header::{Header, HeaderType, HEADER_METADATA_SIZE};
use crate::serialisable::Serialise;
use euroc::{
    CameraRecords, EuRoC, GroundTruthData, GroundTruthIterator, GroundTruthRecord, ImageIterator,
    ImageRecord, ImuData, ImuIterator, ImuRecord, Timestamp,
};
use std::{fmt, iter::Peekable, path::Path};

pub struct EurocLoader {
    imu_data: ImuData,
    image_data: CameraRecords,
    ground_truth_data: GroundTruthData,

    imu_iterator: Peekable<ImuIterator>,
    image_iterator: Peekable<ImageIterator>,
    ground_truth_iterator: Peekable<GroundTruthIterator>,
}

impl EurocLoader {
    pub fn new(dataset_path: &Path) -> Result<Self, anyhow::Error> {
        let euroc = EuRoC::new(dataset_path)?;

        let imu_data = euroc.imu()?;
        let image_data = euroc.left_camera()?;
        let ground_truth_data = euroc.ground_truth()?;

        let imu_iterator = imu_data.records()?.peekable();
        let image_iterator = image_data.records()?.peekable();
        let ground_truth_iterator = ground_truth_data.records()?.peekable();

        Ok(EurocLoader {
            imu_data,
            image_data,
            ground_truth_data,

            imu_iterator,
            image_iterator,
            ground_truth_iterator,
        })
    }

    pub fn reset(&mut self, start_time: f64) -> Result<(), anyhow::Error> {
        self.imu_iterator = self.imu_data.records()?.peekable();
        self.image_iterator = self.image_data.records()?.peekable();
        self.ground_truth_iterator = self.ground_truth_data.records()?.peekable();

        loop {
            if let Some(value) = self.next() {
                if value.timestamp >= start_time {
                    break;
                }
            } else {
                panic!(
                    "Reset issued with start time {} which is beyond the dataset",
                    start_time
                );
            }
        }

        Ok(())
    }
}

impl Iterator for EurocLoader {
    type Item = EurocDataPoint;

    fn next(&mut self) -> Option<Self::Item> {
        let mut timestamp = Timestamp::from(u64::MAX);
        let mut data_point_type = DataPointType::None;

        // First find the next data point in time
        if let Some(Ok(imu_record)) = self.imu_iterator.peek() {
            if imu_record.timestamp < timestamp {
                timestamp = imu_record.timestamp;
                data_point_type = DataPointType::Imu;
            }
        }

        if let Some(Ok(image_record)) = self.image_iterator.peek() {
            if image_record.timestamp < timestamp {
                timestamp = image_record.timestamp;
                data_point_type = DataPointType::Image;
            }
        }

        if let Some(Ok(ground_truth_record)) = self.ground_truth_iterator.peek() {
            if ground_truth_record.timestamp < timestamp {
                timestamp = ground_truth_record.timestamp;
                data_point_type = DataPointType::GroundTruth;
            }
        }

        let mut imu = None;
        let mut image = None;
        let mut ground_truth = None;

        // Then we advance the iterator for that data point
        match data_point_type {
            DataPointType::Imu => imu = Some(self.imu_iterator.next().unwrap().unwrap().into()),
            DataPointType::Image => {
                image = Some(self.image_iterator.next().unwrap().unwrap().into())
            }
            DataPointType::GroundTruth => {
                ground_truth = Some(self.ground_truth_iterator.next().unwrap().unwrap().into())
            }
            DataPointType::None => {}
        }

        Some(EurocDataPoint {
            timestamp: (timestamp.nsecs() as f64) / 1.0e9,
            data_point_type,
            imu,
            image,
            ground_truth,
        })
    }
}

pub struct EurocDataPoint {
    pub timestamp: f64,
    pub data_point_type: DataPointType,
    pub imu: Option<ImuDataPoint>,
    pub image: Option<ImageDataPoint>,
    pub ground_truth: Option<GroundTruthDataPoint>,
}

impl fmt::Display for EurocDataPoint {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f, "\r\nType: {:?}", self.data_point_type)?;
        writeln!(f, "Timestamp: {}", self.timestamp)?;

        match self.data_point_type {
            DataPointType::Imu => {
                let imu = self.imu.clone().unwrap();

                writeln!(f, "Acceleration: {:?}", imu.linear_acceleration)?;
                writeln!(f, "Angular velocity: {:?}", imu.angular_velocity)
            }
            DataPointType::Image => {
                let image = self.image.clone().unwrap().image.to_luma8();
                writeln!(f, "Resolution: {}x{}", image.width(), image.height())
            }
            DataPointType::GroundTruth => {
                let ground_truth = self.ground_truth.clone().unwrap();

                writeln!(f, "Position: {:?}", ground_truth.position)?;
                writeln!(f, "Quaternion: {:?}", ground_truth.orientation)?;
                writeln!(f, "Velocity: {:?}", ground_truth.velocity)?;
                writeln!(
                    f,
                    "Acceleration: {:?}",
                    ground_truth.linear_acceleration_bias
                )?;
                writeln!(
                    f,
                    "Angular velocity: {:?}",
                    ground_truth.angular_velocity_bias
                )
            }
            DataPointType::None => Ok(()),
        }
    }
}

impl From<ImuRecord> for ImuDataPoint {
    fn from(imu_record: ImuRecord) -> ImuDataPoint {
        ImuDataPoint {
            timestamp: (imu_record.timestamp.nsecs() as f64) / 1.0e9,
            linear_acceleration: glam::Vec3::new(
                imu_record.accel.x as f32,
                imu_record.accel.y as f32,
                imu_record.accel.z as f32,
            ),
            angular_velocity: glam::Vec3::new(
                imu_record.gyro.x as f32,
                imu_record.gyro.y as f32,
                imu_record.gyro.z as f32,
            ),
        }
    }
}

impl From<ImageRecord> for ImageDataPoint {
    fn from(image_record: ImageRecord) -> ImageDataPoint {
        ImageDataPoint {
            timestamp: (image_record.timestamp.nsecs() as f64) / 1.0e9,
            image: image_record.image,
        }
    }
}

impl From<GroundTruthRecord> for GroundTruthDataPoint {
    fn from(ground_truth_record: GroundTruthRecord) -> GroundTruthDataPoint {
        GroundTruthDataPoint {
            timestamp: (ground_truth_record.timestamp.nsecs() as f64) / 1.0e9,
            position: glam::Vec3::new(
                ground_truth_record.position.x as f32,
                ground_truth_record.position.y as f32,
                ground_truth_record.position.z as f32,
            ),
            orientation: glam::Quat::from_xyzw(
                ground_truth_record.quaternion.i as f32,
                ground_truth_record.quaternion.j as f32,
                ground_truth_record.quaternion.k as f32,
                ground_truth_record.quaternion.w as f32,
            ),
            velocity: glam::Vec3::new(
                ground_truth_record.velocity.x as f32,
                ground_truth_record.velocity.y as f32,
                ground_truth_record.velocity.z as f32,
            ),
            linear_acceleration_bias: glam::Vec3::new(
                ground_truth_record.accel.x as f32,
                ground_truth_record.accel.y as f32,
                ground_truth_record.accel.z as f32,
            ),
            angular_velocity_bias: glam::Vec3::new(
                ground_truth_record.gyro.x as f32,
                ground_truth_record.gyro.y as f32,
                ground_truth_record.gyro.z as f32,
            ),
        }
    }
}

impl From<&EurocDataPoint> for Packet {
    fn from(data_point: &EurocDataPoint) -> Packet {
        // First we need to serialise in order to get the total amount of bytes in the payload
        let data = match data_point.data_point_type {
            DataPointType::Imu => data_point.imu.as_ref().unwrap().serialise(),
            DataPointType::Image => data_point.image.as_ref().unwrap().serialise(),
            DataPointType::GroundTruth => data_point.ground_truth.as_ref().unwrap().serialise(),
            DataPointType::None => vec![],
        };

        let header = Header {
            header_type: HeaderType::Transmission(data_point.data_point_type.into()),
            payload_size: data.len() as u32,
            meta_data: [0 as u8; HEADER_METADATA_SIZE],
        };

        Packet { header, data }
    }
}
