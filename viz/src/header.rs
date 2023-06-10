use crate::data_point::DataPointType;
use crate::serialisable::Serialise;

pub const HEADER_SIZE: usize = 16;
pub const HEADER_TYPE_INDEX: usize = 0;
pub const HEADER_PAYLOAD_TYPE_INDEX: usize = 1;
pub const HEADER_PAYLOAD_SIZE_INDEX: usize = 2;
pub const HEADER_METADATA_INDEX: usize = 8;
pub const HEADER_METADATA_SIZE: usize = 8;

pub struct Header {
    pub header_type: HeaderType,
    pub payload_size: u32,
    pub meta_data: [u8; HEADER_METADATA_SIZE],
}

pub enum HeaderType {
    Transmission(TransmissionType),
    Request(RequestType),
    None,
}

pub enum RequestType {
    Initialisation(f64),
    NewDataPoint,
    NewBatch,
    None,
}

#[derive(Debug)]
pub enum TransmissionType {
    Imu,
    Image,
    GroundTruth,
    Features,
    Pose,
    Batch,
    None,
}

impl Serialise for Header {
    fn serialise(&self) -> Vec<u8> {
        let mut data: Vec<u8> = vec![0; HEADER_SIZE];

        data[0] = (&self.header_type).into();

        match &self.header_type {
            HeaderType::Transmission(transmission_type) => {
                data[1] = transmission_type.into();
            }
            HeaderType::Request(request_type) => {
                data[1] = request_type.into();
            }
            HeaderType::None => data[1] = HeaderType::None.into(),
        }

        let payload_size_buffer = (self.payload_size as u32).to_be_bytes().to_vec();

        for i in 0..payload_size_buffer.len() {
            data[i + HEADER_PAYLOAD_SIZE_INDEX] = payload_size_buffer[i];
        }

        return data;
    }
}

impl From<[u8; HEADER_SIZE]> for Header {
    fn from(data: [u8; HEADER_SIZE]) -> Header {
        let mut header: Header = Header {
            header_type: HeaderType::None,
            payload_size: 0,
            meta_data: [0 as u8; HEADER_METADATA_SIZE],
        };

        header.header_type = match data[HEADER_TYPE_INDEX] {
            0 => HeaderType::Transmission(data.into()),
            1 => HeaderType::Request(data.into()),
            _ => HeaderType::None,
        };

        header.payload_size = unsafe {
            std::mem::transmute::<[u8; 4], u32>([
                data[HEADER_PAYLOAD_SIZE_INDEX],
                data[HEADER_PAYLOAD_SIZE_INDEX + 1],
                data[HEADER_PAYLOAD_SIZE_INDEX + 2],
                data[HEADER_PAYLOAD_SIZE_INDEX + 3],
            ])
        }
        .to_le();

        for i in 0..HEADER_METADATA_SIZE {
            header.meta_data[i] = data[HEADER_METADATA_INDEX + i];
        }

        return header;
    }
}

impl From<HeaderType> for u8 {
    fn from(data: HeaderType) -> u8 {
        match data {
            HeaderType::Transmission(_) => 0,
            HeaderType::Request(_) => 1,
            HeaderType::None => 99,
        }
    }
}

impl From<&HeaderType> for u8 {
    fn from(data: &HeaderType) -> u8 {
        match data {
            HeaderType::Transmission(_) => 0,
            HeaderType::Request(_) => 1,
            HeaderType::None => 99,
        }
    }
}

impl From<TransmissionType> for u8 {
    fn from(data: TransmissionType) -> u8 {
        match data {
            TransmissionType::Imu => 0,
            TransmissionType::Image => 1,
            TransmissionType::GroundTruth => 2,
            TransmissionType::Features => 3,
            TransmissionType::Pose => 4,
            TransmissionType::Batch => 5,
            TransmissionType::None => 99,
        }
    }
}

impl From<&TransmissionType> for u8 {
    fn from(data: &TransmissionType) -> u8 {
        match data {
            TransmissionType::Imu => 0,
            TransmissionType::Image => 1,
            TransmissionType::GroundTruth => 2,
            TransmissionType::Features => 3,
            TransmissionType::Pose => 4,
            TransmissionType::Batch => 5,
            TransmissionType::None => 99,
        }
    }
}

impl From<[u8; HEADER_SIZE]> for TransmissionType {
    fn from(data: [u8; HEADER_SIZE]) -> TransmissionType {
        match data[HEADER_PAYLOAD_TYPE_INDEX] {
            0 => TransmissionType::Imu,
            1 => TransmissionType::Image,
            2 => TransmissionType::GroundTruth,
            3 => TransmissionType::Features,
            4 => TransmissionType::Pose,
            5 => TransmissionType::Batch,
            _ => TransmissionType::None,
        }
    }
}

impl From<DataPointType> for TransmissionType {
    fn from(data_point_type: DataPointType) -> TransmissionType {
        match data_point_type {
            DataPointType::Imu => TransmissionType::Imu,
            DataPointType::Image => TransmissionType::Image,
            DataPointType::GroundTruth => TransmissionType::GroundTruth,
            DataPointType::None => TransmissionType::None,
        }
    }
}

impl From<RequestType> for u8 {
    fn from(data: RequestType) -> u8 {
        match data {
            RequestType::Initialisation(_) => 0,
            RequestType::NewDataPoint => 1,
            RequestType::NewBatch => 2,
            RequestType::None => 99,
        }
    }
}

impl From<&RequestType> for u8 {
    fn from(data: &RequestType) -> u8 {
        match data {
            RequestType::Initialisation(_) => 0,
            RequestType::NewDataPoint => 1,
            RequestType::NewBatch => 2,
            RequestType::None => 99,
        }
    }
}

impl From<[u8; HEADER_SIZE]> for RequestType {
    fn from(data: [u8; HEADER_SIZE]) -> RequestType {
        match data[HEADER_PAYLOAD_TYPE_INDEX] {
            0 => {
                let start_time = unsafe {
                    std::mem::transmute::<[u8; 8], f64>([
                        data[HEADER_METADATA_INDEX],
                        data[HEADER_METADATA_INDEX + 1],
                        data[HEADER_METADATA_INDEX + 2],
                        data[HEADER_METADATA_INDEX + 3],
                        data[HEADER_METADATA_INDEX + 4],
                        data[HEADER_METADATA_INDEX + 5],
                        data[HEADER_METADATA_INDEX + 6],
                        data[HEADER_METADATA_INDEX + 7],
                    ])
                };

                RequestType::Initialisation(start_time)
            }
            1 => RequestType::NewDataPoint,
            2 => RequestType::NewBatch,
            _ => RequestType::None,
        }
    }
}
