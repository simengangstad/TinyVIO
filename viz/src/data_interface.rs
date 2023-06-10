use crate::data_point::{DataPointType, GroundTruthDataPoint, ImageDataPoint, PoseDataPoint};
use crate::euroc_loader::EurocLoader;
use crate::header::*;
use crate::serialisable::{Deserialise, Serialise};
use glam::Vec2;
use log::{debug, error, warn};
use std::{
    cmp::min,
    io::prelude::*,
    io::ErrorKind,
    net::{Shutdown, SocketAddr, TcpListener, TcpStream},
    path::Path,
    sync::{mpsc, Arc, Mutex},
    thread,
};

pub struct Feature {
    pub point: Vec2,
}

pub struct Packet {
    pub header: Header,
    pub data: Vec<u8>,
}

pub struct DataInterface {
    socket_address: SocketAddr,
    image_data_point_transmitter: mpsc::Sender<ImageDataPoint>,
    pose_data_point_transmitter: mpsc::Sender<PoseDataPoint>,
    ground_truth_data_point_transmitter: mpsc::Sender<GroundTruthDataPoint>,
    message_receiver: Arc<Mutex<mpsc::Receiver<u8>>>,

    feature_data: Arc<Mutex<Vec<Feature>>>,

    dataset_folder_path: String,
}

impl DataInterface {
    pub fn new(
        socket_address: SocketAddr,
        image_data_point_transmitter: mpsc::Sender<ImageDataPoint>,
        pose_data_point_transmitter: mpsc::Sender<PoseDataPoint>,
        ground_truth_data_point_transmitter: mpsc::Sender<GroundTruthDataPoint>,
        message_receiver: Arc<Mutex<mpsc::Receiver<u8>>>,
        feature_data: Arc<Mutex<Vec<Feature>>>,
        dataset_folder_path: &str,
    ) -> DataInterface {
        DataInterface {
            socket_address,
            image_data_point_transmitter,
            pose_data_point_transmitter,
            ground_truth_data_point_transmitter,
            message_receiver,
            feature_data,
            dataset_folder_path: dataset_folder_path.to_string(),
        }
    }

    pub fn start(&self) {
        let listener = match TcpListener::bind(self.socket_address) {
            Ok(l) => l,
            Err(error) => panic!("Could not TCP server, please check your ethernet link setup and your static IP address: {}", error),
        };

        for stream in listener.incoming() {
            match stream {
                Ok(stream) => {
                    stream
                        .set_nonblocking(true)
                        .expect("Failed to set stream to non-blocking");

                    debug!("New connection: {}", stream.peer_addr().unwrap());

                    let image_data_point_transmitter = self.image_data_point_transmitter.clone();
                    let pose_data_point_transmitter = self.pose_data_point_transmitter.clone();
                    let ground_truth_data_point_transmitter =
                        self.ground_truth_data_point_transmitter.clone();
                    let feature_data_clone = self.feature_data.clone();
                    let message_receiver_clone = Arc::clone(&self.message_receiver);
                    let dataset_folder_path_clone = self.dataset_folder_path.clone();

                    thread::spawn(move || {
                        handle_client(
                            stream,
                            image_data_point_transmitter,
                            pose_data_point_transmitter,
                            ground_truth_data_point_transmitter,
                            message_receiver_clone,
                            feature_data_clone,
                            dataset_folder_path_clone,
                        );
                    });
                }
                Err(error) => {
                    error!("Error handling new connection: {}", error);
                }
            }
        }
    }
}

fn read_from_stream(stream: &mut TcpStream, buffer: &mut Vec<u8>, amount: usize) -> Option<usize> {
    let mut packet = [0u8; 2048];
    let mut bytes_read = 0;

    while buffer.len() < amount {
        match stream.read(&mut packet) {
            Ok(size) => {
                let remaining_bytes = amount - buffer.len();
                buffer.append(&mut packet[0..min(size, remaining_bytes)].to_vec());
                bytes_read += size;
            }
            Err(error) if error.kind() == ErrorKind::ConnectionAborted => {
                warn!("Remote aborted connection");
                return None;
            }
            Err(error) if error.kind() == ErrorKind::ConnectionReset => {
                warn!("Remote reset connection");
                return None;
            }
            Err(error) if error.kind() == ErrorKind::WouldBlock => {}
            Err(error) => {
                error!("Error occurred in TCP connection. Error: {}", error.kind());
                stream.shutdown(Shutdown::Both).unwrap();
                return None;
            }
        }
    }

    Some(bytes_read)
}

fn handle_client(
    mut stream: TcpStream,
    image_data_point_transmitter: mpsc::Sender<ImageDataPoint>,
    pose_data_point_transmitter: mpsc::Sender<PoseDataPoint>,
    ground_truth_data_point_transmitter: mpsc::Sender<GroundTruthDataPoint>,
    _message_receiver: Arc<Mutex<mpsc::Receiver<u8>>>,
    _feature_data: Arc<Mutex<Vec<Feature>>>,
    dataset_folder_path: String,
) {
    let mut euroc_loader = match EurocLoader::new(Path::new(&dataset_folder_path)) {
        Ok(euroc_loader) => euroc_loader,
        Err(error) => panic!("Error creating EuRoC Loader: {}", error),
    };

    loop {
        let mut data: Vec<u8> = Vec::new();

        if let Some(size) = read_from_stream(&mut stream, &mut data, HEADER_SIZE) {
            if size < HEADER_SIZE {
                continue;
            }

            let header_buffer: [u8; HEADER_SIZE] = data[0..HEADER_SIZE].try_into().unwrap();
            let header: Header = header_buffer.into();

            match header.header_type {
                HeaderType::Transmission(transmission_type) => match transmission_type {
                    TransmissionType::Image => {
                        debug!("Got image header, payload size: {}", header.payload_size);

                        // Now we want to read the rest of the data
                        let mut image_data_point_buffer: Vec<u8> = Vec::new();
                        image_data_point_buffer.append(&mut data[HEADER_SIZE..size].to_vec());

                        let bytes_remaining =
                            header.payload_size as usize - image_data_point_buffer.len();

                        match read_from_stream(
                            &mut stream,
                            &mut image_data_point_buffer,
                            bytes_remaining,
                        ) {
                            None => {
                                warn!("Failed to read image payload");
                            }
                            Some(size) => {
                                if size != header.payload_size as usize {
                                    error!("Did not receive all bytes for image payload");
                                    continue;
                                }

                                if let Some(image_data_point) =
                                    image_data_point_buffer.deserialise()
                                {
                                    image_data_point_transmitter
                                        .send(image_data_point)
                                        .expect("Failed to send image data point through channel");
                                }
                            }
                        }
                    }
                    TransmissionType::Pose => {
                        debug!("Got pose header, payload size: {}", header.payload_size);

                        let mut pose_buffer: Vec<u8> = Vec::new();
                        pose_buffer.append(&mut data[HEADER_SIZE..size].to_vec());

                        let bytes_remaining = header.payload_size as usize - pose_buffer.len();

                        match read_from_stream(&mut stream, &mut pose_buffer, bytes_remaining) {
                            None => {
                                warn!("Failed to read pose payload");
                            }
                            Some(size) => {
                                if size != header.payload_size as usize {
                                    error!("Did not receive all bytes for pose payload");
                                    continue;
                                }

                                if let Some(pose) = pose_buffer.deserialise() {
                                    pose_data_point_transmitter
                                        .send(pose)
                                        .expect("Failed to send pose data point through channel");
                                }
                            }
                        }
                    }
                    _ => {
                        warn!(
                            "Got unknown transmission type {:?}, payload size: {}",
                            transmission_type, header.payload_size
                        );
                    }
                },
                HeaderType::Request(request) => {
                    match request {
                        RequestType::Initialisation(start_time) => {
                            match euroc_loader.reset(start_time) {
                                Ok(_) => {
                                    debug!("Reset EuRoC Loader to start time at: {}", start_time);
                                }
                                Err(error) => panic!("Failed to reset the EuRoC Loader: {}", error),
                            }
                        }
                        RequestType::NewDataPoint => {
                            debug!("Got new data point request");

                            match euroc_loader.next() {
                                Some(data_point) => {
                                    debug!(
                                        "Sending data point: {:?}, timestamp: {}",
                                        data_point.data_point_type, data_point.timestamp
                                    );

                                    let data_point_packet: Packet = (&data_point).into();
                                    data_point_packet.transmit(&mut stream);

                                    match data_point.data_point_type {
                                        DataPointType::GroundTruth => {
                                            ground_truth_data_point_transmitter.send(data_point.ground_truth.unwrap()).expect("Failed to send ground truth data point over channel");
                                        }
                                        DataPointType::Image => {
                                            image_data_point_transmitter
                                                .send(data_point.image.unwrap())
                                                .expect(
                                                    "Failed to send image data point over channel",
                                                );
                                        }
                                        _ => {}
                                    }

                                    if data_point.data_point_type == DataPointType::GroundTruth {}
                                }
                                None => debug!("No data points left in dataset"),
                            }
                        }
                        RequestType::NewBatch => {
                            debug!("Got new batch request");
                            // Collect all the data up to (and including) the next image

                            let mut batch_packet = Packet {
                                header: Header {
                                    header_type: HeaderType::Transmission(TransmissionType::Batch),
                                    payload_size: 0,
                                    meta_data: [0u8; 8],
                                },
                                data: vec![],
                            };

                            let mut imu_data_points = 0;
                            let mut image_data_points = 0;
                            let mut ground_truth_data_points = 0;

                            for data_point in euroc_loader.by_ref() {
                                let mut data_point_packet: Packet = (&data_point).into();

                                batch_packet
                                    .data
                                    .append(&mut data_point_packet.header.serialise());
                                batch_packet.data.append(&mut data_point_packet.data);

                                match data_point.data_point_type {
                                    DataPointType::Imu => {
                                        imu_data_points += 1;
                                    }
                                    DataPointType::Image => {
                                        image_data_points += 1;
                                        image_data_point_transmitter
                                            .send(data_point.image.unwrap())
                                            .expect("Failed to send image data point over channel");
                                        break;
                                    }
                                    DataPointType::GroundTruth => {
                                        ground_truth_data_points += 1;
                                        ground_truth_data_point_transmitter.send(data_point.ground_truth.unwrap()).expect("Failed to send ground truth data point over channel");
                                    }
                                    DataPointType::None => {
                                        error!("Got none as data point, finishing up this batch");
                                        break;
                                    }
                                }
                            }

                            debug!("Sending batch packet with {} IMU data points, {} image data points and {} ground truth data points", imu_data_points, image_data_points, ground_truth_data_points);

                            batch_packet.header.payload_size = batch_packet.data.len() as u32;
                            batch_packet.transmit(&mut stream);
                        }
                        RequestType::None => {
                            error!("Unknown request");
                        }
                    }
                }
                HeaderType::None => {
                    warn!("Got unknown header, payload size: {}", header.payload_size);
                }
            }
        } else {
            break;
        }

        /*
        if let Some(message) = process_messages(Arc::clone(&message_receiver)) {
            match message {
                MESSAGE_TOGGLE_FREERUNNING => {
                    freerunning = !freerunning;
                    info!("Free running: {}", freerunning);
                }
                MESSAGE_NEXT_IMAGE => {
                    info!("Sending next image...");
                    show_next_image = true;
                    freerunning = false;
                }
                _ => {}
            }
        }
        */
    }
}

impl Packet {
    pub fn transmit(&self, stream: &mut TcpStream) {
        // First send the header
        match stream.write(&self.header.serialise()) {
            Ok(_) => {}
            Err(error) if error.kind() == ErrorKind::WouldBlock => {}
            Err(error) => {
                error!(
                    "Failed to transmit data point header through stream: {}",
                    error
                );
            }
        }

        // Then deliver the actual payload
        let mut data_sent = 0;
        while data_sent < self.data.len() {
            match stream.write(&self.data[data_sent..min(data_sent + 1024, self.data.len())]) {
                Ok(size) => data_sent += size,
                Err(error) if error.kind() == ErrorKind::WouldBlock => {}
                Err(error) => {
                    error!(
                        "Failed to transmit data point payload through stream: {}",
                        error
                    );
                }
            }
        }
    }
}
