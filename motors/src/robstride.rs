use serialport::SerialPort;
use std::io::Write;
use std::time::Duration;

const MAX_PORTS: usize = 10;

const CAN_ID_MASTER: u8 = 0x00; // Control host address - SPIE
const CAN_ID_MOTOR_DEFAULT: u8 = 0x7F; // Default motor address - Unconfigured ID
const CAN_ID_BROADCAST: u8 = 0xFE; // Broadcast address - Default receive address
const CAN_ID_DEBUG_UI: u8 = 0xFD; // Debug address - Upper computer address

// Ubuntu
const BAUDRATE: u32 = 921600;
const TTY_PORT: &str = "/dev/ttyCH341USB0";

// Mac
// Note that this baudrate will let you send something to the device on Mac, but
// it won't work because the device baudrate is supposed to be 921600.
// const BAUDRATE: u32 = 115200;
// const TTY_PORT: &str = "/dev/tty.usbserial-110";

#[repr(u8)]
enum CanComMode {
    AnnounceDevId = 0,
    MotorCtrl,
    MotorFeedback,
    MotorIn,
    MotorReset,
    MotorCali,
    MotorZero,
    MotorId,
    ParaWrite,
    ParaRead,
    ParaUpdate,
    OtaStart,
    OtaInfo,
    OtaIng,
    OtaEnd,
    CaliIng,
    CaliRst,
    SdoRead,
    SdoWrite,
    ParaStrInfo,
    MotorBrake,
    FaultWarn,
    ModeTotal,
}

#[repr(u8)]
enum MotorMode {
    Reset = 0,
    Cali,
    Motor,
    Brake,
}

struct MotorStatus {
    under_volt_fault: bool,
    over_cur_fault: bool,
    over_temp_fault: bool,
    encoder_fault: bool,
    hall_fault: bool,
    no_cali_fault: bool,
    mode: MotorMode,
}

struct ExId(u32);

impl ExId {
    fn new(id: u8, data: u16, mode: u8, res: u8) -> ExId {
        let mut value: u32 = 0;
        value |= (res as u32 & 0x07) << 29;
        value |= (mode as u32 & 0x1F) << 24;
        value |= (data as u32 & 0xFFFF) << 8;
        value |= id as u32 & 0xFF;
        ExId(value)
    }

    fn to_addr(&self) -> u32 {
        let exid_u32 = self.0;
        (exid_u32 << 3) | 0x00000004
    }
}

#[derive(Default)]
struct CanPack {
    ex_id: ExId,
    len: u8,
    data: [u8; 8],
}

fn txd_pack(serial_port: &mut dyn SerialPort, pack: &CanPack) {
    println!("Sending data");

    let mut len = pack.len;
    let mut data = pack.data;

    if len < 1 {
        len = 1;
        data[0] = 0;
    } else if len > 8 {
        len = 8;
    }

    let addr = pack.ex_id.to_addr();

    let mut packet = Vec::with_capacity(9 + len as usize);
    packet.push(b'A');
    packet.push(b'T');

    packet.push(((addr & 0xFF000000) >> 24) as u8);
    packet.push(((addr & 0x00FF0000) >> 16) as u8);
    packet.push(((addr & 0x0000FF00) >> 8) as u8);
    packet.push((addr & 0x000000FF) as u8);

    packet.push(len);

    for i in 0..len as usize {
        packet.push(data[i]);
    }

    packet.push(b'\r');
    packet.push(b'\n');

    // Print the full packet
    print!("Full packet (hex): ");
    for byte in &packet {
        print!("{:02X} ", byte);
    }
    println!();

    // Write the data
    match serial_port.write(&packet) {
        Ok(bytes_written) => {
            if bytes_written != packet.len() {
                eprintln!(
                    "Warning: Not all bytes written. Expected {}, wrote {}",
                    packet.len(),
                    bytes_written
                );
            } else {
                println!("Data sent successfully");
            }
        }
        Err(e) => {
            eprintln!("Error writing to serial port: {}", e);
        }
    }

    // Flush the output buffer
    if let Err(e) = serial_port.flush() {
        eprintln!("Error flushing serial port: {}", e);
    }
}

fn init_serial_port(device: &str) -> Result<Box<dyn SerialPort>, serialport::Error> {
    println!("Initializing serial port: {}", device);
    let port = serialport::new(device, BAUDRATE)
        .timeout(Duration::from_secs(1))
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        .open();

    match port {
        Ok(port) => {
            println!("Serial port initialized successfully");
            Ok(port)
        }
        Err(e) => {
            eprintln!("Error opening serial port: {}", e);
            Err(e)
        }
    }
}

fn send_reset(serial_port: &mut dyn SerialPort, id: u8) {
    let pack = CanPack {
        ex_id: ExId::new(id, CAN_ID_DEBUG_UI as u16, CanComMode::MotorReset as u8, 0),
        len: 8,
        data: [0; 8],
    };
    txd_pack(serial_port, &pack);
}

fn send_start(serial_port: &mut dyn SerialPort, id: u8) {
    let pack = CanPack {
        ex_id: ExId::new(id, CAN_ID_DEBUG_UI as u16, CanComMode::MotorIn as u8, 0),
        len: 8,
        data: [0; 8],
    };
    txd_pack(serial_port, &pack);
}

pub fn debug_main() {
    println!("Starting program");

    // Initialize serial port
    let serial_port = init_serial_port(TTY_PORT);
    if serial_port.is_err() {
        eprintln!("Failed to initialize serial port");
        return;
    }
    let mut serial_port = serial_port.unwrap();

    // Add a delay after initializing the port
    std::thread::sleep(Duration::from_millis(500));

    // Initialize the device
    let device_id: u8 = 1; // Use the default motor address
    send_start(&mut serial_port, device_id);
    // send_reset(&mut serial_port, device_id);

    println!("Program finished");
}
