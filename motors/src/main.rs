use serialport::prelude::*;
use std::error::Error;
use std::io::{Read, Write};
use std::time::Duration;

// Constants
const CAN_ID_MASTER: u8 = 0x00; // Control host address - SPIE
const CAN_ID_MOTOR_DEFAULT: u8 = 0x7F; // Default motor address - Unconfigured ID
const CAN_ID_BROADCAST: u8 = 0xFE; // Broadcast address - Default receive address
const CAN_ID_DEBUG_UI: u8 = 0xFD; // Debug address - Upper computer address

const BAUDRATE: u32 = 921600;

#[cfg(target_os = "linux")]
const TTY_PORT: &str = "/dev/ttyUSB0";

#[cfg(target_os = "macos")]
const TTY_PORT: &str = "/dev/tty.wchusbserial110";

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
compile_error!("Unsupported platform");

// Motor type constants
const P_MIN: f32 = -12.5;
const P_MAX: f32 = 12.5;
const V_MIN: f32 = -20.0;
const V_MAX: f32 = 20.0;
const KP_MIN: f32 = 0.0;
const KP_MAX: f32 = 5000.0;
const KD_MIN: f32 = 0.0;
const KD_MAX: f32 = 100.0;
const T_MIN: f32 = -60.0;
const T_MAX: f32 = 60.0;

// Enums
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
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
#[derive(Debug, Clone, Copy)]
enum MotorMode {
    Reset = 0,
    Cali,
    Motor,
    Brake,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
enum RunMode {
    MitMode = 0,
    PositionMode,
    SpeedMode,
    CurrentMode,
    ToZeroMode,
    CspPositionMode,
}

// Structs
struct MotorStatus {
    under_volt_fault: bool,
    over_cur_fault: bool,
    over_temp_fault: bool,
    encoder_fault: bool,
    hall_fault: bool,
    no_cali_fault: bool,
    mode: MotorMode,
}

struct ExId {
    id: u8,
    data: u16,
    mode: u8, // 5 bits
    res: u8,  // 3 bits
}

impl ExId {
    fn to_u32(&self) -> u32 {
        let mut val = (self.id as u32) << 24;
        val |= (self.data as u32) << 8;
        val |= ((self.mode as u32) & 0x1F) << 3;
        val |= (self.res as u32) & 0x07;
        val
    }

    fn from_u32(val: u32) -> Self {
        let id = (val >> 24) as u8;
        let data = ((val >> 8) & 0xFFFF) as u16;
        let mode = ((val >> 3) & 0x1F) as u8;
        let res = (val & 0x07) as u8;
        ExId {
            id,
            data,
            mode,
            res,
        }
    }
}

struct CanPack {
    ex_id: ExId,
    len: u8,
    data: [u8; 8],
}

struct MotorFeedback {
    can_id: u32,
    position: f32,
    velocity: f32,
    torque: f32,
    mode: MotorMode,
    faults: u16,
    is_set: bool,
}

// Main MotorController struct
struct MotorController {
    port: Box<dyn SerialPort>,
}

impl MotorController {
    fn new() -> Result<Self, Box<dyn Error>> {
        let port = serialport::new(TTY_PORT, BAUDRATE)
            .timeout(Duration::from_millis(1000))
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .open()?;

        Ok(MotorController { port })
    }

    fn txd_pack(&mut self, pack: &CanPack) -> Result<(), Box<dyn Error>> {
        let mut pack_len = pack.len;
        if pack_len < 1 {
            pack_len = 1;
        } else if pack_len > 8 {
            pack_len = 8;
        }

        let total_len = 9 + pack_len as usize;
        let mut data_to_send = Vec::with_capacity(total_len);

        data_to_send.push(b'A');
        data_to_send.push(b'T');

        // Construct addr
        let mut addr = pack.ex_id.to_u32();
        addr = (addr << 3) | 0x00000004;

        data_to_send.push(((addr & 0xFF000000) >> 24) as u8);
        data_to_send.push(((addr & 0x00FF0000) >> 16) as u8);
        data_to_send.push(((addr & 0x0000FF00) >> 8) as u8);
        data_to_send.push((addr & 0x000000FF) as u8);

        data_to_send.push(pack_len);

        for i in 0..pack_len as usize {
            data_to_send.push(pack.data[i]);
        }

        data_to_send.push(b'\r');
        data_to_send.push(b'\n');

        // Print the full string of bits being sent
        print!("tx ");
        for byte in &data_to_send {
            print!("{:02X} ", byte);
        }
        println!();

        // Write the data
        self.port.write_all(&data_to_send)?;

        // Flush the serial port
        self.port.flush()?;

        Ok(())
    }

    fn uint_to_float(x_int: u16, x_min: f32, x_max: f32, bits: i32) -> f32 {
        let span = x_max - x_min;
        let offset = x_min;
        ((x_int as f32) * span) / ((1 << bits) - 1) as f32 + offset
    }

    fn float_to_uint(x: f32, x_min: f32, x_max: f32, bits: i32) -> u16 {
        let span = x_max - x_min;
        let offset = x_min;
        ((x - offset) * ((1 << bits) - 1) as f32 / span) as u16
    }

    fn read_bytes(&mut self) -> Result<MotorFeedback, Box<dyn Error>> {
        let mut feedback = MotorFeedback {
            can_id: 0,
            position: 0.0,
            velocity: 0.0,
            torque: 0.0,
            mode: MotorMode::Reset,
            faults: 0,
            is_set: false,
        };

        let mut buffer = [0u8; 17];
        let bytes_read = self.port.read(&mut buffer)?;

        if bytes_read == 0 {
            println!("No bytes read");
            return Ok(feedback);
        }

        print!("rx ");
        for i in 0..bytes_read {
            print!("{:02X} ", buffer[i]);
        }
        println!();

        if bytes_read == 17 && buffer[0] == b'A' && buffer[1] == b'T' {
            let mut addr: u32 = (buffer[2] as u32) << 24
                | (buffer[3] as u32) << 16
                | (buffer[4] as u32) << 8
                | buffer[5] as u32;
            addr = addr >> 3;
            let ex_id = ExId::from_u32(addr);

            let len = buffer[6];
            let mut rx_frame = CanPack {
                ex_id,
                len,
                data: [0; 8],
            };
            for i in 0..rx_frame.len as usize {
                rx_frame.data[i] = buffer[7 + i];
            }

            // Parse the data
            feedback.can_id = rx_frame.ex_id.data as u32 & 0x00FF;
            feedback.faults = ((rx_frame.ex_id.data & 0x3F00) >> 8) as u16;
            feedback.mode = match (rx_frame.ex_id.data & 0xC000) >> 14 {
                0 => MotorMode::Reset,
                1 => MotorMode::Cali,
                2 => MotorMode::Motor,
                3 => MotorMode::Brake,
                _ => MotorMode::Reset,
            };

            let pos_int_get = ((rx_frame.data[0] as u16) << 8) | rx_frame.data[1] as u16;
            let vel_int_get = ((rx_frame.data[2] as u16) << 8) | rx_frame.data[3] as u16;
            let torque_int_get = ((rx_frame.data[4] as u16) << 8) | rx_frame.data[5] as u16;

            feedback.position = MotorController::uint_to_float(pos_int_get, P_MIN, P_MAX, 16);
            feedback.velocity = MotorController::uint_to_float(vel_int_get, V_MIN, V_MAX, 16);
            feedback.torque = MotorController::uint_to_float(torque_int_get, T_MIN, T_MAX, 16);

            feedback.is_set = true;

            // Print parsed data
            println!("Parsed data:");
            println!("  Motor ID: {}", feedback.can_id);
            println!("  Position: {}", feedback.position);
            println!("  Velocity: {}", feedback.velocity);
            println!("  Torque: {}", feedback.torque);
            println!("  Mode: {:?}", feedback.mode);
            println!("  Faults: {}", feedback.faults);
        }

        Ok(feedback)
    }

    pub fn send_set_mode(
        &mut self,
        runmode: RunMode,
        id: u8,
    ) -> Result<MotorFeedback, Box<dyn Error>> {
        let mut pack = CanPack {
            ex_id: ExId {
                id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::SdoWrite as u8,
                res: 0,
            },
            len: 8,
            data: [0; 8],
        };

        let index: u16 = 0x7005;
        pack.data[0..2].copy_from_slice(&index.to_le_bytes());
        pack.data[4] = runmode as u8;

        self.txd_pack(&pack)?;
        self.read_bytes()
    }

    pub fn send_reset(&mut self, id: u8) -> Result<MotorFeedback, Box<dyn Error>> {
        let pack = CanPack {
            ex_id: ExId {
                id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::MotorReset as u8,
                res: 0,
            },
            len: 8,
            data: [0; 8],
        };

        self.txd_pack(&pack)?;
        self.read_bytes()
    }

    pub fn send_start(&mut self, id: u8) -> Result<MotorFeedback, Box<dyn Error>> {
        let pack = CanPack {
            ex_id: ExId {
                id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::MotorIn as u8,
                res: 0,
            },
            len: 8,
            data: [0; 8],
        };

        self.txd_pack(&pack)?;
        self.read_bytes()
    }

    pub fn send_set_speed_limit(
        &mut self,
        id: u8,
        speed: f32,
    ) -> Result<MotorFeedback, Box<dyn Error>> {
        let mut pack = CanPack {
            ex_id: ExId {
                id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::SdoWrite as u8,
                res: 0,
            },
            len: 8,
            data: [0; 8],
        };

        let index: u16 = 0x7017;
        pack.data[0..2].copy_from_slice(&index.to_le_bytes());
        pack.data[4..8].copy_from_slice(&speed.to_le_bytes());

        self.txd_pack(&pack)?;
        self.read_bytes()
    }

    pub fn send_set_location(
        &mut self,
        id: u8,
        location: f32,
    ) -> Result<MotorFeedback, Box<dyn Error>> {
        let mut pack = CanPack {
            ex_id: ExId {
                id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::SdoWrite as u8,
                res: 0,
            },
            len: 8,
            data: [0; 8],
        };

        let index: u16 = 0x7016;
        pack.data[0..2].copy_from_slice(&index.to_le_bytes());
        pack.data[4..8].copy_from_slice(&location.to_le_bytes());

        self.txd_pack(&pack)?;
        self.read_bytes()
    }

    pub fn send_motor_control(
        &mut self,
        id: u8,
        pos_set: f32,
        vel_set: f32,
        kp_set: f32,
        kd_set: f32,
        torque_set: f32,
    ) -> Result<MotorFeedback, Box<dyn Error>> {
        let pos_int_set = MotorController::float_to_uint(pos_set, P_MIN, P_MAX, 16);
        let vel_int_set = MotorController::float_to_uint(vel_set, V_MIN, V_MAX, 16);
        let kp_int_set = MotorController::float_to_uint(kp_set, KP_MIN, KP_MAX, 16);
        let kd_int_set = MotorController::float_to_uint(kd_set, KD_MIN, KD_MAX, 16);
        let torque_int_set = MotorController::float_to_uint(torque_set, T_MIN, T_MAX, 16);

        let mut pack = CanPack {
            ex_id: ExId {
                id,
                data: torque_int_set,
                mode: CanComMode::MotorCtrl as u8,
                res: 0,
            },
            len: 8,
            data: [0; 8],
        };

        pack.data[0..2].copy_from_slice(&pos_int_set.to_be_bytes());
        pack.data[2..4].copy_from_slice(&vel_int_set.to_be_bytes());
        pack.data[4..6].copy_from_slice(&kp_int_set.to_be_bytes());
        pack.data[6..8].copy_from_slice(&kd_int_set.to_be_bytes());

        self.txd_pack(&pack)?;
        self.read_bytes()
    }

    pub fn send_position_control(
        &mut self,
        id: u8,
        pos_set: f32,
        kp_set: f32,
    ) -> Result<MotorFeedback, Box<dyn Error>> {
        self.send_motor_control(id, pos_set, 0.0, kp_set, 0.0, 0.0)
    }

    pub fn send_torque_control(
        &mut self,
        id: u8,
        torque_set: f32,
    ) -> Result<MotorFeedback, Box<dyn Error>> {
        self.send_motor_control(id, 0.0, 0.0, 0.0, 0.0, torque_set)
    }
}

// Example main function to test the code
fn main() -> Result<(), Box<dyn Error>> {
    println!("Starting program");

    // Initialize serial port
    let mut controller = MotorController::new()?;

    // Set mode to POSITION_MODE
    let feedback = controller.send_set_mode(RunMode::PositionMode, 1)?;
    if feedback.is_set {
        println!("Set mode feedback received");
    }
    std::thread::sleep(Duration::from_millis(50));

    // Reset the motor
    let feedback = controller.send_reset(1)?;
    if feedback.is_set {
        println!("Reset feedback received");
    }
    std::thread::sleep(Duration::from_millis(50));

    // Start the motor
    let feedback = controller.send_start(1)?;
    if feedback.is_set {
        println!("Start feedback received");
    }
    std::thread::sleep(Duration::from_millis(50));

    // Set speed limit
    let feedback = controller.send_set_speed_limit(1, 10.0)?;
    if feedback.is_set {
        println!("Set speed limit feedback received");
    }
    std::thread::sleep(Duration::from_millis(50));

    // Set location
    let feedback = controller.send_set_location(1, 0.0)?;
    if feedback.is_set {
        println!("Set location feedback received");
    }
    std::thread::sleep(Duration::from_secs(10));

    // Reset the motor after the loop
    let feedback = controller.send_reset(1)?;
    if feedback.is_set {
        println!("Reset feedback received");
    }

    println!("Program finished");
    Ok(())
}
