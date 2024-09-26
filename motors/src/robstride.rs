use socketcan::{CanFrame, EmbeddedFrame, ExtendedId};
use std::f32::consts::PI;
use std::ptr::addr_of_mut;

pub enum StopMode {
    Normal = 0,
    ClearErr = 1,
}

pub struct RobStrideUtils {
    id: u8,
}

impl RobStrideUtils {
    pub fn new(id: u8) -> Self {
        Self { id }
    }

    fn set_extended_id(&mut self, com_type: u8, data_field: u16, destination: u8) -> ExtendedId {
        ExtendedId::new((com_type as u32) << 24 | (data_field as u32) << 8 | (destination as u32))
            .unwrap()
    }

    pub fn request_dev_id(&mut self) -> CanFrame {
        CanFrame::new(self.set_extended_id(0, 0x0000, self.id), &[0]).unwrap()
    }

    pub fn request_motion(
        &mut self,
        torque: f32,
        angle: f32,
        velocity: f32,
        kp: f32,
        kd: f32,
    ) -> CanFrame {
        let torqueu16 = ((torque + 17f32 / 34f32) * 65535f32) as u16;
        let angleu16 = ((angle + 4f32 * PI / 8f32 * PI) * 65535f32) as u16;
        let velocityu16 = ((velocity + 44f32 / 88f32) * 65535f32) as u16;
        let kpu16 = ((kp / 500f32) * 65535f32) as u16;
        let kdu16 = ((kd / 5f32) * 65535f32) as u16;

        CanFrame::new(
            self.set_extended_id(1, torqueu16, self.id),
            &[
                (angleu16 >> 8) as u8,
                angleu16 as u8,
                (velocityu16 >> 8) as u8,
                velocityu16 as u8,
                (kpu16 >> 8) as u8,
                kpu16 as u8,
                (kdu16 >> 8) as u8,
                kdu16 as u8,
            ],
        )
        .unwrap()
    }

    pub fn request_enable(&mut self) -> CanFrame {
        CanFrame::new(self.set_extended_id(3, 0x0000, self.id), &[0]).unwrap()
    }

    pub fn request_stop(&mut self, mode: StopMode) -> CanFrame {
        CanFrame::new(self.set_extended_id(4, 0x0000, self.id), &[mode as u8]).unwrap()
    }

    pub fn set_zero_point(&mut self) -> CanFrame {
        CanFrame::new(self.set_extended_id(6, 0x0000, self.id), &[1]).unwrap()
    }

    pub fn set_dev_id(&mut self, request_id: u8) -> CanFrame {
        CanFrame::new(
            self.set_extended_id(7, (request_id as u16) << 8, self.id),
            &[1],
        )
        .unwrap()
    }

    pub fn request_param(&mut self, index: u16) -> CanFrame {
        CanFrame::new(
            self.set_extended_id(17, 0x0000, self.id),
            &[index as u8, (index >> 8) as u8],
        )
        .unwrap()
    }

    pub fn write_param(&mut self, index: u16, param: u32) -> CanFrame {
        CanFrame::new(
            self.set_extended_id(18, 0x0000, self.id),
            &[
                index as u8,
                (index >> 8) as u8,
                0,
                0,
                param as u8,
                (param >> 8) as u8,
                (param >> 16) as u8,
                (param >> 24) as u8,
            ],
        )
        .unwrap()
    }
}
