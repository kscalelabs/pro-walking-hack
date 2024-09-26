use ndarray::{Array, ArrayD};
use ort::{GraphOptimizationLevel, Result, Session, Value, ValueRef};
use std::{thread, time::Duration};

// New struct to capture inputs
pub struct ModelInput {
    x_vel: f32,
    y_vel: f32,
    rot: f32,
    t: f32,
    dof_pos: [f32; 10],
    dof_vel: [f32; 10],
    prev_actions: [f32; 10],
    imu_ang_vel: [f32; 3],
    imu_euler_xyz: [f32; 3],
    buffer: [f32; 574],
}

pub fn generate_model_input() -> ModelInput {
    ModelInput {
        x_vel: 0.0,
        y_vel: 0.0,
        rot: 0.0,
        t: 0.0,
        dof_pos: [0.0; 10],
        dof_vel: [0.0; 10],
        prev_actions: [0.0; 10],
        imu_ang_vel: [0.0; 3],
        imu_euler_xyz: [0.0; 3],
        buffer: [0.0; 574],
    }
}

pub fn load_onnx_model(model_path: &str) -> Result<Session> {
    Session::builder()?
        .with_optimization_level(GraphOptimizationLevel::Level3)?
        .with_intra_threads(4)?
        .commit_from_file(model_path)
}

pub fn model_input_to_array(input: &ModelInput) -> Result<ArrayD<f32>> {
    let mut input_vec = Vec::with_capacity(614);
    input_vec.push(input.x_vel);
    input_vec.push(input.y_vel);
    input_vec.push(input.rot);
    input_vec.push(input.t);
    input_vec.extend_from_slice(&input.dof_pos);
    input_vec.extend_from_slice(&input.dof_vel);
    input_vec.extend_from_slice(&input.prev_actions);
    input_vec.extend_from_slice(&input.imu_ang_vel);
    input_vec.extend_from_slice(&input.imu_euler_xyz);
    input_vec.extend_from_slice(&input.buffer);

    Ok(Array::from_shape_vec((1, 614), input_vec)?.into_dyn())
}
