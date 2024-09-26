use model;
use ort::{Environment, SessionBuilder, Value};
use std::sync::Arc;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize the ONNX Runtime environment
    let environment = Arc::new(Environment::builder().build()?);

    // Load the model
    let session =
        SessionBuilder::new(&environment)?.with_model_from_file("position_control.jit")?;

    // Create dummy input
    let input_tensor = vec![1.0f32; 10]; // Adjust size based on your model's input shape
    let input = Value::from_array(session.allocator(), &input_tensor)?;

    // Run the model
    let outputs = session.run(vec![input])?;

    // Print the output
    println!("Model output: {:?}", outputs);

    Ok(())
}
