mod robstride;

use futures_util::StreamExt;
use robstride::RobStrideUtils;
use robstride::StopMode;
use socketcan::{tokio::CanSocket, CanFrame, EmbeddedFrame, Result};
use std::time::Duration;
use tokio;
use tokio::io::AsyncReadExt;
use tokio::time;

#[tokio::main]
async fn main() -> Result<()> {
    robstride::debug_main();

    Ok(())
}
