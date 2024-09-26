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
    let mut sock_rx = CanSocket::open("can0")?;
    let sock_tx = CanSocket::open("can0")?;

    let mut rs_util = RobStrideUtils::new(0x7f);

    sock_tx.write_frame(rs_util.request_dev_id())?.await?;
    sock_tx.write_frame(rs_util.request_enable())?.await?;
    sock_tx
        .write_frame(rs_util.request_motion(0.0, 1f32, 0f32, 0.2f32, 0.1f32))?
        .await?;
    // sock_tx.write_frame(rs_util.request_stop(StopMode::Normal))?.await?;

    tokio::spawn(async move {
        sock_tx
            .write_frame(rs_util.request_param(0x302d))
            .expect("Error in reading param");
        time::sleep(Duration::from_millis(1)).await;
    });

    while let Some(res) = sock_rx.next().await {
        match res {
            Ok(CanFrame::Data(frame)) => unsafe {
                if frame.data().len() == 8 {
                    let fdata = &frame.data()[4..8].align_to::<f32>();
                    println!("{:?}", fdata);
                }
                println!("{:?}", &frame);
            },
            Ok(CanFrame::Remote(frame)) => println!("{:?}", frame),
            Ok(CanFrame::Error(frame)) => println!("{:?}", frame),
            Err(err) => eprintln!("{}", err),
        }
    }

    Ok(())
}
