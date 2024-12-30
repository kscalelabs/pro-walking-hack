# walking

## Notes

[Where to get onnxruntime binaries](https://github.com/microsoft/onnxruntime/releases)

## Commands

To get the path to the onnxruntime library, run `python -c "import onnxruntime; print(onnxruntime..)"`.

Run model on MacOS:

```bash
ORT_STRATEGY=system ORT_DYLIB_PATH=$(pwd)/lib/macos/libonnxruntime.dylib cargo run -p model -- position_control.onnx
```

## Troubleshooting

#### Motor not found

If motors aren't being detected and you see warnings like this:
```
2024-12-13T01:51:14.276121Z  WARN kos_kbot::actuator: Configured motor not found - ID: 14, Type: RobStride02
```

It might be that the CAN bus isn't properly initialized. To fix this, run these commands:

```bash
ip link set can0 down
ip link set can0 type can bitrate 1000000
ip link set can0 txqueuelen 1000
ip link set can0 up

ip link set can1 down
ip link set can1 type can bitrate 1000000
ip link set can1 txqueuelen 1000
ip link set can1 up
```
You can also put this in a script `configure_can.sh` and run it on startup.
