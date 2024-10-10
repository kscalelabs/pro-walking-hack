# walking

## Notes

[Where to get onnxruntime binaries](https://github.com/microsoft/onnxruntime/releases)

## Commands

To get the path to the onnxruntime library, run `python -c "import onnxruntime; print(onnxruntime..)"`.

Run model on MacOS:

```bash
ORT_STRATEGY=system ORT_DYLIB_PATH=$(pwd)/lib/macos/libonnxruntime.dylib cargo run -p model -- position_control.onnx
```
