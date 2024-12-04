from kinfer.inference import ONNXModel

model = ONNXModel(model_path="light_walking.onnx")

metadata = model.get_metadata()
print(metadata)
