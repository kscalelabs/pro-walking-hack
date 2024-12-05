from kinfer.inference import ONNXModel

model = ONNXModel(model_path="gpr_walking.kinfer")

metadata = model.get_metadata()
print(metadata)
