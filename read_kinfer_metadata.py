from kinfer.inference import ONNXModel

fast = "gpr_walking.kinfer"
slow = "gpr_0.5s_pawel.kinfer"

model = ONNXModel(model_path=slow)

metadata = model.get_metadata()
print(metadata)
