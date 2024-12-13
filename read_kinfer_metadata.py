from kinfer.inference import ONNXModel

fast = "gpr_walking.kinfer"
slow = "gpr_0.5s_pawel.kinfer"
weak = "gpr_walking_weak.kinfer"

model = ONNXModel(model_path=weak)

metadata = model.get_metadata()
print(metadata)
