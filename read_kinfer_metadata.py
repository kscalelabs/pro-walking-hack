from kinfer.inference import ONNXModel

fast = "gpr_walking.kinfer"
slow = "gpr_0.5s_pawel.kinfer"
weak = "gpr_walking_weak.kinfer"
pr126 = "gpr_walking_pr126.kinfer"

model = ONNXModel(model_path=pr126)

metadata = model.get_metadata()
print(metadata)
