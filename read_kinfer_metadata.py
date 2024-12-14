from kinfer.inference import ONNXModel

fast = "gpr_walking.kinfer"
slow = "gpr_0.5s_pawel.kinfer"
weak = "gpr_walking_weak.kinfer"
pr126 = "gpr_walking_pr126.kinfer"
med_gait = "74c91d8_0.4s.kinfer"
standing = "gpr_standing.kinfer"

model = ONNXModel(model_path=standing)

metadata = model.get_metadata()
print(metadata)
