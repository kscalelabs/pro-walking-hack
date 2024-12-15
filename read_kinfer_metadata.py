from kinfer.inference import ONNXModel

fast = "policies/gpr_walking.kinfer"
slow = "policies/gpr_0.5s_pawel.kinfer"
weak = "policies/gpr_walking_weak.kinfer"
pr126 = "policies/gpr_walking_pr126.kinfer"
med_gait = "policies/74c91d8_0.4s.kinfer"
standing = "policies/gpr_standing.kinfer"

model = ONNXModel(model_path=standing)

metadata = model.get_metadata()
print(metadata)
