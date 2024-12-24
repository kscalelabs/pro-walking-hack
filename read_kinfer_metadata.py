from kinfer.inference import ONNXModel

fast = "policies/gpr_walking.kinfer"
slow = "policies/gpr_0.5s_pawel.kinfer"
weak = "policies/gpr_walking_weak.kinfer"
pr126 = "policies/gpr_walking_pr126.kinfer"
med_gait = "policies/74c91d8_0.4s.kinfer"
standing = "policies/gpr_standing.kinfer"

# Imu tests
noisy = "policies/noisy_imu_walking.kinfer"
filtered = "policies/filtered_walking.kinfer"
less_noise = "policies/less_noise.kinfer"

noisy_small = "policies/noisy_small.kinfer"

short_steps = "policies/short_steps.kinfer"

normal_short_plane = "policies/normal_short_plane.kinfer"
normal_short_trimesh = "policies/normal_short_trimesh.kinfer"


model = ONNXModel(model_path=normal_short_plane)

metadata = model.get_metadata()
print(metadata)
