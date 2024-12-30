from kinfer.inference import ONNXModel

standing = "policies/gpr_standing.kinfer"



model = ONNXModel(model_path=standing)

metadata = model.get_metadata()
print(metadata)
