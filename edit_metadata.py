import onnx
import json

# Load the model
model = onnx.load("gpr_standing.kinfer")

# Get current metadata
metadata = None
for prop in model.metadata_props:
    if prop.key == "kinfer_metadata":
        metadata = prop.value
        break

if metadata is None:
    raise ValueError("kinfer_metadata not found in model metadata")

metadata_dict = json.loads(metadata)

# Update the default_standing values
metadata_dict["default_standing"] = [0.0] * len(metadata_dict["default_standing"])

# Convert back to json string
new_metadata = json.dumps(metadata_dict)

# Create new metadata property
new_metadata_prop = onnx.StringStringEntryProto()
new_metadata_prop.key = "kinfer_metadata"
new_metadata_prop.value = new_metadata

# Replace the metadata
del model.metadata_props[:]  # This clears the metadata
model.metadata_props.append(new_metadata_prop)

# Save the updated model
onnx.save(model, "gpr_standing_new.kinfer")
