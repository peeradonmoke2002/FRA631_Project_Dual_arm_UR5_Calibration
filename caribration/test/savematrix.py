import numpy as np
import json
import pathlib

best_matrix = np.array([[ 0.0091902,   0.98683012, -0.08573875,  0.69742562],
                        [ 0.01638979, -0.11234373, -1.02432666, -1.19305022],
                        [-0.99304297,  0.00872967, -0.01697439, -0.00232298],
                        [ 0.0,         0.0,         0.0,         1.0       ]])

data = {
    "name": "camera_to_world_transform",
    "matrix": best_matrix.tolist()
}
config_path = pathlib.Path(__file__).parent.parent / "config" / "best_matrix.json"
print(f"Saving best matrix to {config_path}")
with open(config_path, 'w') as f:
    json.dump(data, f, indent=4)


# To load the matrix from the JSON file, you can use the following code:
# ----
# with open(config_path, 'r') as f:
#     loaded_data = json.load(f)
#     name = loaded_data["name"]
#     matrix = np.array(loaded_data["matrix"])

# print("Name:", name)
# print("Matrix:\n", matrix)