import numpy as np
import json

best_matrix = np.array([[ 0.00465481, -0.99626089,  0.01972409, -0.08235386],
                        [ 0.1348973,   0.009654,    1.02464258, -0.06076352],
                        [-0.98427042,  0.00608958,  0.08377368, -0.10207071],
                        [ 0.0,         0.0,         0.0,         1.0       ]])

data = {
    "name": "camera_to_world_transform",
    "matrix": best_matrix.tolist()
}

with open(r'FRA631_Project_Dual_arm_UR5_Calibration\caribration\config\best_matrix.json', 'w') as f:
    json.dump(data, f, indent=4)


# To load the matrix from the JSON file, you can use the following code:
# ----
# with open('matrix.json', 'r') as f:
#     loaded_data = json.load(f)
#     name = loaded_data["name"]
#     matrix = np.array(loaded_data["matrix"])

# print("Name:", name)
# print("Matrix:\n", matrix)