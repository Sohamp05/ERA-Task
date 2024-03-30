import cv2
import numpy as np

# Step 1: Extract frames from video
def extract_frames(video_path):
    cap = cv2.VideoCapture(video_path)
    frames = []
    while cap.isOpened():
        ret, frame = cap.read()
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if not ret:
            break
        frames.append(frame)
    cap.release()
    return frames

# Step 2: Divide frames into three equal horizontal parts
def divide_frame(frame):
    height, width, _ = frame.shape
    third_width = width // 3
    left_view = frame[:, :third_width]
    right_view = frame[:, third_width:2*third_width]
    top_view = frame[:, 2*third_width:]
    return left_view, right_view, top_view

# Step 3: Create depth maps
def create_depth_map(left_view, right_view):
    # Perform stereo matching (e.g., StereoSGBM_create)
    stereo = cv2.StereoSGBM_create(numDisparities=16, blockSize=15)
    disparity = stereo.compute(cv2.cvtColor(left_view, cv2.COLOR_BGR2GRAY), cv2.cvtColor(right_view, cv2.COLOR_BGR2GRAY))
    # Convert disparity map to depth map
    depth_map = 1.0 / (disparity + 0.01)
    return depth_map

# Step 4: Triangulation
def triangulation(depth_map_left, depth_map_right, baseline, focal_length):
    rows, cols = depth_map_left.shape
    points = []
    for i in range(rows):
        for j in range(cols):
            if depth_map_left[i, j] > 0 and depth_map_right[i, j] > 0:
                disparity = abs(j - cols // 2)
                if disparity != 0:  # Avoid division by zero
                    Z = baseline * focal_length / disparity
                    X = (j - cols // 2) * Z / focal_length
                    Y = i * Z / focal_length
                    points.append((X, Y, Z))
    return points

# Main function
def main():
    video_path = 'C:/Users/sgpan/Downloads/final/final/final_1.avi'
    frames = extract_frames(video_path)
    focal_length = 500  # Focal length of the camera (in pixels)
    baseline = 100  # Baseline between the left and right cameras (in arbitrary units)
    for frame in frames:
        left_view, right_view, top_view = divide_frame(frame)
        depth_map_left = create_depth_map(left_view, right_view)
        depth_map_right = create_depth_map(right_view, left_view)
        points_3d = triangulation(depth_map_left, depth_map_right, baseline, focal_length)
        print("3D Points:", points_3d)

if __name__ == "__main__":
    main()



# import cv2
# import numpy as np


# def extract_frames(video_path):
#     cap = cv2.VideoCapture(video_path)
#     frames = []
#     while cap.isOpened():
#         ret, frame = cap.read()
#         if not ret:
#             break
#         # Divide frame into three equal horizontal parts
#         height, width, _ = frame.shape
#         left_view = frame[:, :width // 3]
#         right_view = frame[:, width // 3: 2 * width // 3]
#         top_view = frame[:, 2 * width // 3:]
#         frames.append((left_view, right_view, top_view))
#     cap.release()
#     return frames

# # Load left and right camera images
# frame =  extract_frames(r"C:\Users\sgpan\Downloads\final\final\final_1.avi")
# left_image1 = frame[0]
# left_image = cv2.cvtColor(left_image1, cv2.COLOR_BGR2RGB)
# right_image1 = frame[1]
# right_image = cv2.cvtColor(right_image1, cv2.COLOR_BGR2RGB)

# # Stereo disparity computation
# stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
# disparity = stereo.compute(left_image, right_image)

# # Convert disparity to depth map
# depth_map = cv2.convertScaleAbs(disparity)

# # Define camera parameters (to be adjusted based on calibration)
# baseline = 0.1  # Baseline between left and right cameras (in meters)
# focal_length = 0.8  # Focal length of cameras (in meters)

# # Triangulation function
# def triangulate_point(disparity_left, disparity_right, pixel_left, pixel_right):
#     depth = (baseline * focal_length) / (disparity_left - disparity_right)
#     x = pixel_left[0] * depth / focal_length
#     y = pixel_left[1] * depth / focal_length
#     return x, y, depth

# # Example reference point (to be adjusted based on the setup)
# reference_point = (0, 0, 0)

# # Perform triangulation for each pixel
# for y in range(left_image.shape[0]):
#     for x in range(left_image.shape[1]):
#         if disparity[y, x] > 0:
#             pixel_left = (x, y)
#             pixel_right = (x - disparity[y, x], y)
#             x_3d, y_3d, z_3d = triangulate_point(disparity[y, x], 0, pixel_left, pixel_right)
#             distance = np.sqrt((x_3d - reference_point[0])**2 + (y_3d - reference_point[1])**2 + (z_3d - reference_point[2])**2)
#             print(f"Pixel ({x}, {y}) - Depth: {z_3d:.2f} meters, Distance from reference point: {distance:.2f} meters")

# # Display depth map and triangulated points
# cv2.imshow('Depth Map', depth_map)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
