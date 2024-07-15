import cv2
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import time
import tkinter as tk
from tkinter import ttk

def generate_point_cloud(depth_map, fx=525.0, fy=525.0, cx=319.5, cy=239.5 ):
    height, width = depth_map.shape
    u, v = np.meshgrid(np.arange(width), np.arange(height))  # creates two 2D arrays filled row-vise and column-vise

    z = depth_map / 1000.0  # Convert depth to meters
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    points = np.stack((x, y, z), axis=-1).reshape(-1, 3)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    return point_cloud
def color_point_cloud_by_distance(point_cloud, plane_model):
    points = np.asarray(point_cloud.points)
    [a, b, c, d] = plane_model
    distances = np.abs(np.dot(points, [a, b, c]) + d) / np.linalg.norm([a, b, c])

    max_distance = np.max(distances)  # Normalize distances for prettier colors
    colors = plt.cm.viridis(distances / max_distance)[:, :3]  # Using viridis colormap

    point_cloud.colors = o3d.utility.Vector3dVector(colors)
    return point_cloud

def custom_ransac(point_cloud, distance_threshold=0.05, num_iterations=100):
    points = np.asarray(point_cloud.points)
    best_plane = None
    max_inliers = -1
    best_inliers = None

    for _ in range(num_iterations):
        sample_indices = np.random.choice(points.shape[0], 3, replace=False)
        sample_points = points[sample_indices]

        p1, p2, p3 = sample_points
        normal = np.cross(p2 - p1, p3 - p1)
        if np.linalg.norm(normal) == 0:
            continue  # Skip if normal is zero to avoid dividing by zero
        normal = normal / np.linalg.norm(normal)
        d = -np.dot(normal, p1)
        plane_model = [normal[0], normal[1], normal[2], d]

        distances = np.abs(np.dot(points, normal) + d) / np.linalg.norm(normal)
        inliers = np.where(distances < distance_threshold)[0]

        if len(inliers) > max_inliers:
            max_inliers = len(inliers)
            best_plane = plane_model
            best_inliers = inliers
    return best_plane, best_inliers

def create_plane_mesh(plane_model, size=1.0):
    [a, b, c, d] = plane_model

    plane_vertices = [
        [-size, -size, (-d - a * -size - b * -size) / c],
        [size, -size, (-d - a * size - b * -size) / c],
        [size, size, (-d - a * size - b * size) / c],
        [-size, size, (-d - a * -size - b * size) / c]
    ]

    plane_triangles = [
        [0, 1, 2],
        [0, 2, 3]
    ]

    plane_mesh = o3d.geometry.TriangleMesh()
    plane_mesh.vertices = o3d.utility.Vector3dVector(plane_vertices)
    plane_mesh.triangles = o3d.utility.Vector3iVector(plane_triangles)
    plane_mesh.compute_vertex_normals()

    return plane_mesh

def fit_plane_to_point_cloud(point_cloud):
    plane_model, inliers = custom_ransac(point_cloud)

    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    inlier_cloud = point_cloud.select_by_index(inliers)
    outlier_cloud = point_cloud.select_by_index(inliers, invert=True)

    return inlier_cloud, outlier_cloud, plane_model
def visualize_point_cloud_with_plane(point_cloud, plane_model, camera_position):
    plane_mesh = create_plane_mesh(plane_model)
    plane_mesh.paint_uniform_color([1.0, 0, 0])  # Red

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    vis.add_geometry(point_cloud)
    vis.add_geometry(plane_mesh)

    camera_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    camera_sphere.translate(camera_position)
    camera_sphere.paint_uniform_color([0.5, 0, 0.5])
    vis.add_geometry(camera_sphere)

    opt = vis.get_render_option()
    opt.mesh_show_back_face = True

    ctr = vis.get_view_control()
    ctr.set_lookat([0, 0, 0])
    ctr.set_up([0, -1, 0])
    ctr.set_front([0, 0, -1])
    ctr.set_zoom(0.5)

    vis.run()
    vis.destroy_window()
def calculate_plane_statistics(point_cloud, plane_model):
    points = np.asarray(point_cloud.points)
    [a, b, c, d] = plane_model
    distances = np.abs(np.dot(points, [a, b, c]) + d) / np.linalg.norm([a, b, c])

    mean_distance = np.mean(distances)
    std_distance = np.std(distances)
    max_distance = np.max(distances)
    min_distance = np.min(distances)

    print(f"Mean distance to plane: {mean_distance:.4f}")
    print(f"Standard deviation of distance to plane: {std_distance:.4f}")
    print(f"Maximum distance to plane: {max_distance:.4f}")
    print(f"Minimum distance to plane: {min_distance:.4f}")

    return mean_distance, std_distance, max_distance, min_distance

def create_control_panel(num_planes, point_cloud, planes, remaining_cloud):
    global root
    def on_ok():
        selected_planes = [var.get() for var in check_vars]
        visualize_selected_planes(point_cloud, selected_planes, planes, remaining_cloud)

    root = tk.Tk()
    root.title("Control Panel")

    check_vars = []
    for i in range(num_planes):
        var = tk.BooleanVar(value=True)
        check_vars.append(var)
        chk = ttk.Checkbutton(root, text=f"Plane {i + 1}", variable=var)
        chk.pack(padx=10, pady=5)

    ok_button = ttk.Button(root, text="OK", command=on_ok)
    ok_button.pack(padx=10, pady=20)

    root.mainloop()
def detect_multiple_planes(point_cloud, distance_threshold=0.01, num_iterations=100, num_planes=3):
    planes = []
    remaining_cloud = point_cloud
    for p in range(num_planes):
        plane_model, inliers = custom_ransac(remaining_cloud, distance_threshold, num_iterations)
        if inliers is None:
            print("No more planes to detect, detected only:", p)
            break
        inlier_cloud = remaining_cloud.select_by_index(inliers)
        outlier_cloud = remaining_cloud.select_by_index(inliers, invert=True)
        planes.append((plane_model, inlier_cloud))
        remaining_cloud = outlier_cloud
    return planes, remaining_cloud
def visualize_selected_planes(point_cloud, selected_planes, planes, remaining_cloud, repaint=False):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(point_cloud)

    colors = plt.get_cmap("tab10")  # Define different colors for each plane

    for i in range(len(planes)):
        plane_mesh, inlier_cloud = planes[i]
        if selected_planes[i]:
            plane_mesh.paint_uniform_color(colors(i)[:3])
            if repaint: inlier_cloud.paint_uniform_color(colors(i)[:3])  # color points in corresponding plane color
            vis.add_geometry(inlier_cloud)
            vis.add_geometry(plane_mesh)

    # Add the remaining points
    if repaint: remaining_cloud.paint_uniform_color([0.5, 0.5, 0.5])  # color remaining points grey
    vis.add_geometry(remaining_cloud)

    opt = vis.get_render_option()
    opt.mesh_show_back_face = True

    ctr = vis.get_view_control()
    ctr.set_lookat([0, 0, 0])
    ctr.set_up([0, -1, 0])
    ctr.set_front([0, 0, -1])
    ctr.set_zoom(0.5)

    vis.run()
    vis.destroy_window()

def visualize_multiple_planes(point_cloud, num_planes=5, distance_threshold=0.01, num_iterations=100):
    planes, remaining_cloud = detect_multiple_planes(point_cloud, distance_threshold, num_iterations, num_planes)

    plane_geometries = []
    for plane_model, inlier_cloud in planes:
        plane_mesh = create_plane_mesh(plane_model)
        plane_geometries.append((plane_mesh, inlier_cloud))

    create_control_panel(num_planes, point_cloud, plane_geometries, remaining_cloud)

if __name__ == "__main__":
    # LOAD A DEPTH MAP
    # depth_map = np.load("depth_maps/tof_flat_wall.npy")
    # depth_map = np.load('depth_maps/tof_capture_light.npy')
    # depth_map = np.load('depth_maps/stereo_scene.npy')

    # point_cloud = generate_point_cloud(depth_map)

    # OR OPEN A POINT CLOUD

    pointcloud_path = "pointclouds/tof_walls_2.ply"
    # pointcloud_path = "pointclouds/tof_boxes.ply"
    point_cloud = o3d.io.read_point_cloud(pointcloud_path)

    # FIT A PLANE THROUGH POINT CLOUD
    inlier_cloud, outlier_cloud, plane_model = fit_plane_to_point_cloud(point_cloud)
    calculate_plane_statistics(point_cloud, plane_model)

    # FIT ONE PLANE TO POINT CLOUD
    # point_cloud = color_point_cloud_by_distance(point_cloud, plane_model)
    # visualize_point_cloud_with_plane(point_cloud, plane_model, [0, 0, 0])

    # FIT MULTIPLE PLANES TO POINT CLOUD
    visualize_multiple_planes(point_cloud, num_planes=3)