#!/usr/bin/env python3
"""
Open-VINS数据可视化工具
使用Pangolin可视化pose、速度、协方差、双目图像等关键数据
"""

import cv2
import numpy as np
import sys
import os
import time
from natsort import natsorted

# Add utils to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Pangolin and OpenGL
import pangolin
import OpenGL.GL as gl

# Import utilities
from utils.visualization_utils import (
    Color, color2bgr, set_gl_color, transform_points,
    quaternion_to_rotation_matrix, quaternion_to_euler_angles, pose_to_transform_matrix,
    draw_grid_y, draw_pose, draw_arrow,
    TopViewY, TopViewFV,
    load_odometry_data, load_poseimu_data, load_imu_data, load_points_msckf_data,
    create_text_image, calculate_frame_distances,
    draw_coordinate_frame, draw_camera_frustum
)


def load_camera_image(data_dir, cam_name, frame_time):
    """Load camera image for a given frame_time

    Args:
        data_dir: Path to the bag directory containing asset_data
        cam_name: 'cam0' or 'cam1'
        frame_time: frame_time string (e.g. '001520530314_000000850000')

    Returns:
        numpy image (BGR) or None
    """
    img_path = os.path.join(data_dir, 'asset_data', cam_name, f"{frame_time}.png")
    if os.path.exists(img_path):
        return cv2.imread(img_path)
    return None


def create_dual_camera_image(cam0_img, cam1_img, target_width=640, target_height=256):
    """Create a combined image with two cameras side by side with labels

    Args:
        cam0_img: BGR image for left camera (or None)
        cam1_img: BGR image for right camera (or None)
        target_width: width of the combined image
        target_height: height of each camera image

    Returns:
        BGR combined image
    """
    half_width = target_width // 2

    # Create background
    combined = np.ones((target_height, target_width, 3), dtype=np.uint8) * 40

    # Process cam0 (left)
    if cam0_img is not None:
        # Resize to fit
        h, w = cam0_img.shape[:2]
        scale = min(half_width / w, target_height / h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        cam0_resized = cv2.resize(cam0_img, (new_w, new_h))
        # Place in left half, centered
        y_offset = (target_height - new_h) // 2
        x_offset = (half_width - new_w) // 2
        combined[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = cam0_resized

    # Process cam1 (right)
    if cam1_img is not None:
        h, w = cam1_img.shape[:2]
        scale = min(half_width / w, target_height / h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        cam1_resized = cv2.resize(cam1_img, (new_w, new_h))
        y_offset = (target_height - new_h) // 2
        x_offset = half_width + (half_width - new_w) // 2
        combined[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = cam1_resized

    # Add labels
    font = cv2.FONT_HERSHEY_SIMPLEX
    # Left camera label
    cv2.putText(combined, "Left (cam0)", (10, 25), font, 0.7, (255, 255, 255), 2)
    # Right camera label
    cv2.putText(combined, "Right (cam1)", (half_width + 10, 25), font, 0.7, (255, 255, 255), 2)

    # Add separator line
    cv2.line(combined, (half_width, 0), (half_width, target_height), (200, 200, 200), 2)

    return combined


def main(data_dir):
    """
    Main visualization function for Open-VINS data

    Args:
        data_dir: Path to the bag directory containing asset_data
    """
    print("Loading odometry data...")
    odometry_data = load_odometry_data(data_dir)
    print(f"Loaded {len(odometry_data)} odometry frames")

    print("Loading poseimu data...")
    poseimu_data = load_poseimu_data(data_dir)
    print(f"Loaded {len(poseimu_data)} poseimu frames")

    print("Loading IMU data...")
    imu_data = load_imu_data(data_dir)
    print(f"Loaded {len(imu_data)} IMU frames")

    print("Loading MSCKF points data...")
    points_data = load_points_msckf_data(data_dir)
    print(f"Loaded {len(points_data)} MSCKF points frames")

    if len(odometry_data) == 0:
        print("No data found, exiting")
        return

    # Extract trajectory from odometry_data
    positions = []
    orientations = []
    covariances = []

    for data in odometry_data:
        pos = data['pose']['position']
        ori = data['pose']['orientation']
        cov = data['pose']['covariance']

        positions.append([pos['x'], pos['y'], pos['z']])
        orientations.append([ori['x'], ori['y'], ori['z'], ori['w']])

        # Convert 36-element covariance to 6x6 matrix
        cov_matrix = np.array(cov).reshape(6, 6)
        covariances.append(cov_matrix)

    positions = np.array(positions)
    orientations = np.array(orientations)

    # Build frame_time list for camera image lookup
    frame_times = [d['frame_time'] for d in odometry_data]

    # Calculate frame distances (cumulative chord length)
    print("Calculating frame distances...")
    frame_distances = calculate_frame_distances(positions)
    print(f"Total distance: {frame_distances[-1]:.3f} m")

    # Calculate center for view
    center = np.mean(positions, axis=0) if len(positions) > 0 else np.zeros(3)

    frame_size = len(odometry_data)
    current_frame_idx = 0

    # Screen parameters
    screen_w = 1920
    screen_h = 1080

    pangolin.CreateWindowAndBind('Open-VINS Visualization', screen_w, screen_h)
    gl.glEnable(gl.GL_DEPTH_TEST)
    gl.glEnable(gl.GL_BLEND)
    gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)

    # Camera setup (top-down view with Y-axis as up)
    scam = pangolin.OpenGlRenderState(
        pangolin.ProjectionMatrix(screen_w, screen_h, 2000, 2000, 960, 540, 0.1, 500),
        pangolin.ModelViewLookAt(center[0], center[1], center[2] + 15,
                                 center[0], center[1], center[2],
                                 0, 1, 0))
    handler = pangolin.Handler3D(scam)

    dcam = pangolin.CreateDisplay()
    dcam.SetBounds(0.0, 1.0, 0.0, 1.0, -screen_w / screen_h)
    dcam.SetHandler(handler)

    # Text image panel (left bottom)
    txt_b, txt_t, txt_l, txt_r = 0.0, 0.5, 0.0, 1. / 6.
    txt_imageh, txt_imagew = 540, 320
    dtxtimg = pangolin.Display('txt')
    dtxtimg.SetBounds(txt_b, txt_t, txt_l, txt_r, float(txt_imagew) / float(txt_imageh))
    dtxtimg.SetLock(pangolin.Lock.LockLeft, pangolin.Lock.LockTop)
    txt_text = pangolin.GlTexture(txt_imagew, txt_imageh, gl.GL_RGB, False, 0, gl.GL_RGB, gl.GL_UNSIGNED_BYTE)

    # Dual camera image panel (right top)
    # Two 512x512 images side by side with labels
    cam_img_w = 640  # combined width
    cam_img_h = 256  # combined height (each cam gets half width)
    dcamimg = pangolin.Display('camimg')
    dcamimg.SetBounds(0.75, 1.0, 1. / 6., 1., float(cam_img_w) / float(cam_img_h))
    dcamimg.SetLock(pangolin.Lock.LockRight, pangolin.Lock.LockTop)
    cam_text = pangolin.GlTexture(cam_img_w, cam_img_h, gl.GL_RGB, False, 0, gl.GL_RGB, gl.GL_UNSIGNED_BYTE)

    # Control panel (left top)
    panel = pangolin.CreatePanel('ui')
    panel.SetBounds(0.5, 1., 0.0, 1. / 6.)

    show_top_view = pangolin.VarBool('ui.TopView', value=True, toggle=False)
    show_grid = pangolin.VarBool('ui.grid', value=True, toggle=True)
    show_world_frame = pangolin.VarBool('ui.WorldFrame (W)', value=True, toggle=True)
    show_trajectory = pangolin.VarBool('ui.trajectory', value=True, toggle=True)
    show_current_pose = pangolin.VarBool('ui.IMU_frame (T_wi)', value=True, toggle=True)
    show_cam_frames = pangolin.VarBool('ui.cam_frames (T_wc)', value=True, toggle=True)
    show_velocity = pangolin.VarBool('ui.velocity', value=True, toggle=True)
    show_covariance = pangolin.VarBool('ui.covariance', value=False, toggle=True)
    show_pointcloud = pangolin.VarBool('ui.pointcloud', value=True, toggle=True)
    show_cam_images = pangolin.VarBool('ui.cam_images', value=True, toggle=True)
    auto_play = pangolin.VarBool('ui.Auto Play', value=False, toggle=False)
    play_step = pangolin.VarBool('ui.>>', value=False, toggle=False)
    play_back = pangolin.VarBool('ui.<<', value=False, toggle=False)
    curr_frame_idx = pangolin.VarInt('ui.frame_idx', value=0, min=0, max=frame_size - 1)

    print("Starting visualization loop...")

    # Auto play timing
    last_frame_time = 0
    frame_interval = 0.05  # 20Hz

    while not pangolin.ShouldQuit():
        current_time = time.time()

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gray_color = 125. / 255.
        gl.glClearColor(gray_color, gray_color, gray_color, 1.0)
        dcam.Activate(scam)

        frame_idx = curr_frame_idx.Get()

        # Auto play control
        if auto_play.Get():
            if current_time - last_frame_time >= frame_interval:
                frame_idx = (frame_idx + 1) % frame_size
                curr_frame_idx.SetVal(frame_idx)
                last_frame_time = current_time
        else:
            # Manual control
            if play_step.Get():
                play_step.SetVal(False)
                frame_idx = (frame_idx + 1) % frame_size

            if play_back.Get():
                play_back.SetVal(False)
                frame_idx = (frame_idx - 1) % frame_size

            curr_frame_idx.SetVal(frame_idx)

        # View control
        if show_top_view.Get():
            show_top_view.SetVal(False)
            scam = TopViewY(center)
            dcam.SetHandler(pangolin.Handler3D(scam))

        # Draw grid
        if show_grid.Get():
            draw_grid_y(1., center)

        # Draw world frame (0.5m axes)
        # World coordinate frame (W): Fixed reference frame at origin
        # X-axis: Red, Y-axis: Green, Z-axis: Blue
        if show_world_frame.Get():
            draw_coordinate_frame(np.eye(4), length=0.5, line_width=3)

        # Draw trajectory
        if show_trajectory.Get() and len(positions) > 1:
            set_gl_color(Color.kCyan)
            gl.glLineWidth(2)

            # Draw trajectory line
            traj_points = positions[:frame_idx+1]
            if len(traj_points) > 1:
                for i in range(len(traj_points) - 1):
                    pangolin.DrawLine([traj_points[i], traj_points[i+1]])

        # Draw start point (first frame position) as a large black point
        if len(positions) > 0:
            set_gl_color(Color.kBlack)
            gl.glPointSize(15.0)  # Large point size
            pangolin.DrawPoints([positions[0]])

        # Draw current pose (IMU frame, 0.4m axes)
        # IMU coordinate frame (I): T_wi transformation from IMU to World
        # Shows the current IMU pose in world frame
        # X-axis: Red, Y-axis: Green, Z-axis: Blue
        if show_current_pose.Get() and frame_idx < len(positions):
            current_pos = positions[frame_idx]
            current_ori = orientations[frame_idx]

            # Create transform matrix T_wi (World <- IMU)
            T = np.eye(4)
            R = quaternion_to_rotation_matrix(current_ori[0], current_ori[1], current_ori[2], current_ori[3])
            T[:3, :3] = R
            T[:3, 3] = current_pos

            # Draw IMU pose axes (0.4m)
            draw_coordinate_frame(T, length=0.4, line_width=2)

        # Draw camera frames (as frustums, 0.4m scale)
        # Camera coordinate frames (C0, C1): T_wc0 and T_wc1 transformations
        # T_wc = T_wi * T_ic (World <- IMU <- Camera)
        # cam0 (left): Magenta frustum, cam1 (right): Cyan frustum
        # Camera convention: Z-axis forward, X-axis right, Y-axis down
        if show_cam_frames.Get() and frame_idx < len(positions):
            current_pos = positions[frame_idx]
            current_ori = orientations[frame_idx]

            # Create IMU transform matrix T_wi (World <- IMU)
            T_imu = np.eye(4)
            R = quaternion_to_rotation_matrix(current_ori[0], current_ori[1], current_ori[2], current_ori[3])
            T_imu[:3, :3] = R
            T_imu[:3, 3] = current_pos

            # T_cam0 = T_wi * T_ic0 (World <- IMU <- cam0)
            # T_cam1 = T_wi * T_ic1 (World <- IMU <- cam1)
            # Using typical stereo camera setup: cameras along X axis of IMU
            # cam0: left, cam1: right, both looking forward along Z axis of IMU
            # These are approximate; replace with actual extrinsics if available
            T_ic0 = np.eye(4)  # cam0 = IMU frame (identity as placeholder)
            T_ic1 = np.eye(4)  # cam1 = IMU frame (identity as placeholder)

            T_cam0 = T_imu @ T_ic0
            T_cam1 = T_imu @ T_ic1

            # Draw cam0 frustum (magenta, 0.4m)
            draw_camera_frustum(T_cam0, scale=0.4, line_width=2, color=Color.kMagenta)

            # Draw cam1 frustum (cyan, 0.4m)
            draw_camera_frustum(T_cam1, scale=0.4, line_width=2, color=Color.kCyan)

        # Draw velocity vector
        if show_velocity.Get() and frame_idx < len(odometry_data):
            twist = odometry_data[frame_idx]['twist']
            vel = twist['linear']
            vel_vector = np.array([vel['x'], vel['y'], vel['z']])

            # Get current position
            if frame_idx < len(positions):
                current_pos = positions[frame_idx]

                # Scale velocity for visualization (multiply by time to show displacement)
                vel_scale = 1.0  # 1 second displacement
                vel_end = current_pos + vel_vector * vel_scale

                # Draw velocity arrow
                draw_arrow(current_pos, vel_end, line_width=2, color=Color.kGreen)

        # Draw covariance ellipse (position part only)
        if show_covariance.Get() and frame_idx < len(covariances):
            current_pos = positions[frame_idx]
            pos_cov = covariances[frame_idx][:3, :3]

            # Draw uncertainty ellipse (3-sigma)
            try:
                eigenvalues, eigenvectors = np.linalg.eigh(pos_cov)
                idx = eigenvalues.argsort()[::-1]
                eigenvalues = eigenvalues[idx]
                eigenvectors = eigenvectors[:, idx]

                radii = np.sqrt(eigenvalues) * 3.0  # 3-sigma

                set_gl_color(Color.kYellow)
                gl.glLineWidth(1)

                # Draw principal axes
                for i in range(3):
                    axis = eigenvectors[:, i] * radii[i]
                    start = current_pos - axis
                    end = current_pos + axis
                    pangolin.DrawLine([start, end])
            except:
                pass

        # Draw MSCKF pointcloud
        if show_pointcloud.Get() and frame_idx < len(points_data):
            points_frame = points_data[frame_idx]
            if 'points' in points_frame and len(points_frame['points']) > 0:
                # Extract point positions
                pts = []
                for pt in points_frame['points']:
                    pts.append([pt['x'], pt['y'], pt['z']])

                if len(pts) > 0:
                    pts = np.array(pts)

                    # Draw points in world frame (assuming points are in IMU frame)
                    # Transform to world frame using current pose
                    if frame_idx < len(positions):
                        current_pos = positions[frame_idx]
                        current_ori = orientations[frame_idx]

                        # Create transform matrix
                        T = np.eye(4)
                        R = quaternion_to_rotation_matrix(current_ori[0], current_ori[1], current_ori[2], current_ori[3])
                        T[:3, :3] = R
                        T[:3, 3] = current_pos

                        # Transform points to world frame
                        pts_world = transform_points(T, pts)

                        # Draw points
                        set_gl_color(Color.kOrange)
                        gl.glPointSize(3.0)
                        pangolin.DrawPoints(pts_world)

        # Draw dual camera images (right top)
        if show_cam_images.Get() and frame_idx < len(frame_times):
            frame_time = frame_times[frame_idx]
            cam0_img = load_camera_image(data_dir, 'cam0', frame_time)
            cam1_img = load_camera_image(data_dir, 'cam1', frame_time)

            combined_img = create_dual_camera_image(cam0_img, cam1_img, cam_img_w, cam_img_h)
            combined_rgb = cv2.cvtColor(combined_img, cv2.COLOR_BGR2RGB)
            cam_text.Upload(combined_rgb, gl.GL_RGB, gl.GL_UNSIGNED_BYTE)
            dcamimg.Activate()
            gl.glColor3f(1.0, 1.0, 1.0)
            cam_text.RenderToViewportFlipY()

        # Create and draw text information
        txt_image = create_text_image(
            frame_idx, frame_size, odometry_data, frame_distances, points_data
        )

        # Convert to RGB and upload
        txt_image_rgb = cv2.cvtColor(txt_image, cv2.COLOR_BGR2RGB)
        txt_text.Upload(txt_image_rgb, gl.GL_RGB, gl.GL_UNSIGNED_BYTE)
        dtxtimg.Activate()
        gl.glColor3f(1.0, 1.0, 1.0)
        txt_text.RenderToViewportFlipY()

        pangolin.FinishFrame()


if __name__ == '__main__':
    # Default parameters
    data_dir = "/home/jerett/OpenProject/LidarSlam/open-vins/src/scripts/data/open_vins_20260429_232853"

    main(data_dir)