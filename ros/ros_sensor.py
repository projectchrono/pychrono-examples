# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2024 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Simulation Based Engineering Lab, University of Wisconsin-Madison
# =============================================================================

import pychrono as ch
import pychrono.sensor as sens
import pychrono.ros as chros

def main():
    # Create the Chrono simulation system.
    sys = ch.ChSystemNSC()

    # Add a mesh object to the simulation for visual interest.
    mmesh = ch.ChTriangleMeshConnected()
    # Load and transform a 3D mesh of a vehicle chassis.
    mmesh.LoadWavefrontMesh(ch.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    mmesh.Transform(ch.ChVector3d(0, 0, 0), ch.ChMatrix33d(1))

    # Create a visual shape from the mesh.
    trimesh_shape = ch.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)

    # Create a body for the mesh and add it to the simulation.
    mesh_body = ch.ChBody()
    mesh_body.SetPos(ch.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(False)  # Make the body movable.
    mesh_body.SetMass(0)  # Set mass to 0 (static object).
    sys.Add(mesh_body)

    # Create a ground body to attach sensors.
    ground_body = ch.ChBodyEasyBox(1, 1, 1, 1000, False, False)
    ground_body.SetPos(ch.ChVector3d(0, 0, 0))
    ground_body.SetFixed(False)  # Make the body movable.
    ground_body.SetMass(0)  # Set mass to 0 (static object).
    sys.Add(ground_body)

    # Create the sensor manager.
    sens_manager = sens.ChSensorManager(sys)

    # Add point lights to the scene for better visualization.
    intensity = 1.0
    sens_manager.scene.AddPointLight(ch.ChVector3f(2, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVector3f(9, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVector3f(16, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVector3f(23, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)

    # Create and configure a camera sensor.
    offset_pose = ch.ChFramed(ch.ChVector3d(-8, 0, 2), ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    cam = sens.ChCameraSensor(ground_body, 30, offset_pose, 1280, 720, 1.408)
    cam.PushFilter(sens.ChFilterVisualize(1280, 720))  # Visualize the camera output.
    cam.PushFilter(sens.ChFilterRGBA8Access())  # Access raw RGBA8 data.
    cam.SetName("camera")
    sens_manager.AddSensor(cam)

    # Create and configure a lidar sensor.
    lidar = sens.ChLidarSensor(ground_body, 5., offset_pose, 90, 300, 2*ch.CH_PI, ch.CH_PI / 12, -ch.CH_PI / 6, 100., 0)
    lidar.PushFilter(sens.ChFilterDIAccess())  # Access raw lidar data.
    lidar.PushFilter(sens.ChFilterPCfromDepth())  # Convert depth data to point cloud.
    lidar.PushFilter(sens.ChFilterXYZIAccess())  # Access point cloud data.
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, 1))  # Visualize the point cloud.
    lidar.SetName("lidar")
    sens_manager.AddSensor(lidar)

    # Create and configure a GPS sensor.
    noise_model_none = sens.ChNoiseNone()
    gps_reference = ch.ChVector3d(-89.4, 433.07, 260.)
    gps = sens.ChGPSSensor(ground_body, 10, offset_pose, gps_reference, noise_model_none)
    gps.PushFilter(sens.ChFilterGPSAccess())  # Access GPS data.
    gps.SetName("gps")
    sens_manager.AddSensor(gps)

    # Create and configure an accelerometer sensor.
    acc = sens.ChAccelerometerSensor(ground_body, 100, offset_pose, noise_model_none)
    acc.PushFilter(sens.ChFilterAccelAccess())  # Access accelerometer data.
    acc.SetName("accelerometer")
    sens_manager.AddSensor(acc)

    # Create and configure a gyroscope sensor.
    gyro = sens.ChGyroscopeSensor(ground_body, 100, offset_pose, noise_model_none)
    gyro.PushFilter(sens.ChFilterGyroAccess())  # Access gyroscope data.
    gyro.SetName("gyroscope")
    sens_manager.AddSensor(gyro)

    # Create and configure a magnetometer sensor.
    mag = sens.ChMagnetometerSensor(ground_body, 100, offset_pose, noise_model_none, gps_reference)
    mag.PushFilter(sens.ChFilterMagnetAccess())  # Access magnetometer data.
    mag.SetName("magnetometer")
    sens_manager.AddSensor(mag)

    # Update the sensor manager to initialize sensors.
    sens_manager.Update()

    # Create the ROS manager and register handlers for the sensors.
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())  # Register the clock handler.

    # Register handlers for each sensor, specifying ROS topics for output.
    ros_manager.RegisterHandler(chros.ChROSCameraHandler(cam.GetUpdateRate() / 4, cam, "~/output/camera/data/image"))
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/data/pointcloud"))
    ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))
    acc_handler = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")
    ros_manager.RegisterHandler(acc_handler)
    gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
    ros_manager.RegisterHandler(gyro_handler)
    mag_handler = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
    ros_manager.RegisterHandler(mag_handler)

    # Create and configure an IMU handler.
    imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")
    imu_handler.SetAccelerometerHandler(acc_handler)
    imu_handler.SetGyroscopeHandler(gyro_handler)
    imu_handler.SetMagnetometerHandler(mag_handler)
    ros_manager.RegisterHandler(imu_handler)

    ros_manager.Initialize()  # Initialize the ROS manager.

    # Run the simulation loop.
    time = 0
    time_step = 1e-3  # Define the simulation time step.
    time_end = 100  # Set the duration of the simulation.

    # Apply rotational velocity to the ground body for sensor motion.
    ground_body.SetAngVelParent(ch.ChVector3d(0, 0, 0.1))

    while time < time_end:
        time = sys.GetChTime()  # Update simulation time.

        # Update sensors and ROS data.
        sens_manager.Update()
        if not ros_manager.Update(time, time_step):
            break  # Exit loop if ROS manager update fails.

        sys.DoStepDynamics(time_step)  # Advance the simulation by one time step.

if __name__ == "__main__":
    main()
