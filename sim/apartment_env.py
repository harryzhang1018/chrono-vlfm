import sys
import os
# Assuming the script is located in the 'experiments/apartment' directory
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(project_root)
# Add the parent directory of 'models' to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import math
from util import turn_left, turn_right, move_forward
my_system = chrono.ChSystemSMC()
my_system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  



patch_mat = chrono.ChContactMaterialSMC()


vir_robot = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 100, True, True,patch_mat)
vir_robot.SetPos(chrono.ChVector3d(0, 0.25, 0))
vir_robot.SetFixed(True)
my_system.Add(vir_robot)

mmesh = chrono.ChTriangleMeshConnected()
mmesh.LoadWavefrontMesh(project_root + '/data/environment/flat_env.obj', False, True)

# scale to a different size
# mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))

trimesh_shape = chrono.ChVisualShapeTriangleMesh()
trimesh_shape.SetMesh(mmesh)
# trimesh_shape.SetScale(chrono.ChVector3d(10, 10, 10))
trimesh_shape.SetName("ENV MESH")
trimesh_shape.SetMutable(False)

mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.AddVisualShape(trimesh_shape)
mesh_body.SetFixed(True)
my_system.Add(mesh_body)


# ---------------------------------------

# Add camera sensor

# -----------------
# Camera parameters
# -----------------

# Output directory
out_dir = "SENSOR_OUTPUT/"

# Camera lens model
lens_model = sens.PINHOLE

# Update rate in Hz
update_rate = 30

# Image width and height
image_width = 640
image_height = 480

# Camera's horizontal field of view
fov = 1.408

# Lag (in seconds) between sensing and when data becomes accessible
lag = 0

# Exposure (in seconds) of each image
exposure_time = 0

manager = sens.ChSensorManager(my_system)

intensity = 1.0
manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1,0,0), chrono.ChVector3f(0,-1,0))




offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0.5, 0), chrono.Q_ROTATE_Z_TO_Y)

lidar = sens.ChLidarSensor(
    vir_robot,             # body lidar is attached to
    update_rate,                     # scanning rate in Hz
    offset_pose,            # offset pose
    image_width,                   # number of horizontal samples
    image_height,                    # number of vertical channels
    1.6,                    # horizontal field of view
    chrono.CH_PI/6,         # vertical field of view
    -chrono.CH_PI/6,
    5.0,                  # max lidar range
    sens.LidarBeamShape_RECTANGULAR,
    1,          # sample radius
    0,       # divergence angle
    0,       # divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN)

lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1/20)
lidar.PushFilter(sens.ChFilterVisualize(image_width, image_height, "depth camera"))
lidar.PushFilter(sens.ChFilterDIAccess())
manager.AddSensor(lidar)

cam = sens.ChCameraSensor(
    vir_robot,              # body camera is attached to
    update_rate,            # update rate in Hz
    offset_pose,            # offset pose
    image_width,            # image width
    image_height,           # image height
    fov                    # camera's horizontal field of view
    )


cam.SetName("Camera Sensor")
cam.SetLag(lag)
cam.SetCollectionWindow(exposure_time)
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "rgb camera"))
# Provides the host access to this RGBA8 buffer
cam.PushFilter(sens.ChFilterRGBA8Access())

# add sensor to manager
manager.AddSensor(cam)

# ---------------------------------------
    
### Create visualization for the gripper fingers
vis = chronoirr.ChVisualSystemIrrlicht(my_system, chrono.ChVector3d(-2, 1, -1))
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.AddLightWithShadow(chrono.ChVector3d(6,6,6),  # point
                      chrono.ChVector3d(0,6,0),  # aimpoint
                      5,                       # radius (power)
                      1,11,                     # near, far
                      55)                       # angle of FOV

# vis.EnableShadows()
# vis.EnableCollisionShapeDrawing(True)
vis.EnableAbsCoordsysDrawing(True)
vis.EnableBodyFrameDrawing(True)

timestep = 0.001
render_step_size = 1.0 / 25  # FPS = 50
control_step_size = 1.0 / 20  # 10 Hz
render_steps = math.ceil(render_step_size / timestep)
control_steps = math.ceil(control_step_size / timestep)
step_number = 0
render_frame = 0

rt_timer = chrono.ChRealtimeStepTimer()

isturn_left = False
isturn_right = False
ismove_forward = False
while vis.Run():
    manager.Update()

    sim_time = my_system.GetChTime()
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        # filename = './IMG_jackal/img_' + str(render_frame) +'.jpg' 
        # vis.WriteImageToFile(filename)
        # render_frame += 1
        rot_state = vir_robot.GetRot().GetCardanAnglesXYZ()
        if 1 < sim_time < 2:
            if not isturn_left:
                print('turning left')
                turn_left(vir_robot)
                isturn_left = True
        if 2 < sim_time < 3:
            if not ismove_forward:
                print('moving forward')
                move_forward(vir_robot)
                ismove_forward = True
        if 3 < sim_time < 4:
            if not isturn_right:
                print('turning right')
                turn_right(vir_robot)
                isturn_right = True


    my_system.DoStepDynamics(timestep)
    # data logging and control applied at frequency control_step_size
    if step_number % control_steps == 0:
        # depth related data:
        depth_buffer = lidar.GetMostRecentDIBuffer()
        if depth_buffer.HasData():
            depth_data = depth_buffer.GetDIData()
            print('shape of depth data: ', depth_data.shape)
        # camera RGB data:
        camera_buffer = cam.GetMostRecentRGBA8Buffer()
        if camera_buffer.HasData():
            camera_data = camera_buffer.GetRGBA8Data()
            print('shape of camera data: ', camera_data.shape)
    
    rt_timer.Spin(timestep)
    step_number += 1