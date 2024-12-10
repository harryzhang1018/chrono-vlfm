import numpy as np
import pychrono as chrono

def turn_left(vir_robot):
    # rot_state = vir_robot.GetRot().GetCardanAnglesXYZ()
    vir_robot.SetRot(chrono.QuatFromAngleY(np.pi/6)*vir_robot.GetRot())

def turn_right(vir_robot):
    vir_robot.SetRot(chrono.QuatFromAngleY(-np.pi/6)*vir_robot.GetRot())

def move_forward(vir_robot):
    rot_state = vir_robot.GetRot().GetCardanAnglesXYZ()
    vir_robot.SetPos(vir_robot.GetPos()+chrono.ChVector3d(0.01*np.sin(rot_state.y+np.pi/2), 0, 0.01*np.cos(rot_state.y+np.pi/2)))


def quaternion_to_pitch(quaternion):
    # Unpack quaternion
    w, x, y, z = quaternion
    
    pitch = np.arctan2(2 * (w * y + x * z), 1 - 2 * (y**2 + z**2))
    return pitch

if __name__ == "__main__":
    print("This is a utility module")
    quat = [[ 0.965926, 0, 0.258819, 0 ],
            [ 0.866025, 0, 0.5, 0 ],
            [ 0.707107, 0, 0.707107, 0 ],
            [ 0.5, 0, 0.866025, 0 ],
            [ 0.258819, 0, 0.965926, 0 ],
            [ 1.82036e-16, 0, 1, 0 ],
            [ -0.258819, 0, 0.965926, 0 ],
            [ -0.5, 0, 0.866025, 0 ],
            [ -0.707107, 0, 0.707107, 0 ],
            [ -0.866025, 0, 0.5, 0 ],
            [ -0.965926, 0, 0.258819, 0 ],
            [ -1, 0, 3.08107e-16, 0 ],
            [ -0.965926, 0, -0.258819, 0 ],
            [ -0.866025, 0, -0.5, 0 ],
            [ -0.707107, 0, -0.707107, 0 ],
            [ -0.5, 0, -0.866025, 0 ],
            [ -0.258819, 0, -0.965926, 0 ],
            [ -4.59592e-16, 0, -1, 0 ],
            [ 0.258819, 0, -0.965926, 0 ],
            [ 0.5, 0, -0.866025, 0 ],
            [ 0.707107, 0, -0.707107, 0 ],
            [ 0.866025, 0, -0.5, 0 ],
            [ 0.965926, 0, -0.258819, 0 ],
            [ 1, 0, -6.81553e-16, 0 ],
            [ 0.965926, 0, 0.258819, 0 ],
            [ 0.866025, 0, 0.5, 0 ],
            [ 0.707107, 0, 0.707107, 0 ],
            [ 0.5, 0, 0.866025, 0 ],
            [ 0.258819, 0, 0.965926, 0 ],
            [ 8.72988e-16, 0, 1, 0 ],
            [ -0.258819, 0, 0.965926, 0 ]]
    
    euler_angles_y = [quaternion_to_pitch(q) for q in quat]
    print(euler_angles_y)