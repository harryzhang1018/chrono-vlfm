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