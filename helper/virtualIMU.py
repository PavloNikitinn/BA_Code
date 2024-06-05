import vpython as vp
import numpy as np
#Virtual IMU to see what happens with data
class virtualIMU:
    def __init__(self, pos=vp.vector(0, 0, 0), size=vp.vector(1, 1, 1)):
        self.scene = vp.canvas(title="IMU Orientation", width=800, height=700, align="left", resizable=True)
        self.rig = vp.box(length=0.5, width=0.5, height=0.2,  pos=vp.vector(0, 0, 0), make_trail=True)
        self.imu = vp.box(length=1, width=1, height=0.2,  pos=vp.vector(2, 0.2, 0), color=vp.color.red)
        self.x_arrow = vp.arrow(length=0.1, shaftwidth=0.1, color=vp.color.red, axis=vp.vector(-1, 0, 0))
        self.y_arrow = vp.arrow(length=0.1, shaftwidth=0.1, color=vp.color.green, axis=vp.vector(0, -1, 0))
        self.z_arrow = vp.arrow(length=0.1, shaftwidth=0.1, color=vp.color.blue, axis=vp.vector(0, 0, 1))
        self.unit_vec = vp.vector(0,1,0)
        self.unit_arrow = vp.arrow(length=2, shaftwidth=0.1, color=vp.color.purple, axis=vp.vector(self.unit_vec))

        self.scene.camera.pos = vp.vector(0, 0, 5)
        self.scene.camera.axis = vp.vector(0, 0, -1)
        #self.scene.autoscale = True
        self.box = vp.compound([self.rig,self.imu])



        self.x_arrowIMU = vp.arrow(length=2, shaftwidth=0.1, color=vp.color.red, axis=vp.vector(-1, 0, 0))
        self.y_arrowIMU = vp.arrow(length=2, shaftwidth=0.1, color=vp.color.green, axis=vp.vector(0, -1, 0))
        self.z_arrowIMU = vp.arrow(length=2, shaftwidth=0.1, color=vp.color.blue, axis=vp.vector(0, 0, 1))

    
    def set_position(self, new_position):
        self.box.pos.x = new_position[0]
        self.box.pos.z = new_position[2]
        self.scene.camera.pos = vp.vector(new_position[0]-3, 5,new_position[2]-3 )

        

    def set_orientation(self, new_orientation):
 
        self.box.rotate(angle=vp.radians(new_orientation[0]), axis=vp.vector(1,0,0))
        self.box.rotate(angle=vp.radians(new_orientation[2]), axis=vp.vector(0,1,0))
        self.box.rotate(angle=vp.radians(new_orientation[1]), axis=vp.vector(0,0,1))

        #self.imu.pos = self.box.pos + vp.vector(0, self.box.size.y,0)

    def set_camera(self, camera_pos, camera_axis, camera_up):
        vp.scene.camera.pos = camera_pos
        vp.scene.camera.axis = camera_axis
        vp.scene.camera.up = camera_up

    def add_data_to_curve(self,new_data_point):
        self.curve.plot(new_data_point)

