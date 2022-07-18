import bge
from collections import OrderedDict
import 载具物理性质
import MathLib
import math

if not hasattr(bge, "__component__"):
    scene = bge.logic.getCurrentScene()

class MyVehicle(bge.types.KX_PythonComponent):
    args = OrderedDict([
    ])

    def start(self, args):
        self.载具 = 载具物理性质.载具物理(self.object)
        print(self.object.childrenRecursive)
        self.摄影机 = [obj for obj in self.载具.childrenRecursive if "Camera" in obj][0]

        self.摄影机跟随 = False



        # scene = bge.logic.getCurrentScene()
        # cam1 = scene.objects["Camera.002"]
        # cam2 = scene.objects["Camera.001"]

        # cam1.useViewport = True
        # cam2.useViewport = True
        # cam2.setOnTop()

        # width = bge.render.getWindowWidth()
        # height = bge.render.getWindowHeight()
        # print(width,height,'height----------------------------')



        # bottom = 0
        # top = height

        # # here are the boundaries for the right viewport
        # left_1 = 0
        # right_1 = width

        # # here are the boundaries for the left viewport
        # left_2 = 0
        # right_2 = int(width/2)

        # # set player viewports with player1 at the right, player 2 at the left
        # cam1.setViewport( right_2, int(height/2), width, height)
        # cam2.setViewport( 0, 0, right_2, int(height/2))

    def update(self):

        if self.摄影机跟随:
            self.载入摄像机()
    
    def 载入摄像机(self):

        
        车辆局部旋转 = self.载具.局部方向.to_euler()
        车辆局部旋转.rotate_axis('X', math.radians(-5.0))
        车辆局部旋转[1] = 0.0

        目标位置 = 车辆局部旋转.to_matrix()
        
        
        速度 = self.载具.DeltaTime*5
        self.摄影机.localOrientation = self.摄影机.localOrientation.lerp(目标位置, 速度)
