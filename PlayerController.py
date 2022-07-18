from numpy import clip as Clamp
from numpy import interp as Lerp
import bge
from collections import OrderedDict
import time
from mathutils import Vector
from copy import deepcopy
import math
import MathLib
import sys

全局字典2 = bge.logic.globalDict
全局字典2['位置']=None
全局字典2['W按下']=0
全局字典2['S按下']=0
全局字典2['A按下']=0
全局字典2['D按下']=0
全局字典2['P按下']=0
全局字典2['J按下']=0
全局字典2['K按下']=0
全局字典2['L按下']=0
全局字典2['M按下']=0
全局字典2['B按下']=0
全局字典2['Z按下']=0
全局字典2['载具坐标']={}
全局字典2['欧拉角Z']=-1.57
全局字典2['载具新坐标']=Vector([142.734, 214.093, 0.3971])
全局字典2['载具方向角']=-1.57
全局字典2['载具速度']=0
全局字典2['轮向角']=0
全局字典2['记录碰撞']=True
if not hasattr(bge, "__component__"):
    pc = bge.constraints
    events = bge.events
    render = bge.render
    k = bge.logic.keyboard.inputs
    scene = bge.logic.getCurrentScene()

    logic = bge.logic



class PlayerController(bge.types.KX_PythonComponent):
    args = OrderedDict([
    ])

    def start(self, args):

        self.载具部件 = self.object.components[0]


        self.摄像机 = [
            obj for obj in self.载具部件.载具.childrenRecursive if obj.__class__.__name__ == "KX_Camera"]
        self.SteerWheel = [
            obj for obj in self.载具部件.载具.childrenRecursive if "Volante" in obj][0]
        self.轮向标 = [
            obj for obj in self.载具部件.载具.childrenRecursive if "轮向标"  in obj][0]
        self.CamParent = [
            obj for obj in self.载具部件.载具.childrenRecursive if "CamParent" in obj][0]
        self.CamParentPos = self.CamParent.parent
        self.CamParent.removeParent()
        self.CurrentCamera = 0

        self.RotVolZero = self.SteerWheel.localOrientation.to_euler()

        scene.pre_draw_setup.append(self.PreUpdate)
        

    def PreUpdate(self):
        self.CamParent.worldPosition = self.CamParentPos.worldPosition
        self.CamParent.worldOrientation = self.CamParent.worldOrientation.lerp(
            self.CamParentPos.worldOrientation, self.载具部件.载具.DeltaTime*11)

    def update(self):

        Axis = [0, 0, 0, 0, 0, 0]
        Buttons = []

        try:
            Axis = bge.logic.joysticks[0].axisValues
            Buttons = bge.logic.joysticks[0].activeButtons
        except:
            pass


        RightRaw = Axis[0]
       
        SignRight = MathLib.Sign(RightRaw)
        DeadzoneRight = 0.0
        RightFinal = MathLib.MapRange(
            RightRaw, 0.0, SignRight, DeadzoneRight*SignRight, SignRight)
        RightFinal = RightFinal * MathLib.LerpF(abs(RightFinal), 1.0, 0.05)
        self.向右移动 = Clamp(RightFinal, -1.0, 1.0)

        ForwardRaw = -Axis[3]
        SignForward = MathLib.Sign(ForwardRaw)
        DeadzoneForward = -0.00
        self.向前移动 = MathLib.MapRange(
            ForwardRaw, 0.0, SignForward, DeadzoneForward*SignForward, SignForward)

        #按键转游戏内变量
        if k[events.WKEY].values[-1]!=0:
            全局字典2['W按下']=1
        else:
            全局字典2['W按下']=0
        if k[events.SKEY].values[-1]!=0:
            全局字典2['S按下']=1
        else:
            全局字典2['S按下']=0
        if k[events.AKEY].values[-1]!=0:
            全局字典2['A按下']=1
        else:
            全局字典2['A按下']=0
        if k[events.DKEY].values[-1]!=0:
            全局字典2['D按下']=1
        else:
            全局字典2['D按下']=0
        if k[events.PKEY].values[-1]!=0:
            全局字典2['P按下']=1
        else:
            全局字典2['P按下']=0
        if k[events.JKEY].values[-1]!=0:
            全局字典2['J按下']=1
        else:
            全局字典2['J按下']=0
        if k[events.KKEY].values[-1]!=0:
            全局字典2['K按下']=1
        else:
            全局字典2['K按下']=0
        if k[events.LKEY].values[-1]!=0:
            全局字典2['L按下']=1
        else:
            全局字典2['L按下']=0        
        if k[events.MKEY].values[-1]!=0:
            全局字典2['M按下']=1
        else:
            全局字典2['M按下']=0    
        #Keyboard

        
        Throttle = (全局字典2['W按下']-全局字典2['S按下'])
        if Throttle != 0 and 全局字典2['S按下']==0:
            
            self.向前移动 = Throttle*0.5
        else:
            self.向前移动 = Throttle


        转向 = (全局字典2['D按下']-全局字典2['A按下'])
        if 转向 != 0:
            self.向右移动 = 转向

        # Apply input
        全局字典2['位置']=self.载具部件.载具.worldPosition

        if 全局字典2['J按下']!=0:
            self.载具部件.载具.传动系统.SetGear(-1)

        if 全局字典2['K按下']!=0:
            self.载具部件.载具.传动系统.SetGear(1)
        if 全局字典2['L按下']!=0:
            self.载具部件.载具.传动系统.SetGear(2)
        if 全局字典2['M按下']!=0:
            self.载具部件.载具.传动系统.SetGear(3)
        if 全局字典2['P按下']!=0:
            全局字典2['碰撞次数']=0
            self.载具部件.载具.worldPosition=全局字典2['载具新坐标']
            欧拉角=self.载具部件.载具.worldOrientation
            欧拉角=欧拉角.to_euler()
            欧拉角[0]=0.0
            欧拉角[1]=0.0
            欧拉角[2]=全局字典2['欧拉角Z']
            self.载具部件.载具.worldOrientation=欧拉角.to_matrix()
            self.载具部件.载具.worldLinearVelocity = Vector([0, 0, 0.00001])
            self.载具部件.载具.Wheels[0].转向.SteeringAngle=0.0
            self.载具部件.载具.Wheels[2].转向.SteeringAngle=0.0
            self.载具部件.载具.EngineAngularVelocity=0.0
            SteerV = self.载具部件.载具.Wheels[0].转向.SteeringAngle*12
            SteerV2 = self.载具部件.载具.Wheels[0].转向.SteeringAngle*-1
            RotVol = self.RotVolZero.copy()
            RotVol2 = self.RotVolZero.copy()
            RotVol.rotate_axis('Y', SteerV)
            RotVol2.rotate_axis('Z', SteerV2)
            目标位置 = RotVol.to_matrix()
            目标位置2 = RotVol2.to_matrix()
            self.SteerWheel.localOrientation = 目标位置
            self.轮向标.localOrientation = 目标位置2
            self.载具部件.载具.localAngularVelocity=Vector([0, 0, 0.00000])
            全局字典2['碰撞次数']=0
            全局字典2['P按下']=0
            
        
        全局字典2['载具坐标']['x坐标']=self.载具部件.载具.worldPosition.x
        全局字典2['载具坐标']['y坐标']=self.载具部件.载具.worldPosition.y
        欧拉角=self.载具部件.载具.worldOrientation
        欧拉角=欧拉角.to_euler()
        全局字典2['载具方向角']=欧拉角[2]
        全局字典2['载具方向角y']=欧拉角[1]
        全局字典2['载具方向角x']=欧拉角[0]
        全局字典2['载具速度']=self.载具部件.载具.速度公里每小时
        if 全局字典2['B按下']!=0:
            全局字典2['worldPosition1']=self.载具部件.载具.worldPosition.copy()

            全局字典2['worldOrientation1']=self.载具部件.载具.worldOrientation.copy()
            全局字典2['worldLinearVelocity1']=self.载具部件.载具.worldLinearVelocity.copy()
            全局字典2['localAngularVelocity1']=self.载具部件.载具.localAngularVelocity.copy()
            print(全局字典2['worldPosition1'],',worldPosition-----------------------------------------')


        if 全局字典2['Z按下']!=0:
            self.载具部件.载具.worldPosition=全局字典2['worldPosition1']
            self.载具部件.载具.worldOrientation=全局字典2['worldOrientation1']
            self.载具部件.载具.worldLinearVelocity =全局字典2['worldLinearVelocity1']
            self.载具部件.载具.localAngularVelocity  = 全局字典2['localAngularVelocity1']
            print(',ZKEY-----------------------------------------',全局字典2['Z按下'])               
            全局字典2['Z按下']=0
        self.载具部件.载具.置油门(Clamp(self.向前移动, -1.0, 1.0))
        self.载具部件.载具.SetBrake(abs(Clamp(self.向前移动, -2.0, 00)))


        if self.向右移动!= 0:
            角度增量=self.向右移动*0.78539815/128
            角度=Clamp(角度增量+self.载具部件.载具.Wheels[0].转向.SteeringAngle, -0.78539815, 0.78539815)
            
            self.载具部件.载具.Wheels[0].转向.SteeringAngle=角度
            self.载具部件.载具.Wheels[2].转向.SteeringAngle=角度

        全局字典2['轮向角']=self.载具部件.载具.Wheels[2].转向.SteeringAngle
        # SteerWheel
        SteerV = self.载具部件.载具.Wheels[0].转向.SteeringAngle*12
        SteerV2 = self.载具部件.载具.Wheels[0].转向.SteeringAngle*-1
        RotVol = self.RotVolZero.copy()
        RotVol2 = self.RotVolZero.copy()
        RotVol.rotate_axis('Y', SteerV)
        RotVol2.rotate_axis('Z', SteerV2)
        目标位置 = RotVol.to_matrix()
        目标位置2 = RotVol2.to_matrix()
        self.SteerWheel.localOrientation = 目标位置
        self.轮向标.localOrientation = 目标位置2

        if 1 in k[events.CKEY].queue:
            print(self.载具部件.载具.GetDebugValues())
            self.CurrentCamera += 1
            if self.CurrentCamera >= len(self.摄像机):
                self.CurrentCamera = 0
            scene.active_camera = self.摄像机[self.CurrentCamera]
