import bge
from collections import OrderedDict
import time
import mathutils
from mathutils import Vector
import numpy
from numpy import clip as Clamp, interp as Lerp
from copy import deepcopy
import math
import MathLib
import sys

events = bge.events
render = bge.render
k = bge.logic.keyboard.inputs
scene = bge.logic.getCurrentScene()
logic = bge.logic


class Wheel():
    def __init__(self):
        # Constant Values
        self.半径 = 0.351867
        self.车轮质量 = 50.0
        self.双驱 = True

        # Realtime Values
        self.局部位置 = mathutils.Vector([0, 0, 0])
        self.LocalOrientation = mathutils.Matrix(
            [[1.0000, 0.0000,  0.0000], [0.0000, 1.0000, 0.0000], [0.0000, 0.0000,  1.0000]])
        self.局部轮速 = mathutils.Vector([0, 0, 0])
        self.整体轮速  = mathutils.Vector([0, 0, 0])
        self.车轮加速度 = mathutils.Vector([0, 0, 0])
        self.角速度 = 0.0
        self.车轮转动 = 0.0
        self.当前扭矩 = 0.0
        self.惯性 = 1.0
        self.角加速度 = 0.0

        # Temporary Values
        self.最后的坐标 = mathutils.Vector([0, 0, 0])
        self.最后的局部速度 = mathutils.Vector([0, 0, 0])
        self.最后的整体速度 = mathutils.Vector([0, 0, 0])
        self.最后的轴向量X = mathutils.Vector([0, 0, 0])

    def 更新实时数据(self, 对象_车轮, 对象_载具, Ray, Time):
        self.局部位置 = (Ray[1] - 对象_载具.worldPosition) @ \
            对象_载具.worldOrientation + Vector([0, 0, self.半径])
        self.局部位置.z = Clamp(self.局部位置.z, -50, 对象_车轮.悬挂.最大上升)
        if Time != 0:
            self.整体轮速 = (Ray[1]-self.最后的坐标) / Time
        self.局部轮速 = self.整体轮速 @对象_车轮.worldOrientation

        发动机转轮 = 0.0
        if self.双驱 and 对象_载具.传动系统.CurrentGear != 0:
            发动机转轮 = 对象_载具.EngineAngularVelocity

        if 对象_载具.传动系统.CurrentGear != 0:

            self.角速度 = (6.28318530718 * 发动机转轮) / \
                (60 * 对象_载具.传动系统.CurrentDriveRatio)
        else:

            self.角速度 = 0.0

        self.角速度 += self.角加速度
        self.车轮转动 = (self.局部轮速.y /
                              self.半径)+self.角速度
        if Time != 0:
            self.车轮加速度 = 对象_载具.localLinearVelocity-对象_载具.LastVelocity / \
                Time  


class 车轮转向():
    def __init__(self):
        # Constant Values
        self.双转向 = False
        self.最大转向角 = math.radians(45.0)

        # Realtime Values
        self.SteeringAngle = 0.0

    def 更新转向角(self, 对象_车轮, 对象_载具, 转向, Sensibility, Time):
        SteeringCurve = 1.0 - \
            MathLib.MapRangeClamped(
                对象_载具.速度公里每小时, 10.0, 150.0, 0.0, 0.85)

        WheelRot = 对象_车轮.localOrientation.to_euler()
        WheelRot[2] = -self.SteeringAngle
        对象_车轮.localOrientation = WheelRot.to_matrix()


class 轮胎():
    def __init__(self):
        # Constant Values
        self.纵向刚度系数 = 0.06
        self.纵向形状系数 = 1.25
        self.纵向峰值 = 3.0
        self.纵向曲率系数 = 0.0
        self.轮胎纵向载荷 = 10.0
        self.轮胎纵向低速载荷 = 0.1

        self.侧向刚度系数 = 0.06
        self.侧向形状系数 = 1.25
        self.侧向峰值 = 3.0
        self.侧向曲率系数 = 0.0
        self.轮胎侧向载荷 = 10.0
        self.轮胎侧向低速载荷 = 1.0

        self.低速值 = 0.5
        self.滚动阻力系数 = 0.6

        # Realtime Values
        self.当前纵向滑动 = 0.0
        self.当前侧向滑动 = 0.0
        self.牵引力 = 0.0
        self.纵向力 = 0.0
        self.侧向力 = 0.0
        self.纵向滑动 = 0.0
        self.侧扁角 = 0.0
        self.滚动阻力 = 0.0
        self.Fy = 0.0
        self.Fx = 0.0
        self.纵向影响侧滑 = 1.5
        self.最大纵向影响侧滑 = 0.2

    def 取轮胎受力(self, 对象_车轮, 对象_载具, Time):
        车轮前向矢量 = 对象_车轮.getAxisVect(MathLib.ForwardVector)
        车轮右向矢量 = 对象_车轮.getAxisVect(MathLib.RightVector)
        车轮位置 = 对象_车轮.worldPosition - \
            对象_车轮.getAxisVect(MathLib.UpVector) * \
            对象_车轮.Wheel.半径

        悬挂作用力 = 对象_车轮.悬挂.当前力Z

        速度 = abs(对象_车轮.Wheel.局部轮速.x) + \
            abs(对象_车轮.Wheel.局部轮速.y)

        if not MathLib.NearFloat(速度, 0.0, self.低速值):
            self.纵向滑动 = ((对象_车轮.Wheel.角速度 * 对象_车轮.Wheel.半径 -
                          对象_车轮.Wheel.局部轮速.y) / 对象_车轮.Wheel.局部轮速.y)+1
            self.侧扁角 = MathLib.GetAngleBetweenVectors(对象_车轮.Wheel.整体轮速.normalized(
            ), 车轮右向矢量)*(1.0-Clamp(abs(self.纵向滑动*self.纵向影响侧滑), 0.0, self.最大纵向影响侧滑))

            self.Fy = -(self.纵向峰值*math.sin(self.纵向形状系数 * math.atan(self.纵向刚度系数 * self.纵向滑动 - self.纵向曲率系数 *
                                                                   (self.纵向刚度系数*self.纵向滑动-math.atan(self.纵向刚度系数*self.纵向滑动)))))*悬挂作用力/self.轮胎纵向载荷  # + SVy
            self.Fx = -(self.侧向峰值 * math.sin(self.侧向形状系数 * math.atan(self.侧向刚度系数 * self.侧扁角 - self.侧向曲率系数 *
                                                                     (self.侧向刚度系数*self.侧扁角-math.atan(self.侧向刚度系数*self.侧扁角)))))*悬挂作用力*self.轮胎侧向载荷  # + SVx
        else:
            self.纵向滑动 = 0.0
            self.侧扁角 = 0.0
            self.Fy = Clamp(-对象_车轮.Wheel.局部轮速.y *
                            悬挂作用力, -悬挂作用力, 悬挂作用力)*self.轮胎纵向低速载荷
            self.Fx = Clamp(-对象_车轮.Wheel.局部轮速.x *
                            悬挂作用力, -悬挂作用力, 悬挂作用力)*self.轮胎侧向低速载荷

        self.滚动阻力 = -self.滚动阻力系数 * 对象_车轮.Wheel.局部轮速.y
        if 对象_车轮.Wheel.双驱:
            if 对象_载具.传动系统.CurrentGear != 0:
                对象_车轮.Wheel.当前扭矩 = 对象_载具.CurrentEngineTorque * \
                    对象_载具.传动系统.CurrentDriveRatio
            else:
                对象_车轮.Wheel.当前扭矩 = Clamp(MathLib.LerpF(
                    对象_车轮.Wheel.当前扭矩, -0.1, Time*5), 0.0, 1000000.0)
            self.牵引力 = 对象_车轮.Wheel.当前扭矩 / 对象_车轮.Wheel.半径
        else:
            self.牵引力 = 0.0

        牵引转矩 = self.Fy * 对象_车轮.Wheel.半径
        总转矩 = self.牵引力 + 牵引转矩 + \
            (对象_载具.CurrentBrakeTorque * -
             MathLib.Sign(对象_车轮.Wheel.局部轮速.y))
        对象_车轮.Wheel.角加速度 = (
            总转矩 / 对象_车轮.Wheel.惯性) * Time

        self.纵向力 = 车轮前向矢量 * (总转矩 + self.滚动阻力 + self.Fy)

        self.侧向力 = 车轮右向矢量 * self.Fx



        return self.纵向力, self.侧向力


class 车轮悬挂():
    def __init__(self):
        # Constant Values
        self.高度 = 0.75
        self.弹簧刚度 = 45000.0
        self.减振器刚度 = 1500.0
        self.最大上升 = 0.12
        self.最大下沉 = 1.0
        self.强制应用高度 = 0.15
        self.强制纵向高度 = 0.15

        # Realtime Values
        self.当前力Z = 0.0
        self.当前长度 = 0.0
        self.当前速度 = 0.0
        self.起始位置 = Vector([0.0, 0.0, 0.0])

        # Temporary Values
        self.最终长度 = 0.0

    def 取悬挂力(self, 对象_车轮, Ray, Time):
        self.当前长度 = (self.起始位置-Ray[1]).length

        if Time != 0:
            self.当前速度 = (self.当前长度-self.最终长度)/Time

        长度限制 = Clamp(self.高度-self.当前长度, 0.0, 10000.0)
        当前弹簧刚度 = self.弹簧刚度
        当前阻尼 = self.减振器刚度
        self.当前力Z = Clamp(当前弹簧刚度 * 长度限制 -
                          当前阻尼 * self.当前速度, -1000.0, 100000.0)

        # Beta
        DotUpVect = Ray[2].dot(MathLib.UpVector)
        SideWheelRate = 10.0
        SideMinWheelRate = 0.15

        return (Ray[2].lerp(MathLib.UpVector, Clamp((DotUpVect+SideMinWheelRate)**SideWheelRate, 0.0, 1.0)))*self.当前力Z
        # return (Ray[2].lerp(UpVector, Clamp((DotUpVect+SideMinWheelRate)**SideWheelRate, 0.0, 1.0)))*self.CurrentForceZ
        # return (UpVector)*self.CurrentForceZ


class 车轮(bge.types.KX_GameObject):
    def __init__(self, oldOwner, 对象_载具, Setup=None):
        self.Enable = True
        self.WheelId = 0
        # print(self.WheelId)

        # Vehicle Setup
        self.对象_载具 = 对象_载具
        self.Wheel = Wheel()
        self.悬挂 = 车轮悬挂()
        self.轮胎 = 轮胎()
        self.转向 = 车轮转向()
        #print("Setup: ", Setup)

        if Setup != None:
            self.置车轮属性(Setup)

    def 置车轮属性(self, SetupValues):
        # Set Wheel Values
        #self.Wheel.WheelId = SetupValues['Wheel Id']
        self.Wheel.半径 = SetupValues['Wheel']['Radius']
        self.Wheel.车轮质量 = SetupValues['Wheel']['Mass']
        self.Wheel.惯性 = self.Wheel.车轮质量 * (self.Wheel.半径)
        #self.Wheel.Inertia = self.Wheel.Mass * (self.Wheel.半径 * self.Wheel.半径) /2
        self.Wheel.双驱 = SetupValues['Wheel']['bIsTraction']

        # Set Suspension Values
        self.悬挂.高度 = SetupValues['Suspension']['Height']
        self.悬挂.最大上升 = SetupValues['Suspension']['MaxRaise']
        self.悬挂.最大下沉 = SetupValues['Suspension']['MaxDrop']
        self.悬挂.弹簧刚度 = SetupValues['Suspension']['SpringStiffness'] * \
            (self.对象_载具.mass*0.001)
        self.悬挂.减振器刚度 = SetupValues['Suspension']['DamperStiffness'] * \
            (self.对象_载具.mass*0.001)
        self.悬挂.强制应用高度 = SetupValues['Suspension']['ForceAppHeight']
        self.悬挂.强制纵向高度 = SetupValues['Suspension']['ForceAppHeightLateral']

        # Set Tire Values
        self.轮胎.纵向刚度系数 = SetupValues['Tire']['LongStiffnessFactor']
        self.轮胎.纵向形状系数 = SetupValues['Tire']['LongShapeFactor']
        self.轮胎.纵向峰值 = SetupValues['Tire']['LongPeakValue']
        self.轮胎.纵向曲率系数 = SetupValues['Tire']['LongCurvatureFactor']
        self.轮胎.轮胎纵向载荷 = SetupValues['Tire']['LongTireLoad']

        self.轮胎.侧向刚度系数 = SetupValues['Tire']['LatStiffnessFactor']
        self.轮胎.侧向形状系数 = SetupValues['Tire']['LatShapeFactor']
        self.轮胎.侧向峰值 = SetupValues['Tire']['LatPeakValue']
        self.轮胎.侧向曲率系数 = SetupValues['Tire']['LatCurvatureFactor']
        self.轮胎.轮胎侧向载荷 = SetupValues['Tire']['LatTireLoad']
        self.轮胎.纵向影响侧滑 = SetupValues['Tire']['LongInfluenceOnLateralSlip']
        self.轮胎.最大纵向影响侧滑 = SetupValues['Tire']['MaxLongInfluenceOnLateralSlip']

        # Set Steering Values
        self.转向.双转向 = SetupValues['Steering']['bIsSteering']
        self.转向.最大转向角 = math.radians(
            SetupValues['Steering']['MaxSteeringAngle'])
