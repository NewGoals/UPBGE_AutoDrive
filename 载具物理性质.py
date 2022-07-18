import bge
from collections import OrderedDict
import time
from mathutils import Vector
import numpy
from numpy import clip as Clamp, interp as Lerp
from copy import deepcopy
import math
import MathLib
import sys
import VehicleWheel
import json

print("load VehiclePhysics successfully!. ")

events = bge.events
render = bge.render
k = bge.logic.keyboard.inputs
scene = bge.logic.getCurrentScene()
#Input = bge.logic.Input
logic = bge.logic


class Gear():
    def __init__(self, Ratio, ShiftUp=0.9, ShiftDown=0.5):
        self.Ratio = Ratio
        self.ShiftUp = ShiftUp
        self.ShiftDown = ShiftDown


class 传动系统():
    def __init__(self):
        # Costant Values
        self.前进档 = [Gear(4.06), Gear(3.5), Gear(2.2)]
        self.空档 = Gear(0.0)
        self.倒车档 = Gear(4.06)
        self.最终传动比 = 3.70

        # Realtime Values
        self.CurrentGear = 2
        self.CurrentDriveRatio = 1.0
        self.TimeChange = time.time()
        self.SetGear(2, True)

    def SetGear(self, Number, Instant=False):
        if time.time() - self.TimeChange > 0.15 or Instant:
            self.TimeChange = time.time()
            self.CurrentGear = Clamp(Number, -1, 3)
            if self.CurrentGear > 0:
                self.CurrentDriveRatio = self.前进档[self.CurrentGear -
                                                           1].Ratio * self.最终传动比
            elif self.CurrentGear == -1:
                self.CurrentDriveRatio = -self.倒车档.Ratio * self.最终传动比
            else:
                self.CurrentDriveRatio = self.空档.Ratio


class 载具基础属性(bge.types.KX_GameObject):
    def __init__(self, oldOwner):
        self.向前移动 = 0.0
        self.向右移动 = 0.0
        self.Brake = 0.0

    def 置油门(self, Value):
        self.向前移动 = Clamp(Value, -1.0, 1.0)

    def SetBrake(self, Value):
        self.Brake = Clamp(Value, 0.0, 2.0)

    def SetSteering(self, Value):


        self.向右移动 = Clamp(Value, -1.0, 1.0)
    def 置转向角(self, Value):
         self.向右移动 = Clamp(Value, -1.0, 1.0)



class 载具物理(载具基础属性):
    def __init__(self, Object):
        super().__init__(Object)
        self.Enable = True

        self.DeltaTime = 0.0
        self.EnableDebug = True

        self.Wheels = []
        self.NumDrivenWheels = 0
        self.CenterOfMassOffset = Vector([0.0, 0.0, -0.1])

        for ob in self.childrenRecursive:
            if "Collider" in ob:
                self.Collider = ob

                self.SetCenterOfMassOffset(self.CenterOfMassOffset)

            if "Wheel" in ob:
                child = ob.children[0]
                ob['wheelVisual'] = child
                child.removeParent()
                child.setParent(self)


                车轮Obj = VehicleWheel.车轮(
                    ob, self, self.LoadSetup(ob["Wheel"]))
                self.Wheels.append(车轮Obj)
                if 车轮Obj.Wheel.双驱:
                    self.NumDrivenWheels += 1

        # Constant Values
        self.EngineTorque = 600.0/self.NumDrivenWheels
        self.BrakeTorque = 3000
        self.MaxEngineTurnOver = 7500.0
        self.SteeringSensibility = 8.0
        self.传动系统 = 传动系统()

        # Realtime Values
        self.速度 = 0.0
        self.EngineTurnOver = 0.0
        self.EngineAngularVelocity = 0.0
        self.CurrentEngineTorque = 0.0
        self.CurrentBrakeTorque = 0.0

        self.LastForce = None
        self.LastTime = time.time()
        self.LastTimeTest = time.time()
        self.LastFixedUpdate = time.time()
        self.LastVelocity = self.localLinearVelocity.copy()
        self.PhysicsTime = time.time()
        scene.pre_draw_setup.append(self.PreUpdate)

    def 置油门(self, Value):
        self.向前移动 = Clamp(Value, 0.0, 1.0)


    def ReloadSetup(self):
        for i in self.Wheels:
            i.置车轮属性(self.LoadSetup(i["Wheel"]))

    def LoadSetup(self, SetupName):
        obj = None

        try:
            with open(logic.expandPath('//'+SetupName+'.json'), 'r') as myfile:
                data = myfile.read()
            obj = json.loads(data)
        except:
            print("Failed to open file: ", str(SetupName)+".json")

        return obj

    def DebugPhysics(self):
        for i in range(len(self.Wheels)):
            SuspensionName = "SuspensionVelocity{0}".format(i)

            if not SuspensionName in self:
                self.addDebugProperty(SuspensionName, True)

            self[SuspensionName] = self.Wheels[i].SuspensionVelocity

    def GetDebugValues(self):
        Values = ""
        Values += "Vehicle Velocity: {0:0.0f} KM/H\n".format(self.速度公里每小时)


        return Values

    def SetCenterOfMassOffset(self, Value):
        last = self.Collider.worldPosition.copy()
        
        self.CenterOfMassOffset = Value
        self.Collider.localPosition = -self.CenterOfMassOffset

        self.Collider.removeParent()
        self.Collider.setParent(self, True, False)

        self.worldPosition += (last-self.Collider.worldPosition)
        

    def PreUpdate(self):
        self.DeltaTime = Clamp(time.time() - self.LastTime, 0.0, 0.0333333)
        #PhyTime = time.time()

        self.速度 = self.worldLinearVelocity.magnitude
        self.速度公里每小时 = self.worldLinearVelocity.magnitude * 3.6
        #print("shudumeigongli: ", self.速度公里每小时)

        self.CurrentEngineTorque = self.向前移动 * self.EngineTorque
        self.CurrentBrakeTorque = self.Brake * self.BrakeTorque
        self.EngineTurnOver = MathLib.LerpF(
            self.EngineTurnOver, self.向前移动*self.MaxEngineTurnOver, self.DeltaTime*3)
        self.EngineAngularVelocity = (6.28318530718*self.EngineTurnOver)/60


        # TESTS ------------------------------------------------------
        if k[events.RKEY].values[-1]:
            self.applyForce([0, 0, -50000], False)
        if k[events.GKEY].values[-1]:
            self.applyTorque([0, 0, 10000], True)
        if k[events.HKEY].values[-1]:
            self.applyTorque([0, 0, -10000], True)
        if k[events.TKEY].values[-1]:
            self.applyForce([0, 0, 100000], False)
        if k[events.LEFTARROWKEY].values[-1]:
            self.applyForce([-30000, 0, 0], True)
        if k[events.RIGHTARROWKEY].values[-1]:
            self.applyForce([30000, 0, 0], True)


        # ------------------------------------------------------

        UpdatePhysics = False
        FixedTimeUpdate = 0.02

        CurrentFixedTime = time.time() - self.LastFixedUpdate
        if CurrentFixedTime > 0.06:
            FixedTimeUpdate = self.DeltaTime
            self.LastFixedUpdate = time.time()
            UpdatePhysics = True
        elif CurrentFixedTime >= 0.010:
            FixedTimeUpdate = (time.time() - self.LastFixedUpdate)
            self.LastFixedUpdate = time.time()
            UpdatePhysics = True

        i = 0
        for 对象_车轮 in self.Wheels:
            对象_车轮.悬挂.起始位置 = 对象_车轮.worldPosition + \
                (self.getAxisVect(Vector([0, 0, 1]))*对象_车轮.Wheel.半径)
            EndRayPoint = 对象_车轮.悬挂.起始位置 + \
                (self.getAxisVect(Vector([0, 0, -1]))*对象_车轮.悬挂.最大下沉)
            Ray = 对象_车轮.rayCast(EndRayPoint, 对象_车轮.悬挂.起始位置)

            if 对象_车轮.转向.双转向:
                对象_车轮.转向.更新转向角(
                    对象_车轮, self, self.向右移动, self.SteeringSensibility, self.DeltaTime)

            if Ray[0] != None:
                # Test
                车轮位置 = Ray[1]+对象_车轮.getAxisVect(Vector([0, 0, 0.1]))

                SuspensionFinalForce = 对象_车轮.悬挂.取悬挂力(
                    对象_车轮, Ray, self.DeltaTime)

                对象_车轮.Wheel.更新实时数据(
                    对象_车轮, self, Ray, self.DeltaTime)

                # Tire Forces
                FLong, FLateral = 对象_车轮.轮胎.取轮胎受力(
                    对象_车轮, self, self.DeltaTime)

                if UpdatePhysics:
                    # Apply Forces
                    #gametime = bge.logic.getTimeScale()
                    self.applyImpulse(Ray[1]+对象_车轮.getAxisVect(Vector(
                        [0, 0, 对象_车轮.悬挂.强制应用高度])), (SuspensionFinalForce+FLong)*FixedTimeUpdate, False)
                    self.applyImpulse(Ray[1]+对象_车轮.getAxisVect(
                        Vector([0, 0, 对象_车轮.悬挂.强制纵向高度])), (FLateral)*FixedTimeUpdate, False)

                # Set Wheel Rotation
                WheelRot = 对象_车轮.Wheel.LocalOrientation.to_euler()
                WheelRot[0] -= (对象_车轮.Wheel.车轮转动) * self.DeltaTime
                WheelRot[2] = 对象_车轮.localOrientation.to_euler()[2]
                对象_车轮.Wheel.LocalOrientation = WheelRot.to_matrix()

                #render.drawLine(WHEELPOS, WHEELPOS+(SuspensionFinalForce)*0.0005,[1,0.0,0,1])
                #render.drawLine(WHEELPOS, WHEELPOS+(FLateral)*0.0005,[1,0.5,0,1])
                #render.drawLine(WHEELPOS, WHEELPOS+(FLong)*0.0005,[1,0,1,1])
                #render.drawLine(对象_车轮.Suspension.StartPoint, Ray[1], [1,1.0,0,1])

                # Save last values
                对象_车轮.Wheel.最后的坐标 = Ray[1].copy()
            else:
                #render.drawLine(对象_车轮.Suspension.StartPoint, EndRayPoint, [1,1.0,0,1])

                对象_车轮.悬挂.当前长度 = 对象_车轮.悬挂.最大下沉               
                对象_车轮.Wheel.局部位置 = (EndRayPoint - self.worldPosition) @  self.worldOrientation + Vector([0, 0, 对象_车轮.Wheel.半径])
                对象_车轮.Wheel.LastPosition = 对象_车轮.worldPosition.copy()

            对象_车轮.悬挂.最终长度 = deepcopy(对象_车轮.悬挂.当前长度)
            对象_车轮.Wheel.最后的局部速度 = 对象_车轮.Wheel.局部轮速.copy()
            对象_车轮.Wheel.最后的整体速度 = 对象_车轮.Wheel.整体轮速.copy()
            对象_车轮.Wheel.最后的轴向量X = 对象_车轮.getAxisVect(
                Vector([1, 0, 0])).copy()

            # Test
            对象_车轮['wheelVisual'].localPosition = 对象_车轮.Wheel.局部位置
            对象_车轮['wheelVisual'].localOrientation = 对象_车轮.Wheel.LocalOrientation

            i += 1
        # if self.EnableDebug:
        #    self.DebugPhysics()

        self.LastVelocity = self.localLinearVelocity.copy()
        self.LastTime = time.time()
        #self.PhysicsTime = time.time() - PhyTime
