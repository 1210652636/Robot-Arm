from openravepy import *
import numpy
env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('fp0.env.xml') # load a simple scene

robot0=env.GetRobots()[0]
robot1=env.GetRobots()[1]
robot2=env.GetRobots()[2]
robot3=env.GetRobots()[3]

Tgoal0 = []
Tgoal1 = []
Tgoal2 = []
Tgoal3 = []
Tgoal00 = numpy.array([[0,-1,0,0.43287],[-1,0,0,0.98805],[0,0,-1,0.2753],[0,0,0,1]])
Tgoal01 = numpy.array([[0,-1,0,0.50287],[-1,0,0,0.98805],[0,0,-1,0.1752],[0,0,0,1]])
Tgoal02 = numpy.array([[0,-1,0,0.37287],[-1,0,0,0.98805],[0,0,-1,0.1752],[0,0,0,1]])
Tgoal03 = numpy.array([[0,-1,0,0.56287],[-1,0,0,0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal04 = numpy.array([[0,-1,0,0.43287],[-1,0,0,0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal05 = numpy.array([[0,-1,0,0.30287],[-1,0,0,0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal0.append(Tgoal00),Tgoal0.append(Tgoal01),Tgoal0.append(Tgoal02),Tgoal0.append(Tgoal03),Tgoal0.append(Tgoal04),Tgoal0.append(Tgoal05)
Tgoal10 = numpy.array([[0,-1,0,-0.43287],[-1,0,0,0.98805],[0,0,-1,0.2753],[0,0,0,1]])
Tgoal11 = numpy.array([[0,-1,0,-0.37287],[-1,0,0,0.98805],[0,0,-1,0.1752],[0,0,0,1]])
Tgoal12 = numpy.array([[0,-1,0,-0.50287],[-1,0,0,0.98805],[0,0,-1,0.1752],[0,0,0,1]])
Tgoal13 = numpy.array([[0,-1,0,-0.30287],[-1,0,0,0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal14 = numpy.array([[0,-1,0,-0.43287],[-1,0,0,0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal15 = numpy.array([[0,-1,0,-0.56287],[-1,0,0,0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal1.append(Tgoal10),Tgoal1.append(Tgoal11),Tgoal1.append(Tgoal12),Tgoal1.append(Tgoal13),Tgoal1.append(Tgoal14),Tgoal1.append(Tgoal15)
Tgoal20 = numpy.array([[0,-1,0,0.43287],[-1,0,0,-0.98805],[0,0,-1,0.2753],[0,0,0,1]])
Tgoal21 = numpy.array([[0,-1,0,0.50287],[-1,0,0,-0.98805],[0,0,-1,0.1752],[0,0,0,1]])
Tgoal22 = numpy.array([[0,-1,0,0.37287],[-1,0,0,-0.98805],[0,0,-1,0.1752],[0,0,0,1]])
Tgoal23 = numpy.array([[0,-1,0,0.56287],[-1,0,0,-0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal24 = numpy.array([[0,-1,0,0.43287],[-1,0,0,-0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal25 = numpy.array([[0,-1,0,0.30287],[-1,0,0,-0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal2.append(Tgoal20),Tgoal2.append(Tgoal21),Tgoal2.append(Tgoal22),Tgoal2.append(Tgoal23),Tgoal2.append(Tgoal24),Tgoal2.append(Tgoal25)
Tgoal30 = numpy.array([[0,-1,0,-0.43287],[-1,0,0,-0.98805],[0,0,-1,0.2753],[0,0,0,1]])
Tgoal31 = numpy.array([[0,-1,0,-0.37287],[-1,0,0,-0.98805],[0,0,-1,0.1752],[0,0,0,1]])
Tgoal32 = numpy.array([[0,-1,0,-0.50287],[-1,0,0,-0.98805],[0,0,-1,0.1752],[0,0,0,1]])
Tgoal33 = numpy.array([[0,-1,0,-0.30287],[-1,0,0,-0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal34 = numpy.array([[0,-1,0,-0.43287],[-1,0,0,-0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal35 = numpy.array([[0,-1,0,-0.56287],[-1,0,0,-0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal3.append(Tgoal30),Tgoal3.append(Tgoal31),Tgoal3.append(Tgoal32),Tgoal3.append(Tgoal33),Tgoal3.append(Tgoal34),Tgoal3.append(Tgoal35)
mug0 = []
mug0.append('mug00'),mug0.append('mug01'),mug0.append('mug02'),mug0.append('mug03'),mug0.append('mug04'),mug0.append('mug05')
mug1 = []
mug1.append('mug10'),mug1.append('mug11'),mug1.append('mug12'),mug1.append('mug13'),mug1.append('mug14'),mug1.append('mug15')
mug2 = []
mug2.append('mug20'),mug2.append('mug21'),mug2.append('mug22'),mug2.append('mug23'),mug2.append('mug24'),mug2.append('mug25')
mug3 = []
mug3.append('mug30'),mug3.append('mug31'),mug3.append('mug32'),mug3.append('mug33'),mug3.append('mug34'),mug3.append('mug35')
#res = manipprob.MoveToHandPosition(matrices=[Tgoal],seedik=10) # call motion planner with goal joint angles
def StraightWallPosition(wallCenterX,wallCenterY,wallHeight,boxSize,robotIndex):
    Tarm0 = []
    Tarm1 = []
    Tarm2 = []
    Tarm3 = []
    Tarm00 = numpy.array([[0,-1,0, wallCenterX+1*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm01 = numpy.array([[0,-1,0, wallCenterX+5*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm02 = numpy.array([[0,-1,0, wallCenterX+1*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm03 = numpy.array([[0,-1,0, wallCenterX+5*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm04 = numpy.array([[0,-1,0, wallCenterX+1*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm05 = numpy.array([[0,-1,0, wallCenterX+5*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm0.append(Tarm00),Tarm0.append(Tarm01),Tarm0.append(Tarm02),Tarm0.append(Tarm03),Tarm0.append(Tarm04),Tarm0.append(Tarm05)
    Tarm10 = numpy.array([[0,-1,0, wallCenterX-1*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm11 = numpy.array([[0,-1,0, wallCenterX-5*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm12 = numpy.array([[0,-1,0, wallCenterX-1*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm13 = numpy.array([[0,-1,0, wallCenterX-5*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm14 = numpy.array([[0,-1,0, wallCenterX-1*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm15 = numpy.array([[0,-1,0, wallCenterX-5*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm1.append(Tarm10),Tarm1.append(Tarm11),Tarm1.append(Tarm12),Tarm1.append(Tarm13),Tarm1.append(Tarm14),Tarm1.append(Tarm15)
    Tarm20 = numpy.array([[0,-1,0, wallCenterX+3*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm21 = numpy.array([[0,-1,0, wallCenterX+7*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm22 = numpy.array([[0,-1,0, wallCenterX+3*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm23 = numpy.array([[0,-1,0, wallCenterX+7*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm24 = numpy.array([[0,-1,0, wallCenterX+3*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm25 = numpy.array([[0,-1,0, wallCenterX+7*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm2.append(Tarm20),Tarm2.append(Tarm21),Tarm2.append(Tarm22),Tarm2.append(Tarm23),Tarm2.append(Tarm24),Tarm2.append(Tarm25)
    Tarm30 = numpy.array([[0,-1,0, wallCenterX-3*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm31 = numpy.array([[0,-1,0, wallCenterX-7*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm32 = numpy.array([[0,-1,0, wallCenterX-3*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm33 = numpy.array([[0,-1,0, wallCenterX-7*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm34 = numpy.array([[0,-1,0, wallCenterX-3*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm35 = numpy.array([[0,-1,0, wallCenterX-7*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm3.append(Tarm30),Tarm3.append(Tarm31),Tarm3.append(Tarm32),Tarm3.append(Tarm33),Tarm3.append(Tarm34),Tarm3.append(Tarm35)
    if robotIndex == 0:
        return Tarm0
    elif robotIndex == 1:
        return Tarm1
    elif robotIndex == 2:
        return Tarm2
    else:
        return Tarm3

def CurveWallPosition(wallCenterX,wallCenterY,wallHeight,boxSize,robotIndex):
    Tarm0 = []
    Tarm1 = []
    Tarm2 = []
    Tarm3 = []
    Tarm00 = numpy.array([[0,-1,0, wallCenterX+1*boxSize],[-1,0,0,wallCenterY-0.5*boxSize],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm01 = numpy.array([[0,-1,0, wallCenterX+5*boxSize],[-1,0,0,wallCenterY-1.5*boxSize],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm02 = numpy.array([[0,-1,0, wallCenterX+1*boxSize],[-1,0,0,wallCenterY-0.5*boxSize],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm03 = numpy.array([[0,-1,0, wallCenterX+5*boxSize],[-1,0,0,wallCenterY-1.5*boxSize],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm04 = numpy.array([[0,-1,0, wallCenterX+1*boxSize],[-1,0,0,wallCenterY-0.5*boxSize],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm05 = numpy.array([[0,-1,0, wallCenterX+5*boxSize],[-1,0,0,wallCenterY-1.5*boxSize],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm0.append(Tarm00),Tarm0.append(Tarm01),Tarm0.append(Tarm02),Tarm0.append(Tarm03),Tarm0.append(Tarm04),Tarm0.append(Tarm05)
    Tarm10 = numpy.array([[0,-1,0, wallCenterX-1*boxSize],[-1,0,0,wallCenterY+0.5*boxSize],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm11 = numpy.array([[0,-1,0, wallCenterX-5*boxSize],[-1,0,0,wallCenterY+1.5*boxSize],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm12 = numpy.array([[0,-1,0, wallCenterX-1*boxSize],[-1,0,0,wallCenterY+0.5*boxSize],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm13 = numpy.array([[0,-1,0, wallCenterX-5*boxSize],[-1,0,0,wallCenterY+1.5*boxSize],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm14 = numpy.array([[0,-1,0, wallCenterX-1*boxSize],[-1,0,0,wallCenterY+0.5*boxSize],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm15 = numpy.array([[0,-1,0, wallCenterX-5*boxSize],[-1,0,0,wallCenterY+1.5*boxSize],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm1.append(Tarm10),Tarm1.append(Tarm11),Tarm1.append(Tarm12),Tarm1.append(Tarm13),Tarm1.append(Tarm14),Tarm1.append(Tarm15)
    Tarm20 = numpy.array([[0,-1,0, wallCenterX+3*boxSize],[-1,0,0,wallCenterY-1.5*boxSize],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm21 = numpy.array([[0,-1,0, wallCenterX+7*boxSize],[-1,0,0,wallCenterY-0.5*boxSize],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm22 = numpy.array([[0,-1,0, wallCenterX+3*boxSize],[-1,0,0,wallCenterY-1.5*boxSize],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm23 = numpy.array([[0,-1,0, wallCenterX+7*boxSize],[-1,0,0,wallCenterY-0.5*boxSize],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm24 = numpy.array([[0,-1,0, wallCenterX+3*boxSize],[-1,0,0,wallCenterY-1.5*boxSize],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm25 = numpy.array([[0,-1,0, wallCenterX+7*boxSize],[-1,0,0,wallCenterY-0.5*boxSize],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm2.append(Tarm20),Tarm2.append(Tarm21),Tarm2.append(Tarm22),Tarm2.append(Tarm23),Tarm2.append(Tarm24),Tarm2.append(Tarm25)
    Tarm30 = numpy.array([[0,-1,0, wallCenterX-3*boxSize],[-1,0,0,wallCenterY+1.5*boxSize],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm31 = numpy.array([[0,-1,0, wallCenterX-7*boxSize],[-1,0,0,wallCenterY+0.5*boxSize],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm32 = numpy.array([[0,-1,0, wallCenterX-3*boxSize],[-1,0,0,wallCenterY+1.5*boxSize],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm33 = numpy.array([[0,-1,0, wallCenterX-7*boxSize],[-1,0,0,wallCenterY+0.5*boxSize],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm34 = numpy.array([[0,-1,0, wallCenterX-3*boxSize],[-1,0,0,wallCenterY+1.5*boxSize],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm35 = numpy.array([[0,-1,0, wallCenterX-7*boxSize],[-1,0,0,wallCenterY+0.5*boxSize],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm3.append(Tarm30),Tarm3.append(Tarm31),Tarm3.append(Tarm32),Tarm3.append(Tarm33),Tarm3.append(Tarm34),Tarm3.append(Tarm35)
    if robotIndex == 0:
        return Tarm0
    elif robotIndex == 1:
        return Tarm1
    elif robotIndex == 2:
        return Tarm2
    else:
        return Tarm3

def TowerPosition(towerCenterX,towerCenterY,towerHeight,towerGap,boxSize,robotIndex):
    Tarm0 = []
    Tarm1 = []
    Tarm2 = []
    Tarm3 = []
    Tarm00 = numpy.array([[0,-1,0, towerCenterX+1*towerGap+1.5*boxSize],[-1,0,0,towerCenterY+1*towerGap+1.5*boxSize],[0,0,-1,towerHeight-10*boxSize],[0,0,0,1]])
    Tarm01 = numpy.array([[0,-1,0, towerCenterX],[-1,0,0,towerCenterY],[0,0,-1,towerHeight-10*boxSize],[0,0,0,1]])
    Tarm02 = numpy.array([[0,-1,0, towerCenterX+1*towerGap+  1*boxSize],[-1,0,0,towerCenterY+1*towerGap+  1*boxSize],[0,0,-1,towerHeight- 8*boxSize],[0,0,0,1]])
    Tarm0.append(Tarm00),Tarm0.append(Tarm01),Tarm0.append(Tarm02)
    Tarm10 = numpy.array([[0,-1,0, towerCenterX-1*towerGap-1.5*boxSize],[-1,0,0,towerCenterY+1*towerGap+1.5*boxSize],[0,0,-1,towerHeight-10*boxSize],[0,0,0,1]])
    Tarm11 = numpy.array([[0,-1,0, towerCenterX-1*towerGap-  1*boxSize],[-1,0,0,towerCenterY+1*towerGap+  1*boxSize],[0,0,-1,towerHeight- 8*boxSize],[0,0,0,1]])
    Tarm12 = numpy.array([[0,-1,0, towerCenterX],[-1,0,0,towerCenterY],[0,0,-1,towerHeight-6*boxSize],[0,0,0,1]])
    Tarm1.append(Tarm10),Tarm1.append(Tarm11),Tarm1.append(Tarm12)
    Tarm20 = numpy.array([[0,-1,0, towerCenterX+1*towerGap+1.5*boxSize],[-1,0,0,towerCenterX-1*towerGap-1.5*boxSize],[0,0,-1,towerHeight-10*boxSize],[0,0,0,1]])
    Tarm21 = numpy.array([[0,-1,0, towerCenterX+1*towerGap+  1*boxSize],[-1,0,0,towerCenterX-1*towerGap-  1*boxSize],[0,0,-1,towerHeight- 8*boxSize],[0,0,0,1]])
    Tarm22 = numpy.array([[0,-1,0, towerCenterX],[-1,0,0,towerCenterY],[0,0,-1,towerHeight-4*boxSize],[0,0,0,1]])
    Tarm2.append(Tarm20),Tarm2.append(Tarm21),Tarm2.append(Tarm22)
    Tarm30 = numpy.array([[0,-1,0, towerCenterX-1*towerGap-1.5*boxSize],[-1,0,0,towerCenterX-1*towerGap-1.5*boxSize],[0,0,-1,towerHeight-10*boxSize],[0,0,0,1]])
    Tarm31 = numpy.array([[0,-1,0, towerCenterX-1*towerGap-  1*boxSize],[-1,0,0,towerCenterX-1*towerGap-  1*boxSize],[0,0,-1,towerHeight- 8*boxSize],[0,0,0,1]])
    Tarm32 = numpy.array([[0,-1,0, towerCenterX],[-1,0,0,towerCenterY],[0,0,-1,towerHeight-2*boxSize],[0,0,0,1]])
    Tarm3.append(Tarm30),Tarm3.append(Tarm31),Tarm3.append(Tarm32)
    if robotIndex == 0:
        return Tarm0
    elif robotIndex == 1:
        return Tarm1
    elif robotIndex == 2:
        return Tarm2
    else:
        return Tarm3

def RobotMove(Robot,Tgoal,mug,Tarm,i):
    ikmodel = databases.inversekinematics.InverseKinematicsModel(Robot,iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    manipprob = interfaces.BaseManipulation(Robot)
    taskprob = interfaces.TaskManipulation(Robot)
    manip = Robot.GetActiveManipulator()
    manipprob.MoveToHandPosition(matrices=[Tgoal[i]],seedik=10)
    Robot.WaitForController(0)
    taskprob.CloseFingers()
    Robot.WaitForController(0)
    with env:
        Robot.Grab(env.GetKinBody(mug[i]))
        manipprob.MoveManipulator(numpy.zeros(len(manip.GetArmIndices())),jitter=0.04)
        incollision0 = not env.CheckCollision(Robot) and not Robot.CheckSelfCollision()
        sol = manip.FindIKSolution(Tarm[i], IkFilterOptions.CheckEnvCollisions) # get collision-free solution
        print("incollision0:",incollision0)
        '''
        while incollision0 is False:
            print("I am thinking")
            incollision0 = not env.CheckCollision(Robot) and not Robot.CheckSelfCollision()
            sol = manip.FindIKSolution(Tarm[i], IkFilterOptions.CheckEnvCollisions) # get collision-free solution
        '''
        manipprob.MoveManipulator(sol,jitter=0.04)
    Robot.WaitForController(0) # wait
    print("ReleaseFingers")
    taskprob.ReleaseFingers(target = env.GetKinBody(mug[i]))
    Robot.WaitForController(0) # wait
    manipprob.MoveManipulator(numpy.zeros(len(manip.GetArmIndices())),jitter=0.04)

Taim0 = CurveWallPosition(0,0,0.2755,0.0501,0)
Taim1 = CurveWallPosition(0,0,0.2755,0.0501,1)
Taim2 = CurveWallPosition(0,0,0.2755,0.0501,2)
Taim3 = CurveWallPosition(0,0,0.2755,0.0501,3)

Toweraim0 = TowerPosition(0,0,0.5761,0.03995,0.0501,0)
Toweraim1 = TowerPosition(0,0,0.5761,0.03995,0.0501,1)
Toweraim2 = TowerPosition(0,0,0.5761,0.03995,0.0501,2)
Toweraim3 = TowerPosition(0,0,0.5761,0.03995,0.0501,3)

for i in range(6):
    RobotMove(robot0,Tgoal0,mug0,Taim0,i)
    RobotMove(robot1,Tgoal1,mug1,Taim1,i)
    RobotMove(robot2,Tgoal2,mug2,Taim2,i)
    RobotMove(robot3,Tgoal3,mug3,Taim3,i)


from openravepy import *
import numpy
env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('fp1.env.xml') # load a simple scene

robot0=env.GetRobots()[0]
robot1=env.GetRobots()[1]
robot2=env.GetRobots()[2]
robot3=env.GetRobots()[3]

Tgoal0 = []
Tgoal1 = []
Tgoal2 = []
Tgoal3 = []
Tgoal00 = numpy.array([[0,-1,0,0.63307],[-1,0,0,0.98805],[0,0,-1,0.2753],[0,0,0,1]])
Tgoal01 = numpy.array([[0,-1,0,0.70307],[-1,0,0,0.98805],[0,0,-1,0.1752],[0,0,0,1]])
Tgoal02 = numpy.array([[0,-1,0,0.57307],[-1,0,0,0.98805],[0,0,-1,0.1752],[0,0,0,1]])
Tgoal03 = numpy.array([[0,-1,0,0.76307],[-1,0,0,0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal04 = numpy.array([[0,-1,0,0.63307],[-1,0,0,0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal05 = numpy.array([[0,-1,0,0.50307],[-1,0,0,0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal0.append(Tgoal00),Tgoal0.append(Tgoal01),Tgoal0.append(Tgoal02),Tgoal0.append(Tgoal03),Tgoal0.append(Tgoal04),Tgoal0.append(Tgoal05)
Tgoal10 = numpy.array([[0,-1,0,0.273],[-1,0,0,0.98805],[0,0,-1,0.2753],[0,0,0,1]])
Tgoal11 = numpy.array([[0,-1,0,0.333],[-1,0,0,0.98805],[0,0,-1,0.1752],[0,0,0,1]])
Tgoal12 = numpy.array([[0,-1,0,0.203],[-1,0,0,0.98805],[0,0,-1,0.1752],[0,0,0,1]])
Tgoal13 = numpy.array([[0,-1,0,0.403],[-1,0,0,0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal14 = numpy.array([[0,-1,0,0.273],[-1,0,0,0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal15 = numpy.array([[0,-1,0,0.143],[-1,0,0,0.98805],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal1.append(Tgoal10),Tgoal1.append(Tgoal11),Tgoal1.append(Tgoal12),Tgoal1.append(Tgoal13),Tgoal1.append(Tgoal14),Tgoal1.append(Tgoal15)
Tgoal20 = numpy.array([[0,-1,0,0.63307],[-1,0,0,0.-0.88795],[0,0,-1,0.2753],[0,0,0,1]])
Tgoal21 = numpy.array([[0,-1,0,0.70307],[-1,0,0,0.-0.88795],[0,0,-1,0.1752],[0,0,0,1]])
Tgoal22 = numpy.array([[0,-1,0,0.57307],[-1,0,0,0.-0.88795],[0,0,-1,0.1752],[0,0,0,1]])
Tgoal23 = numpy.array([[0,-1,0,0.76307],[-1,0,0,0.-0.88795],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal24 = numpy.array([[0,-1,0,0.63307],[-1,0,0,0.-0.88795],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal25 = numpy.array([[0,-1,0,0.50307],[-1,0,0,0.-0.88795],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal2.append(Tgoal20),Tgoal2.append(Tgoal21),Tgoal2.append(Tgoal22),Tgoal2.append(Tgoal23),Tgoal2.append(Tgoal24),Tgoal2.append(Tgoal25)
Tgoal30 = numpy.array([[0,-1,0,0.273],[-1,0,0,0.-0.88795],[0,0,-1,0.2753],[0,0,0,1]])
Tgoal31 = numpy.array([[0,-1,0,0.333],[-1,0,0,0.-0.88795],[0,0,-1,0.1752],[0,0,0,1]])
Tgoal32 = numpy.array([[0,-1,0,0.203],[-1,0,0,0.-0.88795],[0,0,-1,0.1752],[0,0,0,1]])
Tgoal33 = numpy.array([[0,-1,0,0.403],[-1,0,0,0.-0.88795],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal34 = numpy.array([[0,-1,0,0.273],[-1,0,0,0.-0.88795],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal35 = numpy.array([[0,-1,0,0.143],[-1,0,0,0.-0.88795],[0,0,-1,0.0751],[0,0,0,1]])
Tgoal3.append(Tgoal30),Tgoal3.append(Tgoal31),Tgoal3.append(Tgoal32),Tgoal3.append(Tgoal33),Tgoal3.append(Tgoal34),Tgoal3.append(Tgoal35)
#x
#0.1498,0.2500,0.3502,0.4504,0.5506,0.6508,0.7510,0.8512
mug0 = []
mug0.append('mug00'),mug0.append('mug01'),mug0.append('mug02'),mug0.append('mug03'),mug0.append('mug04'),mug0.append('mug05')
mug1 = []
mug1.append('mug10'),mug1.append('mug11'),mug1.append('mug12'),mug1.append('mug13'),mug1.append('mug14'),mug1.append('mug15')
mug2 = []
mug2.append('mug20'),mug2.append('mug21'),mug2.append('mug22'),mug2.append('mug23'),mug2.append('mug24'),mug2.append('mug25')
mug3 = []
mug3.append('mug30'),mug3.append('mug31'),mug3.append('mug32'),mug3.append('mug33'),mug3.append('mug34'),mug3.append('mug35')
#res = manipprob.MoveToHandPosition(matrices=[Tgoal],seedik=10) # call motion planner with goal joint angles
def StraightWallPosition(wallCenterX,wallCenterY,wallHeight,boxSize,robotIndex):
    #x
    #0.1498,0.2500,0.3502,0.4504,0.5506,0.6508,0.7510,0.8512
    Tarm0 = []
    Tarm1 = []
    Tarm2 = []
    Tarm3 = []
    Tarm00 = numpy.array([[0,-1,0, wallCenterX+7*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm01 = numpy.array([[0,-1,0, wallCenterX-1*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm02 = numpy.array([[0,-1,0, wallCenterX+7*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm03 = numpy.array([[0,-1,0, wallCenterX-1*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm04 = numpy.array([[0,-1,0, wallCenterX+7*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm05 = numpy.array([[0,-1,0, wallCenterX-1*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm0.append(Tarm00),Tarm0.append(Tarm01),Tarm0.append(Tarm02),Tarm0.append(Tarm03),Tarm0.append(Tarm04),Tarm0.append(Tarm05)
    Tarm10 = numpy.array([[0,-1,0, wallCenterX+5*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm11 = numpy.array([[0,-1,0, wallCenterX-3*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm12 = numpy.array([[0,-1,0, wallCenterX+5*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm13 = numpy.array([[0,-1,0, wallCenterX-3*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm14 = numpy.array([[0,-1,0, wallCenterX+5*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm15 = numpy.array([[0,-1,0, wallCenterX-3*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm1.append(Tarm10),Tarm1.append(Tarm11),Tarm1.append(Tarm12),Tarm1.append(Tarm13),Tarm1.append(Tarm14),Tarm1.append(Tarm15)
    Tarm20 = numpy.array([[0,-1,0, wallCenterX+3*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm21 = numpy.array([[0,-1,0, wallCenterX-5*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm22 = numpy.array([[0,-1,0, wallCenterX+3*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm23 = numpy.array([[0,-1,0, wallCenterX-5*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm24 = numpy.array([[0,-1,0, wallCenterX+3*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm25 = numpy.array([[0,-1,0, wallCenterX-5*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm2.append(Tarm20),Tarm2.append(Tarm21),Tarm2.append(Tarm22),Tarm2.append(Tarm23),Tarm2.append(Tarm24),Tarm2.append(Tarm25)
    Tarm30 = numpy.array([[0,-1,0, wallCenterX+1*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm31 = numpy.array([[0,-1,0, wallCenterX-7*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm32 = numpy.array([[0,-1,0, wallCenterX+1*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm33 = numpy.array([[0,-1,0, wallCenterX-7*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm34 = numpy.array([[0,-1,0, wallCenterX+1*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm35 = numpy.array([[0,-1,0, wallCenterX-7*boxSize],[-1,0,0,wallCenterY],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm3.append(Tarm30),Tarm3.append(Tarm31),Tarm3.append(Tarm32),Tarm3.append(Tarm33),Tarm3.append(Tarm34),Tarm3.append(Tarm35)
    if robotIndex == 0:
        return Tarm0
    elif robotIndex == 1:
        return Tarm1
    elif robotIndex == 2:
        return Tarm2
    else:
        return Tarm3

def CurveWallPosition(wallCenterX,wallCenterY,wallHeight,boxSize,robotIndex):
    Tarm0 = []
    Tarm1 = []
    Tarm2 = []
    Tarm3 = []
    Tarm00 = numpy.array([[0,-1,0, wallCenterX+1*boxSize],[-1,0,0,wallCenterY-0.5*boxSize],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm01 = numpy.array([[0,-1,0, wallCenterX+5*boxSize],[-1,0,0,wallCenterY-1.5*boxSize],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm02 = numpy.array([[0,-1,0, wallCenterX+1*boxSize],[-1,0,0,wallCenterY-0.5*boxSize],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm03 = numpy.array([[0,-1,0, wallCenterX+5*boxSize],[-1,0,0,wallCenterY-1.5*boxSize],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm04 = numpy.array([[0,-1,0, wallCenterX+1*boxSize],[-1,0,0,wallCenterY-0.5*boxSize],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm05 = numpy.array([[0,-1,0, wallCenterX+5*boxSize],[-1,0,0,wallCenterY-1.5*boxSize],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm0.append(Tarm00),Tarm0.append(Tarm01),Tarm0.append(Tarm02),Tarm0.append(Tarm03),Tarm0.append(Tarm04),Tarm0.append(Tarm05)
    Tarm10 = numpy.array([[0,-1,0, wallCenterX-1*boxSize],[-1,0,0,wallCenterY+0.5*boxSize],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm11 = numpy.array([[0,-1,0, wallCenterX-5*boxSize],[-1,0,0,wallCenterY+1.5*boxSize],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm12 = numpy.array([[0,-1,0, wallCenterX-1*boxSize],[-1,0,0,wallCenterY+0.5*boxSize],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm13 = numpy.array([[0,-1,0, wallCenterX-5*boxSize],[-1,0,0,wallCenterY+1.5*boxSize],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm14 = numpy.array([[0,-1,0, wallCenterX-1*boxSize],[-1,0,0,wallCenterY+0.5*boxSize],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm15 = numpy.array([[0,-1,0, wallCenterX-5*boxSize],[-1,0,0,wallCenterY+1.5*boxSize],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm1.append(Tarm10),Tarm1.append(Tarm11),Tarm1.append(Tarm12),Tarm1.append(Tarm13),Tarm1.append(Tarm14),Tarm1.append(Tarm15)
    Tarm20 = numpy.array([[0,-1,0, wallCenterX+3*boxSize],[-1,0,0,wallCenterY-1.5*boxSize],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm21 = numpy.array([[0,-1,0, wallCenterX+7*boxSize],[-1,0,0,wallCenterY-0.5*boxSize],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm22 = numpy.array([[0,-1,0, wallCenterX+3*boxSize],[-1,0,0,wallCenterY-1.5*boxSize],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm23 = numpy.array([[0,-1,0, wallCenterX+7*boxSize],[-1,0,0,wallCenterY-0.5*boxSize],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm24 = numpy.array([[0,-1,0, wallCenterX+3*boxSize],[-1,0,0,wallCenterY-1.5*boxSize],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm25 = numpy.array([[0,-1,0, wallCenterX+7*boxSize],[-1,0,0,wallCenterY-0.5*boxSize],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm2.append(Tarm20),Tarm2.append(Tarm21),Tarm2.append(Tarm22),Tarm2.append(Tarm23),Tarm2.append(Tarm24),Tarm2.append(Tarm25)
    Tarm30 = numpy.array([[0,-1,0, wallCenterX-3*boxSize],[-1,0,0,wallCenterY+1.5*boxSize],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm31 = numpy.array([[0,-1,0, wallCenterX-7*boxSize],[-1,0,0,wallCenterY+0.5*boxSize],[0,0,-1,wallHeight-4*boxSize],[0,0,0,1]])
    Tarm32 = numpy.array([[0,-1,0, wallCenterX-3*boxSize],[-1,0,0,wallCenterY+1.5*boxSize],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm33 = numpy.array([[0,-1,0, wallCenterX-7*boxSize],[-1,0,0,wallCenterY+0.5*boxSize],[0,0,-1,wallHeight-2*boxSize],[0,0,0,1]])
    Tarm34 = numpy.array([[0,-1,0, wallCenterX-3*boxSize],[-1,0,0,wallCenterY+1.5*boxSize],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm35 = numpy.array([[0,-1,0, wallCenterX-7*boxSize],[-1,0,0,wallCenterY+0.5*boxSize],[0,0,-1,wallHeight-0*boxSize],[0,0,0,1]])
    Tarm3.append(Tarm30),Tarm3.append(Tarm31),Tarm3.append(Tarm32),Tarm3.append(Tarm33),Tarm3.append(Tarm34),Tarm3.append(Tarm35)
    if robotIndex == 0:
        return Tarm0
    elif robotIndex == 1:
        return Tarm1
    elif robotIndex == 2:
        return Tarm2
    else:
        return Tarm3

def TowerPosition(towerCenterX,towerCenterY,towerHeight,towerGap,boxSize,robotIndex):
    Tarm0 = []
    Tarm1 = []
    Tarm2 = []
    Tarm3 = []
    Tarm00 = numpy.array([[0,-1,0, towerCenterX+1*towerGap+1.5*boxSize],[-1,0,0,towerCenterY-1*towerGap-1.5*boxSize],[0,0,-1,towerHeight-10*boxSize],[0,0,0,1]])
    Tarm01 = numpy.array([[0,-1,0, towerCenterX+1*towerGap+  1*boxSize],[-1,0,0,towerCenterY+1*towerGap+  1*boxSize],[0,0,-1,towerHeight- 8*boxSize],[0,0,0,1]])
    Tarm02 = numpy.array([[0,-1,0, towerCenterX+0*towerGap+  1*boxSize],[-1,0,0,towerCenterY+0*towerGap+  1*boxSize],[0,0,-1,towerHeight- 6*boxSize],[0,0,0,1]])
    Tarm03 = numpy.array([[0,-1,0, towerCenterX+0*towerGap+  1*boxSize],[-1,0,0,towerCenterY+0*towerGap+  1*boxSize],[0,0,-1,towerHeight- 2*boxSize],[0,0,0,1]])
    Tarm0.append(Tarm00),Tarm0.append(Tarm01),Tarm0.append(Tarm02),Tarm0.append(Tarm03)
    Tarm10 = numpy.array([[0,-1,0, towerCenterX-1*towerGap-1.5*boxSize],[-1,0,0,towerCenterY-1*towerGap-1.5*boxSize],[0,0,-1,towerHeight-10*boxSize],[0,0,0,1]])
    Tarm11 = numpy.array([[0,-1,0, towerCenterX-1*towerGap-  1*boxSize],[-1,0,0,towerCenterY+1*towerGap+  1*boxSize],[0,0,-1,towerHeight- 8*boxSize],[0,0,0,1]])
    Tarm12 = numpy.array([[0,-1,0, towerCenterX+0*towerGap+  1*boxSize],[-1,0,0,towerCenterY+0*towerGap+  1*boxSize],[0,0,-1,towerHeight- 4*boxSize],[0,0,0,1]])
    Tarm13 = numpy.array([[0,-1,0, towerCenterX+0*towerGap+  1*boxSize],[-1,0,0,towerCenterY+0*towerGap+  1*boxSize],[0,0,-1,towerHeight- 0*boxSize],[0,0,0,1]])
    Tarm1.append(Tarm10),Tarm1.append(Tarm11),Tarm1.append(Tarm12),Tarm1.append(Tarm13)
    Tarm20 = numpy.array([[0,-1,0, towerCenterX+1*towerGap+1.5*boxSize],[-1,0,0,towerCenterY+1*towerGap+1.5*boxSize],[0,0,-1,towerHeight-10*boxSize],[0,0,0,1]])
    Tarm21 = numpy.array([[0,-1,0, towerCenterX+1*towerGap+  1*boxSize],[-1,0,0,towerCenterY-1*towerGap-  1*boxSize],[0,0,-1,towerHeight- 8*boxSize],[0,0,0,1]])
    Tarm22 = numpy.array([[0,-1,0, towerCenterX-0*towerGap-  1*boxSize],[-1,0,0,towerCenterY-0*towerGap-  1*boxSize],[0,0,-1,towerHeight- 6*boxSize],[0,0,0,1]])
    Tarm23 = numpy.array([[0,-1,0, towerCenterX-0*towerGap-  1*boxSize],[-1,0,0,towerCenterY-0*towerGap-  1*boxSize],[0,0,-1,towerHeight- 2*boxSize],[0,0,0,1]])
    Tarm2.append(Tarm20),Tarm2.append(Tarm21),Tarm2.append(Tarm22),Tarm2.append(Tarm23)
    Tarm30 = numpy.array([[0,-1,0, towerCenterX-1*towerGap-1.5*boxSize],[-1,0,0,towerCenterY+1*towerGap+1.5*boxSize],[0,0,-1,towerHeight-10*boxSize],[0,0,0,1]])
    Tarm31 = numpy.array([[0,-1,0, towerCenterX-1*towerGap-  1*boxSize],[-1,0,0,towerCenterY-1*towerGap-  1*boxSize],[0,0,-1,towerHeight- 8*boxSize],[0,0,0,1]])
    Tarm32 = numpy.array([[0,-1,0, towerCenterX-0*towerGap-  1*boxSize],[-1,0,0,towerCenterY-0*towerGap-  1*boxSize],[0,0,-1,towerHeight- 4*boxSize],[0,0,0,1]])
    Tarm33 = numpy.array([[0,-1,0, towerCenterX-0*towerGap-  1*boxSize],[-1,0,0,towerCenterY-0*towerGap-  1*boxSize],[0,0,-1,towerHeight- 0*boxSize],[0,0,0,1]])
    Tarm3.append(Tarm30),Tarm3.append(Tarm31),Tarm3.append(Tarm32),Tarm3.append(Tarm33)
    if robotIndex == 0:
        return Tarm0
    elif robotIndex == 1:
        return Tarm1
    elif robotIndex == 2:
        return Tarm2
    else:
        return Tarm3

def RobotMove(Robot,Tgoal,mug,Tarm,i):
    ikmodel = databases.inversekinematics.InverseKinematicsModel(Robot,iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    manipprob = interfaces.BaseManipulation(Robot)
    taskprob = interfaces.TaskManipulation(Robot)
    manip = Robot.GetActiveManipulator()
    manipprob.MoveToHandPosition(matrices=[Tgoal[i]],seedik=10)
    Robot.WaitForController(0)
    taskprob.CloseFingers()
    Robot.WaitForController(0)
    with env:
        Robot.Grab(env.GetKinBody(mug[i]))
        manipprob.MoveManipulator(numpy.zeros(len(manip.GetArmIndices())),jitter=0.04)
        incollision0 = not env.CheckCollision(Robot) and not Robot.CheckSelfCollision()
        sol = manip.FindIKSolution(Tarm[i], IkFilterOptions.CheckEnvCollisions) # get collision-free solution
        print("incollision0:",incollision0)
        '''
        while incollision0 is False:
            print("I am thinking")
            incollision0 = not env.CheckCollision(Robot) and not Robot.CheckSelfCollision()
            sol = manip.FindIKSolution(Tarm[i], IkFilterOptions.CheckEnvCollisions) # get collision-free solution
        '''
        manipprob.MoveManipulator(sol,jitter=0.04)
    Robot.WaitForController(0) # wait
    print("ReleaseFingers")
    taskprob.ReleaseFingers(target = env.GetKinBody(mug[i]))
    Robot.WaitForController(0) # wait
    manipprob.MoveManipulator(numpy.zeros(len(manip.GetArmIndices())),jitter=0.04)

Taim0 = CurveWallPosition(0.5005,0.05005,0.2755,0.0501,0)
Taim1 = CurveWallPosition(0.5005,0.05005,0.2755,0.0501,1)
Taim2 = CurveWallPosition(0.5005,0.05005,0.2755,0.0501,2)
Taim3 = CurveWallPosition(0.5005,0.05005,0.2755,0.0501,3)

Toweraim0 = TowerPosition(0.50045,0.050275,0.5761,0.03995,0.0501,0)
Toweraim1 = TowerPosition(0.50045,0.050275,0.5761,0.03995,0.0501,1)
Toweraim2 = TowerPosition(0.50045,0.050275,0.5761,0.03995,0.0501,2)
Toweraim3 = TowerPosition(0.50045,0.050275,0.5761,0.03995,0.0501,3)

for i in range(6):
    RobotMove(robot0,Tgoal0,mug0,Taim0,i)
    RobotMove(robot1,Tgoal1,mug1,Taim1,i)
    RobotMove(robot2,Tgoal2,mug2,Taim2,i)
    RobotMove(robot3,Tgoal3,mug3,Taim3,i)
