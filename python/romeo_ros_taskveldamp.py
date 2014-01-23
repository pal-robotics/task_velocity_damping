# ______________________________________________________________________________
# ******************************************************************************
# ______________________________________________________________________________
# ******************************************************************************

from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *

from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger,updateComDisplay
from dynamic_graph.sot.core.utils.attime import attime,ALWAYS,refset,sigset
from numpy import *
import numpy

from dynamic_graph.sot.core.meta_task_velocity_damping import MetaTaskVelocityDamping

from dynamic_graph.sot.core.utils.history import History

from dynamic_graph.sot.dyninv.robot_specific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor,specificitiesName,jointRankName

# --- HELPER FUNCTION ---
# -----------------------
# --- --- --- --- --- ---
import numpy
import math

def mat2rpy(m):
    rpy = numpy.zeros(3)
    rpy[0] = math.atan2(m[1,0],m[0,0])
    rpy[1] = math.atan2(-m[2,0],math.sqrt(m[2,1]**2+m[2,2]**2))
    rpy[2] = math.atan2(m[2,1],m[2,2])
    return rpy

def mat2quat(m):
    quat = numpy.zeros(4)
    quat[3] = 0.5 * math.sqrt(m[0,0]+m[1,1]+m[2,2]+1)
    quat[0] = 0.5 * numpy.sign(m[2,1]-m[1,2])*math.sqrt(m[0,0]-m[1,1]-m[2,2]+1)
    quat[1] = 0.5 * numpy.sign(m[0,2]-m[2,0])*math.sqrt(m[1,1]-m[2,2]-m[0,0]+1)
    quat[2] = 0.5 * numpy.sign(m[1,0]-m[0,1])*math.sqrt(m[2,2]-m[0,0]-m[1,1]+1)
    return quat

def quat2mat(q):
    w = q[3]
    x = q[0]
    y = q[1]
    z = q[2]
    m = numpy.matrix( [[ 2*(w**2+x**2)-1 , 2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),2*(w**2+y**2)-1,2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),2*(w**2+z**2)-1]])
    return m

def rotx(angle):
    m = numpy.matrix([[1,0,0],[0,numpy.cos(angle),-numpy.sin(angle)],[0,numpy.sin(angle),numpy.cos(angle)]])
    return m

def roty(angle):
    m = numpy.matrix([[numpy.cos(angle),0,numpy.sin(angle)],[0,1,0],[-numpy.sin(angle),0,numpy.cos(angle)]])
    return m

def rotz(angle):
    m = numpy.matrix([[numpy.cos(angle),-numpy.sin(angle),0],[numpy.sin(angle),numpy.cos(angle),0],[0,0,1]])
    return m

def goalDef(xyz = [0,0,0],quat=[0,0,0,1]):
    goal = numpy.matrix([[0 , 0, 0, xyz[0]], [0,  0, 0, xyz[1]], [0, 0, 0, xyz[2]], [0, 0, 0, 1]])
    goal_r = quat2mat(quat)
    goal[0:3,0:3] = goal_r
    return goal

class MetaTaskIneqKine6d(MetaTaskKine6d):
    def createTask(self):
        self.task = TaskInequality('inequalitytask'+self.name)
        
    def createFeatures(self):
        self.feature    = FeaturePoint6d('ineqfeature'+self.name)
        self.featureDes = FeaturePoint6d('ineqfeature'+self.name+'_ref')
        self.feature.selec.value = '111111'
        self.feature.frame('current')

def createInequalityTask(taskName, jointName, selectionMask='000111', positionVector=(0,0,0), referenceInf=(-100,-100,-100), referenceSup=(100,100,100)):
    print 'selection mask', selectionMask
    taskIneq = MetaTaskIneqKine6d(taskName, robot.dynamic, jointName, jointName)
    taskIneq.feature.frame('desired')
    gotoNd(taskIneq, positionVector, '111')
    taskIneq.feature.selec.value = selectionMask
#     taskIneq.task.add(taskIneq.feature.name)
    taskIneq.task.referenceSup.value = referenceSup
    taskIneq.task.referenceInf.value = referenceInf
    taskIneq.task.selec.value = selectionMask
    taskIneq.task.dt.value = 0.001
    taskIneq.task.controlGain.value = 0.9
    return taskIneq

def createVelocityDampingTask(taskName, jointName, collisionCenter, di, ds):
    taskVelDamp = MetaTaskVelocityDamping(taskName, robot.dynamic, jointName, jointName, collisionCenter, di, ds)
    return taskVelDamp


# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------

robotName = 'romeo'
#robot = RobotSimu('romeo')

from dynamic_graph.sot.romeo.romeo import *
robot = Robot(robotName, device=RobotSimu(robotName))
#robot.resize(robotDim)
robotDim=robotDimension[robotName]
dt=5e-3

from dynamic_graph.sot.dyninv.robot_specific import halfSittingConfig
x0=-0.00949035111398315034
y0=0
z0=0.64870185118253043
# halfSittingConfig[robotName] = (x0,y0,z0,0,0,0)+initialConfig[robotName][6:]
halfSittingConfig[robotName] = initialConfig[robotName]

q0=list(halfSittingConfig[robotName])
initialConfig[robotName]=tuple(q0)

#robot.set( initialConfig[robotName] )
robot.halfSitting = halfSittingConfig
addRobotViewer(robot.device,small=True,small_extra=24,verbose=True)

#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
@loopInThread
def inc():
    robot.device.increment(dt)
    attime.run(robot.device.control.time)
    updateComDisplay(robot,robot.dynamic.com)
    #history.record()

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

#-----------------------------------------------------------------------------
#---- DYN --------------------------------------------------------------------
#-----------------------------------------------------------------------------
modelDir  = pkgDataRootDir[robotName]
xmlDir    = pkgDataRootDir[robotName]
specificitiesPath = xmlDir + '/' + specificitiesName[robotName]
jointRankPath     = xmlDir + '/' + jointRankName[robotName]
"""
dyn = Dynamic("dyn")
dyn.setFiles(modelDir, modelName[robotName],specificitiesPath,jointRankPath)
dyn.parse()

dyn.inertiaRotor.value = inertiaRotor[robotName]
dyn.gearRatio.value    = gearRatio[robotName]
"""
plug(robot.device.state,robot.dynamic.position)
robot.dynamic.velocity.value = robotDim*(0.,)
robot.dynamic.acceleration.value = robotDim*(0.,)

robot.dynamic.ffposition.unplug()
robot.dynamic.ffvelocity.unplug()
robot.dynamic.ffacceleration.unplug()

robot.dynamic.setProperty('ComputeBackwardDynamics','true')
robot.dynamic.setProperty('ComputeAccelerationCoM','true')

robot.device.control.unplug()

# Binds with ROS. assert that roscore is running.
# if you prefer a ROS free installation, please comment those lines.
from dynamic_graph.ros import *
ros = Ros(robot)


# ---- SOT ---------------------------------------------------------------------
# ---- SOT ---------------------------------------------------------------------
# ---- SOT ---------------------------------------------------------------------
# The solver SOTH of dyninv is used, but normally, the SOT solver should be sufficient
from dynamic_graph.sot.dyninv import SolverKine
def toList(sot):
    return map(lambda x: x[1:-1],sot.dispStack().split('|')[1:])
SolverKine.toList = toList


sot = SolverKine('sot')
sot.setSize(robotDim)
plug(sot.control,robot.device.control)

# ---- TASKS -------------------------------------------------------------------
# ---- TASKS -------------------------------------------------------------------
# ---- TASKS -------------------------------------------------------------------


# ---- TASK GRIP ---
taskRH=MetaTaskKine6d('rh',robot.dynamic,'rh','right-wrist')
taskRH.feature.frame('desired')

taskLH=MetaTaskKine6d('lh',robot.dynamic,'lh','left-wrist')
taskLH.feature.frame('desired')

# ---- INEQUALITY TEST ----
taskINEQ = createInequalityTask('rh', 'right-wrist', '001', (0,0,0), (0.0,), (0.2,))

# --- STATIC COM (if not walking)
taskCom = MetaTaskKineCom(robot.dynamic)


# --- TASK SUPPORT SMALL --------------------------------------------
featureSupportSmall = FeatureGeneric('featureSupportSmall')
plug(robot.dynamic.com,featureSupportSmall.errorIN)
plug(robot.dynamic.Jcom,featureSupportSmall.jacobianIN)

taskSupportSmall=TaskInequality('taskSupportSmall')
taskSupportSmall.add(featureSupportSmall.name)
taskSupportSmall.selec.value = '011'
#taskSupportSmall.referenceInf.value = (-0.02,-0.02,0)    # Xmin, Ymin
#askSupportSmall.referenceSup.value = (0.02,0.02,0)  # Xmax, Ymax
taskSupportSmall.referenceInf.value = (0,-0.05,0)    # Xmin, Ymin
taskSupportSmall.referenceSup.value = (0.02,0.05,0)  # Xmax, Ymax
taskSupportSmall.dt.value=dt


# --- Task Joint Limits -----------------------------------------
robot.dynamic.upperJl.recompute(0)
robot.dynamic.lowerJl.recompute(0)
taskJL = TaskJointLimits('taskJL')
plug(robot.dynamic.position,taskJL.position)
taskJL.controlGain.value = 10
taskJL.referenceInf.value = robot.dynamic.lowerJl.value
taskJL.referenceSup.value = robot.dynamic.upperJl.value
taskJL.dt.value = dt
taskJL.selec.value = toFlags(range(6,22)+range(22,28)+range(29,35))


# --- CONTACTS
# define contactLF and contactRF
for name,joint in [ ['LF','left-ankle'], ['RF','right-ankle' ], ['waist', 'waist']  ]:
    contact = MetaTaskKine6d('contact'+name,robot.dynamic,name,joint)
    contact.feature.frame('desired')
    contact.gain.setConstant(10)
    locals()['contact'+name] = contact

# --- TRACER -----------------------------------------------------------------
from dynamic_graph.tracer import *
#tr = Tracer('tr')
#tr.open('/tmp/','','.dat')
#tr.start()
#robot.after.addSignal('tr.triger')

#tr.add('dyn.com','com')

#history = History(dyn,1)

# --- SHORTCUTS ----------------------------------------------------------------
qn = taskJL.normalizedPosition
@optionalparentheses
def pqn(details=True):
    ''' Display the normalized configuration vector. '''
    qn.recompute(robot.state.time)
    s = [ "{0:.1f}".format(v) for v in qn.value]
    if details:
        print("Rleg: "+" ".join(s[:6]))
        print("Lleg: "+" ".join(s[6:12]))
        print("Body: "+" ".join(s[12:16]))
        print("Rarm: "+" ".join(s[16:23]))
        print("Larm :"+" ".join(s[23:30]))
    else:
        print(" ".join(s[:30]))


def jlbound(t=None):
    '''Display the velocity bound induced by the JL as a double-column matrix.'''
    if t==None: t=robot.state.time
    taskJL.task.recompute(t)
    return matrix([ [float(x),float(y)] for x,y
                    in [ c.split(',') for c in taskJL.task.value[6:-3].split('),(') ] ])

def p6d(R,t):
    M=eye(4)
    M[0:3,0:3]=R
    M[0:3,3]=t
    return M

def push(task):
    if isinstance(task,str): taskName=task
    elif "task" in task.__dict__:  taskName=task.task.name
    else: taskName=task.name
    if taskName not in sot.toList():
        sot.push(taskName)
        if taskName!="taskposture" and "taskposture" in sot.toList():
            sot.down("taskposture")


def pop(task):
    if isinstance(task,str): taskName=task
    elif "task" in task.__dict__:  taskName=task.task.name
    else: taskName=task.name
    if taskName in sot.toList(): sot.rm(taskName)


# --- DISPLAY ------------------------------------------------------------------
# --- DISPLAY ------------------------------------------------------------------
# --- DISPLAY ------------------------------------------------------------------
RAD=pi/180
comproj = [0.1,-0.95,1.6]
#robot.viewer.updateElementConfig('footproj',[0.5,0.15,1.6+0.08,0,-pi/2,0 ])
robot.device.viewer.updateElementConfig('footproj',comproj+[0,-pi/2,0 ])
#robot.device.viewer.updateElementConfig('zmp2',[0,0,-10,0,0,0])


class BallPosition:
    def __init__(self,xyz=(0,-1.1,0.9)):
        self.ball = xyz
        self.prec = 0
        self.t = 0
        self.duration = 0
        self.f = 0
        self.xyz= self.ball
        
    def move(self,xyz,duration=50):
        self.prec = self.ball
        self.ball = xyz
        self.t = 0
        self.duration = duration
        self.changeTargets()

        if duration>0:
            self.f = lambda : self.moveDisplay()
            attime(ALWAYS,self.f)
        else:
            self.moveDisplay()

    def moveDisplay(self):
        tau = 1.0 if self.duration<=0 else float(self.t) / self.duration
        xyz = tau * array(self.ball) + (1-tau) * array(self.prec)
        robot.device.viewer.updateElementConfig('zmp',vectorToTuple(xyz)+(0,0,0))

        self.t += 1
        if self.t>self.duration and self.duration>0:
            attime.stop(self.f)
        self.xyz= xyz
        
    def changeTargets(self):
        gotoNd(taskRH,self.ball,'111',(4.9,0.9,0.01,0.9))

b = BallPosition()

# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------

robot.dynamic.com.recompute(0)
taskCom.featureDes.errorIN.value = robot.dynamic.com.value
taskCom.task.controlGain.value = 10

#ball = BallPosition((0,-1.1,0.9))
#ball.move((0.5,-0.2,1.0),0)
gain = (4.9,0.9,0.01,0.9)
gotoNd(taskRH,(0.0,-0.2,1.1),'111',gain)
next()
next()
next()
 
goalP2 = goalDef((0.3,-0.2, 1.1))
taskVelDamp = createVelocityDampingTask('velDamp', 'right-wrist', goalP2, 0.3, 0.2)
taskVelDamp.task.controlGain.value = 0.5
robot.device.viewer.updateElementConfig('zmp',(0.3,-0.2, 1.1,0,0,0))


push(taskJL)
sot.addContact(contactRF)
sot.addContact(contactLF)
#sot.addContact(contactwaist)
push(taskVelDamp)
#push(taskINEQ)
push(taskRH)
   
go()

@optionalparentheses
def i():
    xyz=ball.xyz
    xyz[0] += 0.1
    ball.move(vectorToTuple(xyz),30)
