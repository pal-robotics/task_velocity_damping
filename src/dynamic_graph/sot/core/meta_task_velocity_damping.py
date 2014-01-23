from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.task_velocity_damping import TaskVelocityDamping

from dynamic_graph import plug
from dynamic_graph.sot.core import *

from matrix_util import matrixToTuple

class MetaTaskVelocityDamping(object):
    name=''
    dyn=0
    task=0       
    collisionCenter = None
    _ds = 0
    _di = 0
    controlGain = 0
    
    def createTask(self):
        self.task = TaskVelocityDamping(self.name)
        
    def createOpPoint(self,opPoint,opPointRef = 'right-wrist'):
        self.opPoint=opPoint
        if self.opPointExist(opPoint): 
            print 'no op point created in velocity damping'
            return
        self.dyn.createOpPoint(opPoint,opPointRef)
        
    def opPointExist(self,opPoint):
        sigsP = filter(lambda x: x.getName().split(':')[-1] == opPoint,
                       self.dyn.signals())
        sigsJ = filter(lambda x: x.getName().split(':')[-1] == 'J'+opPoint,
                       self.dyn.signals())
        return len(sigsP)==1 & len(sigsJ)==1

    def plugEverything(self):
        self.dyn.signal(self.opPoint).recompute(0)
        self.dyn.signal('J'+self.opPoint).recompute(0)
        plug(self.dyn.signal(self.opPoint),self.task.p1)
        plug(self.dyn.signal('J'+self.opPoint),self.task.jVel)
        
        if (self.collisionCenter != None):
        	self.task.p2.value = matrixToTuple(self.collisionCenter)
        self.task.di.value = self._di
        self.task.ds.value = self._ds
        self.task.controlGain.value = self.controlGain
        self.task.dt.value = 0.001
        
    def __init__(self,name,dynamic, opPoint, opPointRef, collisionCenter, di, ds, controlGain=1):
        self.name='taskVelocity'+str(name)
        self.dyn = dynamic
        self.collisionCenter = collisionCenter
        self._ds = ds
        self._di = di
        self.controlGain = controlGain
        self.createTask()
        self.createOpPoint(opPoint,opPointRef)
        self.plugEverything()
        
