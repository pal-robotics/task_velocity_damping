from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.task_velocity_damping import TaskVelocityDamping

from dynamic_graph import plug
from dynamic_graph.sot.core import *

from matrix_util import matrixToTuple

class MetaTaskDynamicVelocityDamping(object):
    name=''
    dyn=0
    task=0   
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
        self.task.controlGain.value = self.controlGain
        self.task.dt.value = 0.001
        
    def __init__(self,name, controlGain=1):
        self.name='taskDynamicVelocity'+str(name)
        self.controlGain = controlGain
        self.createTask()
        self.plugEverything()
        
