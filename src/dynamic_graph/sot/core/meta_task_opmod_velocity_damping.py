from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.task_velocity_damping import TaskVelocityDamping
from dynamic_graph.sot.core.dyn_oppoint_modifier import DynamicOppointModifier
from dynamic_graph.sot.core.meta_task_velocity_damping import MetaTaskVelocityDamping

from dynamic_graph import plug
from dynamic_graph.sot.core import *

from matrix_util import matrixToTuple
import numpy

class MetaTaskOpModVelocityDamping(MetaTaskVelocityDamping):

    def createOpPointModif(self):
	self.opPointModif = DynamicOppointModifier('dynopmodif'+self.name)
	offset=numpy.eye(4)
	offset[0:3,3] = (0.0,0.0,0.0)
	self.opPointModif.setTransformation(matrixToTuple(offset))
	plug(self.dyn.signal(self.opPoint),self.opPointModif.signal('positionIN'))
	plug(self.dyn.signal('J'+self.opPoint),self.opPointModif.signal('jacobianIN'))
	self.opPointModif.activ = True

    def plugEverything(self):
        plug(self.opPointModif.signal('position'),self.task.p1 )
        plug(self.opPointModif.signal('jacobian'),self.task.jVel)
        self.task.di.value = self._di
        self.task.ds.value = self._ds
	print type(self.task)
        #self.task.controlGain.value = self.controlGain
        self.task.dt.value = 0.001
        
#p1 is the moving point, p2 considered to be static
    def __init__(self,name,dynamic, opPoint, opPointRef,di, ds, controlGain=1, collisionCenter= None):
        self.name='taskVelocity'+str(name)
        self.dyn = dynamic
        self.collisionCenter = collisionCenter
        self._ds = ds
        self._di = di
        self.controlGain = controlGain
        self.createTask()
        self.createOpPoint(opPoint,opPointRef)
	self.createOpPointModif()
        self.plugEverything()
    
    @property
    def opmodif(self):
        if not self.opPointModif.activ: 
            return False
        else: 
            return self.opPointModif.getTransformation()

    @opmodif.setter
    def opmodif(self,m):
        # print 'opmodif m value = ', m
        # opmodif m equals to input matrix
        if isinstance(m,bool) and m==False:
            plug(self.dyn.signal(self.opPoint),self.task.p1)
            plug(self.dyn.signal('J'+self.opPoint),self.task.jVel)
            self.opPointModif.activ = False
        else:
            if not self.opPointModif.activ:
                plug(self.opPointModif.signal('position'),self.task.p1 )
                plug(self.opPointModif.signal('jacobian'),self.task.jVel)
            self.opPointModif.setTransformation(m)
            self.opPointModif.activ = True


__all__ = ["MetaTaskOpModVelocityDamping"]
