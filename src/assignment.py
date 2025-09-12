import os, math, numpy as np
import Sofa.Core, SofaRuntime

from scipy import signal
import math 
import numpy as np

from controller import SoftBodyController

###

root = Sofa.Core.Node()
path = os.path.dirname(os.path.abspath(__file__))+'/plot/'
SofaRuntime.importPlugin("Sofa.Component")

###

class FingerController(Sofa.Core.Controller):

	def __init__(self, *args, **kwargs):
		Sofa.Core.Controller.__init__(self,*args, **kwargs)

		self.dt = 0.1
		self.time = 0.0

		self.node = kwargs['node']
		self.effector = kwargs['effector']

		# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		## TODO Set PID constants
		## kp = ???  # Proportional gain
		## ki = ???   # Integral gain
		## kd = ???   # Derivative gain
		# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
		# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		## TODO Initialize the SoftBodyController with the PID constants
		## self.controller = SoftBodyController(kp, ki, kd)
		# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	def onAnimateBeginEvent(self, event):
		self.time = self.node.time.value
		print(f'Position Fingertip:\t[{self.effector[0][0]:.5f}, {self.effector[0][1]:.5f}, {self.effector[0][2]:.5f}]')

		# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		## TODO For each target, record closest position reached and a screenshot acommpanying the result
		target = np.array([-50.0, 0.0,  50.0])
		# target = np.array([-50.0, 0.0, -50.0])
		# target = np.array([ 50.0, 0.0, -50.0])
		# target = np.array([ 50.0, 0.0,  50.0])
		# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		## TODO Implement a controller for the pneumatic actuation
		## p1, p2, p3 = ???
		##
		## print(f"Pressure values:\t[{p1:.5f}, {p2:.5f}, {p3:.5f}]\n")
		##
		## self.node.finger.cavity1.SurfaceForceField.value[0] = p1
		## self.node.finger.cavity2.SurfaceForceField.value[0] = p2
		## self.node.finger.cavity3.SurfaceForceField.value[0] = p3
		# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


def createScene(rootNode):
	rootNode.name = 'rootNode'

	rootNode.dt = 0.1
	rootNode.gravity=[0, 0, 0]

	# Required plugins
	rootNode.addObject('RequiredPlugin', name='SofaPython3')
	rootNode.addObject('RequiredPlugin', name="Sofa.Component.Engine.Select")
	rootNode.addObject('RequiredPlugin', name="Sofa.Component.IO.Mesh")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.LinearSolver.Iterative")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.Mapping.Linear")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.Mass")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.ODESolver.Forward")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.Setting")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.SolidMechanics.FEM.HyperElastic")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.SolidMechanics.FEM.Elastic")	
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.SolidMechanics.Spring")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.StateContainer")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.Topology.Container.Dynamic")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.Visual")
	rootNode.addObject("RequiredPlugin", name="Sofa.GL.Component.Rendering3D")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.AnimationLoop")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.Constraint.Lagrangian.Solver")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.MechanicalLoad")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.LinearSolver.Direct")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.Constraint.Lagrangian.Correction")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.Constraint.Projective")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.ODESolver.Backward")
	rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Algorithm')
	rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Intersection')
	rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')
	rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Response.Contact')
	rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear')
	rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')
	rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Model') # Needed to use components [BilateralLagrangianConstraint]  
	rootNode.addObject('RequiredPlugin', name='SoftRobots') # Needed to use components [SurfacePressureConstraint]  

	# Visual
	rootNode.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels hideCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')
	rootNode.addObject('OglSceneFrame', style='Arrows', alignment='TopRight')

	# Solvers and Loop
	rootNode.addObject('FreeMotionAnimationLoop', computeBoundingBox="0")
	rootNode.addObject('GenericConstraintSolver', tolerance=1e-24, maxIterations=1000)


	# ************************************************************************************************************************************************
	## Finger
	finger = rootNode.addChild('finger')
	finger.addObject('EulerImplicitSolver', name="Solver", rayleighStiffness="0.0", rayleighMass="0.0")
	finger.addObject('SparseLDLSolver', name="LinearSolver")

	finger.addObject('MeshVTKLoader', name='loader', filename="mesh/finger.vtu", translation = [0, 0, 0], rotation=[0, 30, 0], scale3d=[1000, 1000, 1000])
	finger.addObject('MechanicalObject', name='dofs', template='Vec3d', src = '@loader')
	finger.addObject('TetrahedronSetTopologyContainer', name="topo", src ='@loader')
	finger.addObject('TetrahedronSetTopologyModifier' ,  name="Modifier")
	finger.addObject('TetrahedronSetGeometryAlgorithms',name="GeomAlgo")

	finger.addObject('BoxROI', name='boxROI',box="-10 -5 -10 10 5 10", drawBoxes = True)
	finger.addObject('FixedConstraint', indices = '@boxROI.indices')

	# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	## TODO Assign mass of the object
	## finger.addObject('UniformMass', totalMass="???", src = '@topo')
	# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	## TODO Small description of differences between Neo-Hookean model and Hooke's law
	## ...
	# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	finger.addObject('TetrahedronHyperelasticityFEMForceField', template='Vec3d', name='FEM', src ='@topo', ParameterSet='0.24203 0.0', materialName="StableNeoHookean")
	finger.addObject('LinearSolverConstraintCorrection')


	# ************************************************************************************************************************************************
	## Cavity
	cavity1 = finger.addChild('cavity1')
	cavity1.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity1.obj', translation = [0, 0, 0], rotation=[0, 0, 0], scale3d=[1000, 1000, 1000])
	cavity1.addObject('MeshTopology', src='@loader', name='topo')
	cavity1.addObject('MechanicalObject', name='dofs', template='Vec3d')
	cavity1.addObject('SurfacePressureConstraint', name='SurfaceForceField', template='Vec3d', value = 0.0, triangles='@topo.triangles', valueType='pressure')
	cavity1.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=False)

	cavity2 = finger.addChild('cavity2')
	cavity2.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity2.obj', translation = [0, 0, 0], rotation=[0, 0, 0], scale3d=[1000, 1000, 1000])
	cavity2.addObject('MeshTopology', src='@loader', name='topo')
	cavity2.addObject('MechanicalObject', name='dofs', template='Vec3d')
	cavity2.addObject('SurfacePressureConstraint', name='SurfaceForceField', template='Vec3d', value = 0.0, triangles='@topo.triangles', valueType='pressure')
	cavity2.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=False)

	cavity3 = finger.addChild('cavity3')
	cavity3.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity3.obj', translation = [0, 0, 0], rotation=[0, 0, 0], scale3d=[1000, 1000, 1000])
	cavity3.addObject('MeshTopology', src='@loader', name='topo')
	cavity3.addObject('MechanicalObject', name='dofs', template='Vec3d')
	cavity3.addObject('SurfacePressureConstraint', name='SurfaceForceField', template='Vec3d', value = 0.0, triangles='@topo.triangles', valueType='pressure')
	cavity3.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=False)


	# ************************************************************************************************************************************************
	## Controller

	effector = finger.addChild('effector')
	effector.addObject('MechanicalObject', name='dofs', template='Vec3',position=[0.0, 160.0, 0.0], showObject=True, showObjectScale=1, drawMode=2, showColor='blue', rotation = [0, 0, 0])
	effector.addObject('BarycentricMapping')

	finger.addObject(FingerController(node=rootNode, effector=rootNode.finger.effector.dofs.position))

	return rootNode
