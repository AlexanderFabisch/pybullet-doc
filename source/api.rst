.. _api:

=================
API Documentation
=================

.. contents:: :local:
    :depth: 2


:mod:`pybullet`
===============

.. automodule:: pybullet_api
    :no-members:
    :no-inherited-members:

Connect, disconnect, bullet_client
----------------------------------

.. autosummary::
   :toctree: _apidoc/

   ~pybullet_api.connect
   ~pybullet_api.getConnectionInfo
   ~pybullet_api.isConnected
   ~pybullet_api.setTimeOut
   ~pybullet_api.disconnect


Control Physics Engine
----------------------

.. autosummary::
   :toctree: _apidoc/

   ~pybullet_api.resetSimulation
   ~pybullet_api.setPhysicsEngineParameter
   ~pybullet_api.getPhysicsEngineParameters
   ~pybullet_api.setTimeStep
   ~pybullet_api.setDefaultContactERP
   ~pybullet_api.setGravity
   ~pybullet_api.startStateLogging
   ~pybullet_api.stopStateLogging
   ~pybullet_api.submitProfileTiming
   ~pybullet_api.loadURDF
   ~pybullet_api.loadSDF
   ~pybullet_api.loadMJCF
   ~pybullet_api.saveState
   ~pybullet_api.saveBullet
   ~pybullet_api.restoreState
   ~pybullet_api.removeState
   ~pybullet_api.saveWorld
   ~pybullet_api.createCollisionShape
   ~pybullet_api.createCollisionShapeArray
   ~pybullet_api.removeCollisionShape
   ~pybullet_api.createVisualShape
   ~pybullet_api.createVisualShapeArray
   ~pybullet_api.createMultiBody
   ~pybullet_api.createVisualShape
   ~pybullet_api.getMeshData
   ~pybullet_api.stepSimulation
   ~pybullet_api.performCollisionDetection
   ~pybullet_api.setRealTimeSimulation
   ~pybullet_api.getBasePositionAndOrientation
   ~pybullet_api.resetBasePositionAndOrientation

Transforms: Position and Orientation
------------------------------------

.. autosummary::
   :toctree: _apidoc/

   ~pybullet_api.getQuaternionFromEuler
   ~pybullet_api.getEulerFromQuaternion
   ~pybullet_api.getMatrixFromQuaternion
   ~pybullet_api.getAxisAngleFromQuaternion
   ~pybullet_api.multiplyTransforms
   ~pybullet_api.invertTransform
   ~pybullet_api.getDifferenceQuaternion

Controlling a Robot
-------------------

.. autosummary::
   :toctree: _apidoc/

   ~pybullet_api.getNumJoints
   ~pybullet_api.getJointInfo
   ~pybullet_api.setJointMotorControl2
   ~pybullet_api.setJointMotorControlArray
   ~pybullet_api.setJointMotorControlMultiDof
   ~pybullet_api.setJointMotorControlMultiDofArray
   ~pybullet_api.getJointState
   ~pybullet_api.getJointStates
   ~pybullet_api.resetJointState
   ~pybullet_api.getJointStateMultiDof
   ~pybullet_api.getJointStatesMultiDof
   ~pybullet_api.resetJointStateMultiDof
   ~pybullet_api.resetJointStatesMultiDof
   ~pybullet_api.enableJointForceTorqueSensor
   ~pybullet_api.getLinkState
   ~pybullet_api.getLinkStates
   ~pybullet_api.getBaseVelocity
   ~pybullet_api.resetBaseVelocity
   ~pybullet_api.applyExternalForce
   ~pybullet_api.applyExternalTorque
   ~pybullet_api.getNumBodies
   ~pybullet_api.getBodyInfo
   ~pybullet_api.getBodyUniqueId
   ~pybullet_api.removeBody
   ~pybullet_api.syncBodyInfo
   ~pybullet_api.createConstraint
   ~pybullet_api.changeConstraint
   ~pybullet_api.removeConstraint
   ~pybullet_api.getNumConstraints
   ~pybullet_api.getConstraintUniqueId
   ~pybullet_api.getConstraintInfo
   ~pybullet_api.getConstraintState
   ~pybullet_api.getDynamicsInfo
   ~pybullet_api.changeDynamics

Deformables and Cloth (FEM, PBD)
--------------------------------

.. autosummary::
   :toctree: _apidoc/

   ~pybullet_api.loadSoftBody
   ~pybullet_api.loadURDF
   ~pybullet_api.createSoftBodyAnchor

Synthetic Camera Rendering
--------------------------

.. autosummary::
   :toctree: _apidoc/

   ~pybullet_api.computeViewMatrix
   ~pybullet_api.computeProjectionMatrix
   ~pybullet_api.computeViewMatrixFromYawPitchRoll
   ~pybullet_api.computeProjectionMatrixFOV
   ~pybullet_api.getCameraImage
   ~pybullet_api.isNumpyEnabled
   ~pybullet_api.getVisualShapeData
   ~pybullet_api.changeVisualShape
   ~pybullet_api.loadTexture

Collision Detection Queries
---------------------------

.. autosummary::
   :toctree: _apidoc/

   ~pybullet_api.getOverlappingObjects
   ~pybullet_api.getAABB
   ~pybullet_api.getContactPoints
   ~pybullet_api.getClosestPoints
   ~pybullet_api.rayTest
   ~pybullet_api.rayTestBatch
   ~pybullet_api.getCollisionShapeData
   ~pybullet_api.vhacd
   ~pybullet_api.setCollisionFilterGroupMask
   ~pybullet_api.setCollisionFilterPair

Inverse Dynamics, Kinematics
----------------------------

.. autosummary::
   :toctree: _apidoc/

   ~pybullet_api.calculateInverseDynamics
   ~pybullet_api.calculateJacobian
   ~pybullet_api.calculateMassMatrix
   ~pybullet_api.calculateInverseKinematics
   ~pybullet_api.calculateInverseKinematics2

Virtual Reality
---------------

.. autosummary::
   :toctree: _apidoc/

   ~pybullet_api.getVREvents
   ~pybullet_api.setVRCameraState

Debug GUI, Lines, Text, Parameters
----------------------------------

.. autosummary::
   :toctree: _apidoc/

   ~pybullet_api.addUserDebugLine
   ~pybullet_api.addUserDebugText
   ~pybullet_api.addUserDebugParameter
   ~pybullet_api.readUserDebugParameter
   ~pybullet_api.removeAllUserParameters
   ~pybullet_api.removeUserDebugItem
   ~pybullet_api.removeAllUserDebugItems
   ~pybullet_api.setDebugObjectColor
   ~pybullet_api.addUserData
   ~pybullet_api.getUserData
   ~pybullet_api.syncUserData
   ~pybullet_api.removeUserData
   ~pybullet_api.getUserDataId
   ~pybullet_api.getNumUserData
   ~pybullet_api.getUserDataInfo
   ~pybullet_api.configureDebugVisualizer
   ~pybullet_api.getDebugVisualizerCamera
   ~pybullet_api.resetDebugVisualizerCamera
   ~pybullet_api.getKeyboardEvents
   ~pybullet_api.getMouseEvents

Plugins
-------

.. autosummary::
   :toctree: _apidoc/

   ~pybullet_api.loadPlugin
   ~pybullet_api.executePluginCommand
   ~pybullet_api.unloadPlugin


:mod:`pybullet_data`
====================

.. automodule:: pybullet_data
    :no-members:
    :no-inherited-members:

.. autosummary::
   :toctree: _apidoc/

   ~pybullet_data.getDataPath


:mod:`pybullet_utils`
=====================

.. warning::

    Not fully documented yet!

.. automodule:: pybullet_utils
    :no-members:
    :no-inherited-members:

.. autosummary::
   :toctree: _apidoc/

   ~pybullet_utils.bullet_client.BulletClient
   ~pybullet_utils.urdfEditor.UrdfEditor
   ~pybullet_utils.pd_controller_stable.PDControllerStableMultiDof
   ~pybullet_utils.pd_controller_stable.PDControllerStable


:mod:`pybullet_envs`
=====================

.. warning::

    Not fully documented yet!

.. automodule:: pybullet_envs
    :no-members:
    :no-inherited-members:

.. autosummary::
   :toctree: _apidoc/

   pybullet_envs.getList
   pybullet_envs.register

.. autosummary::
   :toctree: _apidoc/

   pybullet_envs.env_bases.MJCFBaseBulletEnv
   pybullet_envs.env_bases.Camera
   pybullet_envs.gym_locomotion_envs.WalkerBaseBulletEnv
   pybullet_envs.gym_locomotion_envs.HopperBulletEnv
   pybullet_envs.gym_locomotion_envs.Walker2DBulletEnv
   pybullet_envs.gym_locomotion_envs.HalfCheetahBulletEnv
   pybullet_envs.gym_locomotion_envs.AntBulletEnv
   pybullet_envs.gym_locomotion_envs.HumanoidBulletEnv
   pybullet_envs.gym_locomotion_envs.HumanoidFlagrunBulletEnv
   pybullet_envs.gym_locomotion_envs.HumanoidFlagrunHarderBulletEnv
   pybullet_envs.gym_manipulator_envs.ReacherBulletEnv
   pybullet_envs.gym_manipulator_envs.PusherBulletEnv
   pybullet_envs.gym_manipulator_envs.StrikerBulletEnv
   pybullet_envs.gym_manipulator_envs.ThrowerBulletEnv
   pybullet_envs.robot_pendula.InvertedPendulum
   pybullet_envs.robot_pendula.InvertedPendulumSwingup
   pybullet_envs.robot_pendula.InvertedDoublePendulum
   pybullet_envs.robot_bases.XmlBasedRobot
   pybullet_envs.robot_bases.MJCFBasedRobot
   pybullet_envs.robot_bases.URDFBasedRobot
   pybullet_envs.robot_bases.SDFBasedRobot
   pybullet_envs.robot_bases.BodyPart
   pybullet_envs.robot_bases.Joint
