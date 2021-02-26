************
Introduction
************

The official documentation is available as
`PDF <https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstartguide.pdf>`_
or `Google Doc <https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3>`_.

PyBullet is a fast and easy to use Python module for robotics simulation and
machine learning, with a focus on sim-to-real transfer. With PyBullet you can
load articulated bodies from URDF, SDF, MJCF and other file formats. PyBullet
provides forward dynamics simulation, inverse dynamics computation, forward
and inverse kinematics, collision detection and ray intersection queries. The
`Bullet Physics SDK <https://github.com/bulletphysics/bullet3>`_ includes
PyBullet robotic examples such as a simulated Minitaur quadruped, humanoids
running using TensorFlow inference and KUKA arms grasping objects. Reduced
coordinate multibodies, rigidbodies and deformables are handled by a unified
LCP constraint solver, similar to this
`thesis <https://drive.google.com/file/d/0Bz3vEa19XOYGNWdZWGpMdUdqVmZ5ZVBOaEh4ZnpNaUxxZFNV/view>`_.

Aside from physics simulation, there are bindings to rendering, with a CPU
renderer (TinyRenderer) and OpenGL 3.x rendering and visualization and support
for Virtual Reality headsets such as HTC Vive and Oculus Rift. PyBullet also
has functionality to perform collision detection queries (closest points,
overlapping pairs, ray intersection test etc) and to add debug rendering (debug
lines and text). PyBullet has cross-platform built-in client-server support for
shared memory, UDP and TCP networking. So you can run PyBullet on Linux
connecting to a Windows VR server.

PyBullet wraps the new
`Bullet C-API <https://github.com/bulletphysics/bullet3/blob/master/examples/SharedMemory/PhysicsClientC_API.h>`_,
which is designed to be independent from the underlying physics engine and
render engine, so we can easily migrate to newer versions of Bullet, or
use a different physics engine or render engine. By default, PyBullet uses
the Bullet 2.x API on the CPU. We will expose Bullet 3.x running on GPU
using OpenCL as well. There is also a C++ API similar to PyBullet, see
`b3RobotSimulatorClientAPI <https://github.com/bulletphysics/bullet3/blob/master/examples/RobotSimulator/b3RobotSimulatorClientAPI.h>`_.

PyBullet can be easily used with TensorFlow and OpenAI Gym. Researchers from
Google Brain [`1 <https://arxiv.org/abs/1804.10332>`_,
`2 <https://sites.google.com/view/graspgan>`_, `3 <https://xcyan.github.io/geoaware_grasping/>`_ ,
`4 <https://arxiv.org/abs/1712.07642>`_],
`X <https://sites.google.com/view/multi-task-domain-adaptation>`_
[`5 <https://sim2real.github.io/assets/slides/bai-Learning_to_Grasp_Using_Simulation_Yunfei_Bai_Google_X.pdf>`_,
`6 <https://www.linkedin.com/pulse/speeding-up-robot-learning-100x-simulation-mrinal-kalakrishnan/>`_],
Stanford AI Lab [`7 <https://stanfordvl.github.io/ntp/?utm_content=buffer8b1fc>`_,
`8 <http://gibsonenv.stanford.edu/>`_], `OpenAI <https://openai.com/blog/roboschool/>`_,
INRIA [`9 <https://openlab-flowers.inria.fr/t/openai-gym-a-toolkit-for-comparing-reinforcement-learning-algorithms/184>`_]
and many other labs use PyBullet. If you use PyBullet in your research, please
add a citation (:ref:`citation`).


Hello PyBullet World
====================

Here is a PyBullet introduction script that we discuss step by step:

.. code-block:: python

    import pybullet as p
    import time
    import pybullet_data
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0, 0, -10)
    planeId = p.loadURDF("plane.urdf")
    startPos = [0,0,1]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    boxId = p.loadURDF("r2d2.urdf", startPos, startOrientation)
    #set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
    for i in range (10000):
        p.stepSimulation()
        time.sleep(1./240.)
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    print(cubePos, cubeOrn)
    p.disconnect()


Additional Data
===============

You can provide your own data files, or you can use the PyBullet_data package
that ships with PyBullet. For this, import pybullet_data and register the
directory using pybullet.setAdditionalSearchPath(pybullet_data.getDataPath()).


Visualizer
==========

Keyboard Control
----------------

.. https://github.com/bulletphysics/bullet3/blob/master/examples/ExampleBrowser/OpenGLExampleBrowser.cpp#L198

You can use the following keys to control the pybullet GUI.

+------+-----------------------------------------------------+
| Key  | Effect                                              |
+======+=====================================================+
| Ctrl | Activate camera rotation (keep key pressed)         |
+------+-----------------------------------------------------+
| Esc  | Quit pybullet                                       |
+------+-----------------------------------------------------+
| F1   | Toggle saving a series of screenshots (in working   |
|      | directory)                                          |
+------+-----------------------------------------------------+
| a    | Toggle axis-aligned bounding boxes (only wireframe) |
+------+-----------------------------------------------------+
| c    | Toggle contact points                               |
+------+-----------------------------------------------------+
| d    | ?                                                   |
+------+-----------------------------------------------------+
| w    | Toggle wireframe                                    |
+------+-----------------------------------------------------+
| s    | Toggle shadows                                      |
+------+-----------------------------------------------------+
| g    | Toggle debug view (grid and GUI)                    |
+------+-----------------------------------------------------+
| v    | Toggle render visual geometry                       |
+------+-----------------------------------------------------+
| k    | Toggle constraints                                  |
+------+-----------------------------------------------------+
| l    | Toggle constraint limits                            |
+------+-----------------------------------------------------+
| j    | Toggle draw frames                                  |
+------+-----------------------------------------------------+
| p    | Write timing information (/tmp/timings_*.json on    |
|      | linux)                                              |
+------+-----------------------------------------------------+


Example Scripts
===============

Example scripts (could be out-of-date, check actual Bullet/examples/pybullet/examples folder.)


* examples/pybullet/tensorflow/humanoid_running.py:
  load a humanoid and use a trained neural network to control the running using
  TensorFlow, trained by OpenAI
* examples/pybullet/gym/pybullet_envs/bullet/minitaur.py and minitaur_gym_env.py:
  Minitaur environment for OpenAI GYM and TensorFlow.
  You can also use python -m pybullet_envs.examples.minitaur_gym_env_example
  after you did pip install pybullet to see the Minitaur in action.
* examples/pybullet/examples/quadruped.py:
  load a quadruped from URDF file, step the simulation, control the motors
  for a simple hopping gait based on sine waves.Will also log the state to
  file using p.startStateLogging.
  `See video <https://www.youtube.com/watch?v=lv7lybtOzeo>`_.
* examples/quadruped_playback.py:
  Create a quadruped (Minitaur), read log file and set positions as motor
  control targets.
* examples/pybullet/examples/testrender.py:
  load a URDF file and render an image, get the pixels (RGB, depth,
  segmentation mask) and display the image using MatPlotLib.
* examples/pybullet/examples/testrender_np.py:
  Similar to testrender.py, but speed up the pixel transfer using NumPy
  arrays. Also includes simple benchmark/timings.
* examples/pybullet/examples/saveWorld.py:
  Save the state (position, orientation) of objects into a pybullet Python
  scripts. This is mainly useful to setup a scene in VR and save the initial
  state. Not all state is serialized.
* examples/pybullet/examples/inverse_kinematics.py:
  Show how to use the calculateInverseKinematics command, creating a Kuka
  ARM clock
* examples/pybullet/examples/rollPitchYaw.py:
  Show how to use slider GUI widgets
* examples/pybullet/examples/constraint.py:
  Programmatically create a constraint between links.
* examples/pybullet/examples/vrhand.py:
  Control a hand using a VR glove, tracked by a VR controller.
  See `video <https://www.youtube.com/watch?v=0JC-yukK-jo>`_.
