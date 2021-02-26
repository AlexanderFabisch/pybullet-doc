***********************
Support, Tips, Citation
***********************

Question: Where do we go for support and to report issues?
----------------------------------------------------------

Answer: There is a `discussion forum <https://pybullet.org/Bullet/phpBB3/viewforum.php?f=24>`_
at http://pybullet.org/Bullet and an issue tracker
at https://github.com/bulletphysics/bullet3

.. _citation:

Question: How do we add a citation to PyBullet in our academic paper?
---------------------------------------------------------------------

Answer:

.. code-block:: bibtex

    @MISC{coumans2020,
        author = {Erwin Coumans and Yunfei Bai},
        title = {PyBullet, a Python module for physics simulation for games, robotics and machine learning},
        howpublished = {\url{http://pybullet.org}},
        year = {2016--2022}
    }

Question: Can PyBullet be used in Google Colab?
-----------------------------------------------

Answer: Yes, we provide precompiled manylinux wheels that can be used in Colab.
Also GPY rendering works using EGL. See an example Colab
`here <https://colab.research.google.com/drive/1u6j7JOqM05vUUjpVp5VNk0pd8q-vqGlx?authuser=1>`_.

Question: What happens to Bullet 2.x and the Bullet 3 OpenCL implementation?
----------------------------------------------------------------------------

Answer: PyBullet is wrapping the Bullet
`C-API <https://github.com/bulletphysics/bullet3/blob/master/examples/SharedMemory/PhysicsClientC_API.h>`_.
We will put the Bullet 3 OpenCL GPU API (and future Bullet 4.x API) behind this C-API. So if you use PyBullet or the C-API you are future-proof. Not to be confused with the Bullet 2.x C++ API.

Question: Should I use torque/force control or velocity/position control mode?
------------------------------------------------------------------------------

Answer: In general it is best to start with position or velocity control.
It will take much more effort to get force/torque control working reliably.

Question: The velocity of objects seems to be smaller than expected.
--------------------------------------------------------------------
Does PyBullet apply some default damping? Also the velocity doesn't exceed 100 units.

Answer: Yes, PyBullet applies some angular and linear damping to increase stability. You
can modify/disable this damping using the 'changeDynamics' command, using
linearDamping=0 and angularDamping=0 as arguments. 
The maximum linear/angular velocity is clamped to 100 units for stability.

Question: How to turn off gravity only for some parts of a robot (for example the arm)?
---------------------------------------------------------------------------------------

Answer: At the moment this is not exposed, so you would need to either turn of
gravity acceleration for all objects, and manually apply gravity for the
objects that need it. Or you can actively compute gravity compensation forces,
like happens on a real robot. Since Bullet has a full constraint system, it
would be trivial to compute those anti-gravity forces: You could run a second
simulation (PyBullet lets you connect to multiple physics servers) and
position the robot under gravity, set joint position control to keep the
position as desired, and gather those 'anti-gravity' forces. Then apply those
in the main simulation.

Question: How to scale up/down objects?
---------------------------------------

Answer: You can use the globalScaleFactor value as optional argument to loadURDF and
loadSDF. Otherwise scaling of visual shapes and collision shapes is part of most file formats, such as URDF and SDF. At the moment you cannot rescale objects.

Question: How can I get textures in my models?
----------------------------------------------
Answer:    You can use the Wavefront .obj file format. This will support material files (.mtl).
There are various examples using textures in the Bullet/data folder. You can change the texture for existing textured objects using the 'changeTexture' API.

Question: Which texture file formats are valid for PyBullet?
------------------------------------------------------------

Answer: Bullet uses stb_image to load texture files, which loads PNG, JPG,
TGA, GIF etc. see `stb_image.h <https://github.com/bulletphysics/bullet3/blob/master/examples/ThirdPartyLibs/stb_image/stb_image.h>`_
for details.

Question: How can I improve the performance and stability of the collision detection?
-------------------------------------------------------------------------------------

Answer: There are many ways to optimize, for example: shape type

1. Choose one or multiple primitive collision shape types such as box, sphere,
   capsule, cylinder to approximate an object, instead of using convex or
   concave triangle meshes.
2. If you really need to use triangle meshes, create a convex decomposition
   using Hierarchical Approximate Convex Decomposition (v-HACD). The
   `test_hacd <https://github.com/bulletphysics/bullet3/blob/master/Extras/VHACD/test/src/premake4.lua>`_
   utility converts convex triangle mesh in an OBJ file into a new OBJ file
   with multiple convex hull objects. See for example
   `Bullet/data/teddy_vhacd.urdf <https://github.com/bulletphysics/bullet3/blob/master/data/teddy_vhacd.urdf>`_
   pointing to `Bullet/data/teddy2_VHACD_CHs.obj <https://github.com/bulletphysics/bullet3/blob/master/data/teddy2_VHACD_CHs.obj>`_,
   or `duck_vhacd.urdf <https://github.com/bulletphysics/bullet3/blob/master/data/duck_vhacd.urdf>`_
   pointing to `duck_vhacd.obj <https://github.com/bulletphysics/bullet3/blob/master/data/duck_vhacd.obj>`_.
3. Reduce the number of vertices in a triangle mesh. For example Blender 3D
   has a great mesh decimation modifier that interactively lets you see the
   result of the mesh simplification.
4. Use small positive values for rolling friction (0.01 for example) and spinning
   friction for round objects such as sphere and capsules and robotic grippers
   using the <rolling_friction> and <spinning_friction> nodes inside <link><contact>
   nodes. See for example `Bullet/data/sphere2.urdf <https://github.com/bulletphysics/bullet3/blob/master/data/sphere2.urdf>`_.
5. Use a small amount of compliance for wheels using the <stiffness value="30000"/>
   <damping value="1000"/> inside the URDF <link><contact> xml node. See for
   example the `Bullet/data/husky/husky.urdf <https://github.com/bulletphysics/bullet3/blob/master/data/husky/husky.urdf>`_
   vehicle.
6. Use the double precision build of Bullet, this is good both for contact
   stability and collision accuracy. Choose some good constraint solver
   setting and time step.
7. Decouple the physics simulation from the graphics. PyBullet already does
   this for the GUI and various physics servers: the OpenGL graphics
   visualization runs in its own thread, independent of the physics simulation.

Question: What are the options for friction handling?
-----------------------------------------------------

Answer: by default, Bullet and PyBullet uses an exact implicit cone friction
for the Coulomb friction model. In addition, You can enable rolling and
spinning friction by adding a <rolling_friction> and <spinning_friction>
node inside the <link><contact> node, see the
`Bullet/data/sphere2.urdf <https://github.com/bulletphysics/bullet3/blob/master/data/sphere2.urdf>`_
for example. Instead of the cone friction, you can enable pyramidal
approximation.

Question: What kind of constant or threshold inside Bullet, that makes high speeds impossible?
----------------------------------------------------------------------------------------------
Answer: By default, Bullet relies on discrete collision detection in
combination with penetration recovery. Relying purely on discrete
collision detection means that an object should not travel faster than
its own radius within one timestep. PyBullet uses 1./240 as a default
timestep. Time steps larger than 1./60 can introduce instabilities for
various reasons (deep penetrations, numerical integrator). Bullet has is
an option for continuous collision detection to catch collisions for objects
that move faster than their own radius within one timestep. Unfortunately,
this continuous collision detection can introduce its own issues (performance
and non-physical response, lack of restitution), hence this experimental
feature is not enabled by default. Check out the
`experimentalCcdSphereRadius.py <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/experimentalCcdSphereRadius.py>`_
example how to enable it.

Question: Some APIs are not documented.
---------------------------------------

Usually this means that either (1) we haven't updated the Quickstart Guide
yet or (2) the feature is too experimental to document. You can file an
issue in the tracker if you really want to know about a specific undocumented
API.