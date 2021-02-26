*********************
Deformables and Cloth
*********************

Aside from articulated multi body and rigid body dynamics, PyBullet also
implements deformable and cloth simulation, using Finite Element Method
(FEM) mass spring systems as well as using Position Based Dynamics (PBD).
Some research papers using the implementation are

* `Sim-to-Real Reinforcement Learning for Deformable Object Manipulation <https://sites.google.com/view/sim-to-real-deformable>`_ (`thesis <https://www.imperial.ac.uk/media/imperial-college/faculty-of-engineering/computing/public/1718-ug-projects/Jan-Matas-Learning-end-to-end-robotic-manipulation-of-deformable-objects.pdf>`_)
* `Assistive Gym: A Physics Simulation Framework for Assistive Robotics <https://arxiv.org/abs/1910.04700>`_
* `Learning to Rearrange Deformable Cables, Fabrics, and Bags with Goal-Conditioned Transporter Networks <https://berkeleyautomation.github.io/bags/>`_

PyBullet implements two-way coupling between multi-body / rigid body and
deformables. Two-way coupling works for collisions as well as manual
created attachments (see :func:`~pybullet_api.createSoftBodyAnchor`).

By default, PyBullet will use position based dynamics (PBD). You can enable
Finite Element Method (FEM) based simulation by resetting the world as follows:

.. code-block:: python

    pybullet.resetSimulation(pybullet.RESET_USE_DEFORMABLE_WORLD)

See also
`deformable_torus.py <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/deformable_torus.py>`_
as an example.

Forces on Soft Bodies
=====================

Bullet has 6 forces that can deform soft bodies:

* mass-spring force (default in PyBullet)
* Neo-Hookean force (only available for VTK meshes in PyBullet)
* corotated force (only available for deformables defined in URDF files with VTK meshes in PyBullet)
* gravity force (activated by default, can be scaled in URDF files)
* mouse picking force (activated by default)
* linear elasticity force (not available in PyBullet)

You can combine, for example, a mass-spring force, Neo-Hookean force, and a
corotated force.

Force Parameters
================

Here are typical values for the Parameters that we can see in the examples:

* Neo-Hookean force
    * mu: 6, 8, 20, 180
    * lambda: 3, 6, 80, 600
    * damping: 0.003, 0.01, 0.02; values below 0.003 are usually not stable
* mass-spring force
    * elastic_stiffness: 0.05, 2, 10, 15, 30, 100
    * damping_stiffness: 0.005, 0.2, 0.5, 1
    * bending_stiffness: none, defaults to elastic_stiffness in this case
    * bending_stride: TODO
* corotated force
    * mu: None (TODO)
    * lambda: None (TODO)

mu and lambda are called `Lamé parameters <https://en.wikipedia.org/wiki/Lam%C3%A9_parameters>`_.
Internally Bullet computes
`Young's modulus <https://en.wikipedia.org/wiki/Young%27s_modulus>`_
:math:`E = \mu (3 \lambda + 2 \mu) / (\lambda + \mu)` and the
`Poisson's ratio <https://en.wikipedia.org/wiki/Poisson%27s_ratio>`_
:math:`\nu = 0.5 \lambda / (\mu + \lambda)` from these parameters in the Neo-Hookean model.
Furthermore, in the Neo-Hookean model Bullet computes an individual damping for mu
and lambda by multiplying the damping parameter with mu and lambda respectively.
You can also compute the Lamé parameters from Poisson's ratio and Young's modulus:
:math:`\mu = 0.5 E / (1 + \nu)` and :math:`\lambda = E \nu / ((1 + \nu) \cdot (1 - 2 \nu))`.
Poisson's ratio must be in `[0, 0.5)` and 0.49 is almost volume-preserving.
The unit of Young's modulus is Pascal (Pa) and values for soft material can be in
the range of 10,000 to 1,000,000 Pa.

Creating Soft Bodies
====================

There are a few ways to create deformable objects, both cloth or volumetric.
:func:`~pybullet_api.loadSoftBody` lets you load a deformable object from a VTK
or OBJ file.

The **recommended way** to define soft bodies is to use PyBullet's extended URDF
format. PyBullet extends the URDF format to specify deformable objects using
the 'deformable' tag (example:
`deformable_torus.urdf <https://github.com/bulletphysics/bullet3/blob/5233b72160df8e8915320b03793d85dd0c6792ba/data/torus_deform.urdf>`_).

The following is a complete list of all available tags and attributes, of which
not all have to be used:

.. code-block:: xml

    <deformable name="torus">
        <inertial>
            <mass value="1" />
            <inertia ixx="0.0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0" />
        </inertial>
        <collision_margin value="0.006" />
        <friction value= "0.5" />
        <repulsion_stiffness value="0.5" />
        <gravity_factor value="1" />
        <cache_barycenter />

        <!-- mass-spring force -->
        <spring elastic_stiffness="1" damping_stiffness="0.1" bending_stiffness="0.1" bending_stride="0"/>
        <!-- Neo-Hookean force -->
        <neohookean mu= "1" lambda= "1" damping= "0.1" />
        <!-- corotated force -->
        <corotated mu="1" lambda="1" />

        <visual filename="torus.vtk"/>
        <collision filename="torus.vtk"/>
    </deformable>

For details, see the `URDF parser <https://github.com/bulletphysics/bullet3/blob/master/examples/Importers/ImportURDFDemo/UrdfParser.cpp#L1121>`_.

Additional references
=====================

* Position Based Dynamics, https://matthias-research.github.io/pages/publications/posBasedDyn.pdf
* Extended Position-Based Dynamics, https://blog.mmacklin.com/2016/09/15/xpbd/ (TODO not sure which one is actually used and when)
* Neo-Hookean Model (implemented by Bullet): Stable Neo-Hookean Flesh Simulation, https://graphics.pixar.com/library/StableElasticity/paper.pdf
* Finite Element Method: https://en.wikipedia.org/wiki/Finite_element_method

VTK Meshes
==========

To use all the available forces in PyBullet, we have to load a VTK mesh from
a URDF file:

* `VTK file formats <https://vtk.org/wp-content/uploads/2015/04/file-formats.pdf>`_
* `Writing VTK files in Python <https://vtk.org/Wiki/VTK/Writing_VTK_files_using_python>`_

`Mesh conversion procedure: <https://github.com/bulletphysics/bullet3/issues/2726>`_

* You should use `TetWild <https://github.com/Yixin-Hu/TetWild>`_
  (dependencies on Ubuntu: `libgmp-dev libmpfr-dev libboost-all-dev`)
  to convert arbitrary meshes to tetrahedrons via `./TetWild ~/box.stl`,
  which will create a file `box_.msh`. You can play around with the
  options `--ideal-absolute-edge-length FLOAT` or `--ideal-edge-length FLOAT`
  to simplify the generated mesh. Good values are, e.g., 100.
* You can convert this file to VTK with `gmsh <https://pypi.org/project/gmsh/>`_
  via `gmsh -format vtk -save -o box.vtk box_.msh`, which will create `box.vtk`.

Soft Body Dynamics in Bullet
============================

The soft body dynamics provides rope, cloth simulation and volumetric soft
bodies, on top of the existing rigid body dynamics. There is two-way
interaction between soft bodies, rigid bodies and collision objects

* `btSoftBody` is the main soft body object. It is derived from
  `btCollisionObject`. Unlike rigid bodies, soft bodies don’t have a single
  world transform: each node/vertex is specified in world coordinate.
* `btSoftRigidDynamicsWorld` is the container for soft bodies, rigid bodies
  and collision objects.

It is best to learn from `examples/SoftBodyDemo` how to use soft body
simulation.

Here are some basic guidelines in a nutshell:

Construction from a triangle mesh
---------------------------------

The `btSoftBodyHelpers::CreateFromTriMesh` can automatically create a soft
body from a triangle mesh.

Collision clusters
------------------

By default, soft bodies perform collision detection between vertices
(nodes) and triangles (faces). This requires a dense tessellation, otherwise
collisions might be missed. An improved method uses automatic decomposition
into convex deformable clusters. To enable collision clusters, use

.. code-block:: c++

    psb->generateClusters(numSubdivisions);
    //enable cluster collision between soft body and rigid body
    psb->m_cfg.collisions+=btSoftBody::fCollision::CL_RS;
    //enable cluster collision between soft body and soft body
    psb->m_cfg.collisions += btSoftBody::fCollision::CL_SS;

The Softbody of the ExampleBrowser has a debug option to visualize the
convex collision clusters.

Applying forces to a soft body
------------------------------

There are methods to apply a force to each vertex (node) or at an individual node:

.. code-block:: c++

    softbody ->addForce(const btVector3& forceVector);
    softbody ->addForce(const btVector3& forceVector, int node);

Soft body constraints
---------------------

It is possible to fix one or more vertices (nodes), making it immovable:

.. code-block:: c++

    softbody->setMass(node,0.f);

or to attach one or more vertices of a soft body to a rigid body:

.. code-block:: c++

    softbody->appendAnchor(int node,btRigidBody* rigidbody, bool disableCollisionBetweenLinkedBodies=false);

It is also possible to attach two soft bodies using constraints, see
`Bullet/Demos/SoftBody`.
