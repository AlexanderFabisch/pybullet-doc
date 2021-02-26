********************************
Collision Detection and Friction
********************************

You can query the contact point information that existed during the last
:func:`~pybullet_api.stepSimulation`. To get the contact points you can use
:func:`~pybullet_api.getContactPoints`. Note that
:func:`~pybullet_api.getContactPoints` will not recompute any contact point
information.

Enable/Disable Collisions
=========================

By default, collision detection is enabled between different dynamic moving
bodies. Self-collision between links of the same body can be enabled using
flags such as 'URDF_USE_SELF_COLLISION' flag in :func:`~pybullet_api.loadURDF`.

You can enable and disable collision detection between groups of objects using
:func:`~pybullet_api.setCollisionFilterGroupMask`.

URDF Extensions
===============

PyBullet extends the URDF format to include contact coefficients for
individual links. For example:

.. code-block:: xml

    <link name="...">
        <contact>
            <!-- default values are listed here -->
            <inertia_scaling value="1"/>
            <lateral_friction value="0.5"/> <!-- https://en.wikipedia.org/wiki/Friction -->
            <rolling_friction value="0"/>
            <restitution value="0"/> <!-- https://en.wikipedia.org/wiki/Coefficient_of_restitution -->
            <spinning_friction value="0"/>
            <stiffness value="1e30"/> <!-- https://en.wikipedia.org/wiki/Stiffness -->
            <damping value="1"/> <!-- https://en.wikipedia.org/wiki/Damping#Definition -->
            <friction_anchor/>
        </contact>
    </link>

Note that this is different from standard URDF, which defines contact
coefficients per
`collision tag <http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model#Contact_Coefficients>`_.

Friction Coefficients
=====================

In physics we usually use a friction coefficient to describe the contact
behavior of two materials. In Bullet we define a friction coefficient for
each material individually. The friction coefficient of the interaction
between two materials will be computed as the
`product of the individual friction coefficients
<https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=10985>`_.

General Tips for Collision Detection and Friction
=================================================

.. note::

    The following tips are from Section 13 of the
    `Bullet User Manual <https://raw.githubusercontent.com/bulletphysics/bullet3/master/docs/Bullet_User_Manual.pdf>`_.
    They apply to the C++ interface. Not everything directly translates to
    PyBullet.

Avoid very small and very large collision shapes
------------------------------------------------

The minimum object size for moving objects is about 0.2 units, 20 centimeters
for Earth gravity. If smaller objects or bigger gravity are manipulated,
reduce the internal simulation frequency accordingly, using the third argument
of `btDiscreteDynamicsWorld::stepSimulation`. By default it is 60 Hz. For instance,
simulating a dice throw (1cm-wide box with a gravity of :math:`9.8 m/s^2`)
requires a frequency of at least 300Hz (1./300.). It is recommended to keep
the maximum size of moving objects smaller then about 5 units/meters.

Avoid large mass ratios (differences)
-------------------------------------

Simulation becomes unstable when a heavy object is resting on a very light
object. It is best to keep the mass around 1. This means accurate interaction
between a tank and a very light object is not realistic.

Combine multiple static triangle meshes into one
------------------------------------------------

Many small `btBvhTriangleMeshShape` pollute the broadphase. Better combine them.

Use the default internal fixed timestep
---------------------------------------

Bullet works best with a fixed internal timestep of at least 60 hertz
(1/60 second).

For safety and stability, Bullet will automatically subdivide the variable
timestep into fixed internal simulation substeps, up to a maximum number of
substeps specified as second argument to stepSimulation. When the timestep
is smaller then the internal substep, Bullet will interpolate the motion.

This safety mechanism can be disabled by passing 0 as maximum number of
substeps (second argument to stepSimulation): the internal timestep and
substeps are disabled, and the actual timestep is simulated. It is not
recommended to disable this safety mechanism.

For ragdolls use btConeTwistConstraint
--------------------------------------

It is better to build a ragdoll out of `btHingeConstraint` and/or
`btConeTwistLimit` for knees, elbows and arms.

Don’t set the collision margin to zero
--------------------------------------

Collision detection system needs some margin for performance and stability.
If the gap is noticeable, please compensate the graphics representation.

Use less then 100 vertices in a convex mesh
-------------------------------------------

It is best to keep the number of vertices in a `btConvexHullShape` limited.
It is better for performance, and too many vertices might cause instability.
Use the `btShapeHull` utility to simplify convex hulls.

Avoid huge or degenerate triangles in a triangle mesh
-----------------------------------------------------

Keep the size of triangles reasonable, say below 10 units/meters. Also
degenerate triangles with large size ratios between each sides or close
to zero area can better be avoided.

Per triangle friction and restitution value
-------------------------------------------

By default, there is only one friction value for one rigid body. You can
achieve per shape or per triangle friction for more detail. See the
`Demos/ConcaveDemo` how to set the friction per triangle. Basically, add
CF_CUSTOM_MATERIAL_CALLBACK to the collision flags or the rigid body, and
register a global material callback function. To identify the triangle in
the mesh, both triangleID and partId ofthe mesh is passed to the material
callback. This matches the triangleId/partId of the striding mesh interface.
An easier way is to use the `btMultimaterialTriangleMeshShape`. See the
`Demos/MultiMaterialDemo` for usage.

Other MLCP Constraint Solvers
-----------------------------

Bulletuses its `btSequentialImpulseConstraintSolver` by default. You can use
a different constraint solver, by passing it into the constructor of your
`btDynamicsWorld`. Those alternative MLCP constraint solvers are in
`Bullet/src/BulletDynamics/MLCPSolvers`. See the source code of
`examples/vehicles/VehicleDemo` how to use a different constraint solver.

Custom Friction Model
---------------------

If you want to have a different friction model for certain types of objects,
you can register a friction function in the constraint solver for certain
body types. This feature is not compatible with the cache friendly constraint
solver setting.

See `#define USER_DEFINED_FRICTION_MODEL` in `Demos/CcdPhysicsDemo.cpp`.
