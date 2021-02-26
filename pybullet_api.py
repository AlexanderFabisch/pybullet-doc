import inspect
import pybullet


__doc__ = pybullet.__doc__


for name, entry in inspect.getmembers(pybullet, inspect.isroutine):
    original_docstring = entry.__doc__
    locals()[name] = lambda x: x
    locals()[name].__doc__ = "**Original docstring:** %s" % original_docstring


###############################################################################
# Connect, disconnect, bullet_client
###############################################################################


def connect(method, key=pybullet.SHARED_MEMORY_KEY, hostname="localhost", port=1234, options=""):
    """Connect to an existing physics server (using shared memory by default).

    **Overview of Connection Methods**

    Diagram with various physics client (blue) and physics server (green)
    options. Dark green servers provide OpenGL debug visualization.

    .. image:: ../_static/10000201000003C00000021C8FB399541FC5E271.png
        :alt: Diagram
        :width: 100%
        :align: center

    **Connect using DIRECT, GUI**

    The DIRECT connection sends the commands directly to the physics engine,
    without using any transport layer and no graphics visualization window,
    and directly returns the status after executing the command.

    The GUI connection will create a new graphical user interface (GUI) with
    3D OpenGL rendering, within the same process space as PyBullet. On Linux
    and Windows this GUI runs in a separate thread, while on OSX it runs in
    the same thread due to operating system limitations. On Mac OSX you may
    see a spinning wheel in the OpenGL Window, until you run a 'stepSimulation'
    or other PyBullet command.

    The commands and status messages are sent between PyBullet client and the
    GUI physics simulation server using an ordinary memory buffer.

    It is also possible to connect to a physics server in a different process on
    the same machine or on a remote machine using SHARED_MEMORY, UDP or TCP
    networking. See the section about Shared Memory, UDP and TCP for details.

    Unlike almost all other methods, this method doesn't parse keyword arguments,
    due to backward compatibility.

    **Connect using Shared Memory**

    There are a few physics servers that allow shared memory connection: the
    App_SharedMemoryPhysics, App_SharedMemoryPhysics_GUI and the Bullet Example
    Browser has one example under Experimental/Physics Server that allows shared
    memory connection. This will let you execute the physics simulation and
    rendering in a separate process.

    You can also connect over shared memory to the App_SharedMemoryPhysics_VR, the
    Virtual Reality application with support for head-mounted display and 6-dof
    tracked controllers such as HTC Vive and Oculus Rift with Touch controllers.
    Since the Valve OpenVR SDK only works properly under Windows, the
    App_SharedMemoryPhysics_VR can only be build under Windows using premake
    (preferably) or cmake.

    **Connect using UDP or TCP networking**

    For UDP networking, there is a App_PhysicsServerUDP that listens to a certain
    UDP port. It uses the open source enet library for reliable UDP networking.
    This allows you to execute the physics simulation and rendering on a separate
    machine. For TCP PyBullet uses the clsocket library. This can be useful when
    using SSH tunneling from a machine behind a firewall to a robot simulation.
    For example you can run a control stack or machine learning using PyBullet
    on Linux, while running the physics server on Windows in Virtual Reality
    using HTC Vive or Rift.

    One more UDP application is the App_PhysicsServerSharedMemoryBridgeUDP
    application that acts as a bridge to an existing physics server: you can
    connect over UDP to this bridge, and the bridge connects to a physics
    server using shared memory: the bridge passes messages between client and
    server. In a similar way there is a TCP version (replace UDP by TCP).

    There is also a GRPC client and server support, which is not enabled by
    default. You can try it out using the premake4 build system using the
    --enable_grpc option (see Bullet/build3/premake4).

    Note: at the moment, both client and server need to be either 32bit or
    64bit builds!

    **bullet_client**

    If you want to use multiple independent simulations in parallel, you can
    use pybullet_utils.bullet_client. An instance of
    bullet_client.BulletClient(connection_mode=pybullet.GUI, options='') has
    the same API as a pybullet instance. It will automatically add the
    appropriate physicsClientId to each API call. The PyBullet Gym
    environments use bullet_client to allow training of multiple environments
    in parallel, see the implementation in env_bases.py. Another small example
    shows how to have two separate instances, each with their own objects,
    see multipleScenes.py.

    Examples
    --------

    .. code-block:: python

        pybullet.connect(pybullet.DIRECT)
        pybullet.connect(pybullet.GUI, options="--opengl2")
        pybullet.connect(pybullet.GUI, options="--background_color_red=1 --background_color_blue=1 --background_color_green=1")
        pybullet.connect(pybullet.SHARED_MEMORY, 1234)
        pybullet.connect(pybullet.UDP, "192.168.0.1")
        pybullet.connect(pybullet.UDP, "localhost", 1234)
        pybullet.connect(pybullet.TCP, "localhost", 6667)

    Parameters
    ----------
    method : int
        Possible options: DIRECT, GUI, SHARED_MEMORY, UDP, TCP, GUI_SERVER,
        SHARED_MEMORY_SERVER, SHARED_MEMORY_GUI.
        DIRECT mode create a new physics engine and directly communicates with it.
        GUI will create a physics engine with graphical GUI frontend and
        communicates with it. SHARED_MEMORY will connect to an existing physics
        engine process on the same machine, and communicates with it over shared
        memory. TCP or UDP will connect to an existing physics server over TCP or
        UDP networking. GUI_SERVER is similar to GUI but also acts as a server that
        allows external SHARED_MEMORY connections. SHARED_MEMORY_SERVER is similar
        to DIRECT but also acts as a server that allows external SHARED_MEMORY
        connections. SHARED_MEMORY_GUI is similar to DIRECT but will attempt to
        connect to an external graphics server for display. The Bullet
        ExampleBrowser has an option to act as Physics Server or Graphics Server.

    key : int, optional (default: pybullet.SHARED_MEMORY_KEY)
        In SHARED_MEMORY mode, optional shared memory key. When starting
        ExampleBrowser or SharedMemoryPhysics_* you can use optional command-line
        --shared_memory_key to set the key. This allows to run multiple servers on
        the same machine.

    hostname : str, optional (default: 'localhost')
        IP address or host name, for example "127.0.0.1" or "localhost" or
        "mymachine.domain.com"

    port : int, optional (default: 1234)
        UDP port number. Default UDP port is 1234, default TCP port is 6667
        (matching the defaults in the server)

    options : str, optional (default: '')
        Command-line option passed into the GUI server.
        For example, you can set the background color, with red/green/blue
        parameters in the range [0..1] as follows:
        `p.connect(p.GUI, options="--background_color_red=1 --background_color_blue=1 --background_color_green=1")`
        Options are:

        * `--background_color_red=1` (red channel of background color)
        * `--background_color_blue=1` (blue channel of background color)
        * `--background_color_green=1` (green channel of background color)
        * `--mouse_move_multiplier=0.400000`  (mouse sensitivity)
        * `--mouse_wheel_multiplier=0.400000` (mouse wheel sensitivity)
        * `--width=<int>` width of the window in pixels
        * `--height=<int>` height of the window, in pixels.
        * `--mp4=moviename.mp4` (records movie, requires ffmpeg)
        * `--mp4fps=<int>` (for movie recording, set frames per second).
        * `--png_skip_frames=<int>` (skip 'n' frames in between screenshots)

    Returns
    -------
    physicsClientId : int
        A physics client id or -1 if not connected. The physics client Id is an
        optional argument to most of the other PyBullet commands. If you don't
        provide it, it will assume physics client id = 0. You can connect to
        multiple different physics servers, except for GUI.
    """
    return 0


def getConnectionInfo(physicsClientId=0):
    """Get connection info.

    Parameters
    ----------
    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    connectionInfo : dict
        Dictionary with entries 'isConnected' and 'connectionMethod',
        where connection method corresponds to the argument `method` of
        :func:`~pybullet_api.connect`.
    """


def isConnected(physicsClientId=0):
    """Return if a given client id is connected.

    Parameters
    ----------
    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    is_connected : bool
        Is the client id connected?
    """


def setTimeOut(timeOutInSeconds, physicsClientId=0):
    """Set the timeout in seconds (used for most of the API calls).

    If a command is not processed by the server within a specific timeout
    value, the client will disconnect.

    Parameters
    ----------
    timeOutInSeconds : int
        Timeout value in seconds.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def disconnect(physicsClientId=0):
    """Disconnect from the physics server.

    Parameters
    ----------
    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def resetSimulation(flags=0, physicsClientId=0):
    """Reset the simulation: remove all objects and start from an empty world.

    Parameters
    ----------
    flags : int, optional (default: 0)
        Experimental flags, best to ignore.
        Possible options: RESET_USE_SIMPLE_BROADPHASE,
        RESET_USE_DEFORMABLE_WORLD, RESET_USE_DISCRETE_DYNAMICS_WORLD

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def setPhysicsEngineParameter():
    """Set some internal physics engine parameter, such as cfm or erp etc.

    You can set physics engine parameters using :func:`~pybullet_api.setPhysicsEngineParameter`.

    Parameters
    ----------
    fixedTimeStep : float
        See the warning in the setTimeStep section. physics engine timestep in
        fraction of seconds, each time you call 'stepSimulation' simulated
        time will progress this amount. Same as 'setTimeStep'

    numSolverIterations : int
        Choose the maximum number of constraint solver iterations. If the
        solverResidualThreshold is reached, the solver may terminate before
        the numSolverIterations.

    useSplitImpulse : int
        Advanced feature, only when using maximal coordinates: split the
        positional constraint solving and velocity constraint solving in
        two stages, to prevent huge penetration recovery forces.

    splitImpulsePenetrationThreshold : float
        Related to 'useSplitImpulse': if the penetration for a particular
        contact constraint is less than this specified threshold, no split
        impulse will happen for that contact.

    numSubSteps : int
        Subdivide the physics simulation step further by 'numSubSteps'.
        This will trade performance over accuracy.

    collisionFilterMode : int
        Use 0 for default collision filter: (group A&maskB) AND
        (groupB&maskA). Use 1 to switch to the OR collision filter:
        (group A&maskB) OR (groupB&maskA)

    contactBreakingThreshold : int
        Contact points with distance exceeding this threshold are not
        processed by the LCP solver. In addition, AABBs are extended
        by this number. Defaults to 0.02 in Bullet 2.x.

    maxNumCmdPer1ms : int
        Experimental: add 1ms sleep if the number of commands executed exceed
        this threshold.

    enableFileCaching : int
        Set to 0 to disable file caching, such as .obj wavefront file loading

    restitutionVelocityThreshold : float
        If relative velocity is below this threshold, restitution will be zero.

    erp : float
        Constraint error reduction parameter (non-contact, non-friction)

    contactERP : float
        Contact error reduction parameter

    frictionERP : float
        Friction error reduction parameter (when positional friction anchors
        are enabled)

    enableConeFriction : int
        Set to 0 to disable implicit cone friction and use pyramid
        approximation (cone is default). NOTE: Although enabled by default,
        it is worth trying to disable this feature, in case there are
        friction artifacts.

    deterministicOverlappingPairs : int
        Set to 1 to enable and  0 to disable sorting of overlapping pairs
        (backward compatibility setting).

    allowedCcdPenetration : float
        If contrinuous collision detection (CCD) is enabled, CCD will not
        be used if the penetration is below this threshold.

    jointFeedbackMode : int
        Speficy joint feedback frame: JOINT_FEEDBACK_IN_WORLD_SPACE or
        JOINT_FEEDBACK_IN_JOINT_FRAME

    solverResidualThreshold : float, optional (default: 1e-7)
        Velocity threshold, if the maximum velocity-level error for each
        constraint is below this threshold the solver will terminate
        (unless the solver hits the numSolverIterations).

    contactSlop : float
        Position correction of contacts is not resolved below this threshold,
        to allow more stable contact.

    enableSAT : int
        If true/1, enable separating axis theorem based convex collision
        detection, if features are available (instead of using GJK and EPA).
        Requires URDF_INITIALIZE_SAT_FEATURES in
        :func:`~pybullet_api.loadURDF`.
        See satCollision.py example.

    constraintSolverType : int
        Experimental (best to ignore): allow to use a direct LCP solver, such
        as Dantzig. See switchConstraintSolverType.py example.

    globalCFM : float
        Experimental (best to ignore) global default constraint force mixing
        parameter.

    minimumSolverIslandSize : int
        Experimental (best to ignore), minimum size of constraint solving
        islands, to avoid very small islands of independent constraints.

    reportSolverAnalytics : int
        When True/1, additional solve analytics is available.
        The will be return values of :func:`~pybullet_api.stepSimulation`.

    warmStartingFactor : float
        Fraction of previous-frame force/impulse that is used to initialize the
        initial solver solution

    sparseSdfVoxelSize : float
        TODO related to soft bodies

    numNonContactInnerIterations : int
        TODO

    physicsClientId : int, optional (default: 0)
        If you are connected to multiple servers, you can pick one.
    """


def getPhysicsEngineParameters(physicsClientId=0):
    """Get the current values of internal physics engine parameters.

    Parameters
    ----------
    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    physics_engine_parameters : dict
        Values of parameters fixedTimeStep, numSubSteps, numSolverIterations,
        useRealTimeSimulation, gravityAccelerationX, gravityAccelerationY,
        gravityAccelerationZ, numNonContactInnerIterations
    """


def setTimeStep(timeStep, physicsClientId=0):
    """Set the amount of time to proceed at each call to :func:`~pybullet_api.stepSimulation`.

    .. warning::

        In many cases it is best to leave the timeStep to default, which is
        240Hz. Several parameters are tuned with this value in mind. For
        example, the number of solver iterations and the error reduction
        parameters (erp) for contact, friction and non-contact joints are
        related to the time step. If you change the time step, you may need
        to re-tune those values accordingly, especially the erp values.

    You can set the physics engine timestep that is used when calling
    :func:`~pybullet_api.stepSimulation`. It is best to only call this method
    at the start of a simulation. Don't change this time step regularly.
    Changing the timestep can also be achieved using
    :func:`~pybullet_api.setPhysicsEngineParameter`.

    Parameters
    ----------
    timeStep : float
        Each time you call :func:`~pybullet_api.stepSimulation` the timeStep
        will proceed with 'timeStep'.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def setDefaultContactERP(defaultContactERP, physicsClientId=0):
    """Set the amount of contact penetration Error Recovery Parameter (ERP) in each time step.

    Parameters
    ----------
    defaultContactERP : float
        Amount of contact penetration Error Recovery Parameter (ERP) in each
        time step. This is a tuning parameter to control resting contact
        stability. This value depends on the time step.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def loadURDF(
        fileName, basePosition=(0, 0, 0), baseOrientation=(0, 0, 0, 1),
        useMaximalCoordinates=False, useFixedBase=False, flags=0, globalScaling=1):
    """Create a multibody by loading a URDF file.

    The loadURDF will send a command to the physics server to load a physics
    model from a Universal Robot Description File (URDF). The URDF file is
    used by the ROS project (Robot Operating System) to describe robots and
    other objects, it was created by the WillowGarage and the Open Source
    Robotics Foundation (OSRF). Many robots have public URDF files, you can
    find a description and tutorial here: http://wiki.ros.org/urdf/Tutorials

    Important note: most joints (slider, revolute, continuous) have motors
    enabled by default that prevent free motion. This is similar to a robot
    joint with a very high-friction harmonic drive. You should set the
    joint motor control mode and target settings using
    :func:`~pybullet_api.setJointMotorControl2`.

    Warning: by default, PyBullet will cache some files to speed up loading.
    You can disable file caching using setPhysicsEngineParameter(enableFileCaching=0).

    By default, loadURDF will use a convex hull for mesh collision detection.
    For static (mass = 0, not moving) meshes, you can make the mesh concave
    by adding a tag in the URDF: <link concave="yes" name="baseLink"> see
    `samurai.urdf <https://github.com/bulletphysics/bullet3/blob/master/data/samurai.urdf>`_
    for an example. There are some other extensions to the URDF format, you
    can browser the examples to explore. PyBullet doesn't process all
    information from a URDF file. See the examples and URDF files to get an
    idea what features are supported. Usually there is a Python API instead
    to control the feature. Each link can only have a single material, so
    if you have multiple visual shapes with different materials, you need
    to split them into separate links, connected by fixed joints. You can
    use the OBJ2SDF utility to do this, part of Bullet.

    ROS-compliant URDF files often include `file://`, `package://`, or
    `model://` in their paths to mesh files. These parts will be dropped
    when searching for the meshes. The mesh will be searched with
    respect to the base paths `../../`, `../`, `./`, and all of the
    parent folders that can be extracted from the path to the URDF file.

    Parameters
    ----------
    fileName : str
        A relative or absolute path to the URDF file on the file system of the physics server.

    basePosition : array-like, shape (3,), optional (default: (0, 0, 0))
        Create the base of the object at the specified position in world
        space coordinates [X,Y,Z]. Note that this position is of the URDF
        link position. If the inertial frame is non-zero, this is different
        from the center of mass position. Use
        :func:`~pybullet_api.resetBasePositionAndOrientation` to set the
        center of mass location/orientation.

    baseOrientation : array-like, shape (4,), optional (default: (0, 0, 0, 1))
        Create the base of the object at the specified orientation as world
        space quaternion [X,Y,Z,W]. See note in basePosition.

    useMaximalCoordinates : int, optional (default: False)
        Experimental. By default, the joints in the URDF file are created
        using the reduced coordinate method: the joints are simulated
        using the Featherstone Articulated Body Algorithm (ABA, btMultiBody
        in Bullet 2.x). The useMaximalCoordinates option will create a 6
        degree of freedom rigid body for each link, and constraints between
        those rigid bodies are used to model joints.

    useFixedBase : int, optional (default: 0)
        Force the base of the loaded object to be static.

    flags : int, optional (default: 0)
        The following flags can be combined using a bitwise OR, `|`:
        URDF_MERGE_FIXED_LINKS: this will remove fixed links from the URDF
        file and merge the resulting links. This is good for performance,
        since various algorithms (articulated body algorithm, forward
        kinematics etc) have linear complexity in the number of joints,
        including fixed joints.
        URDF_USE_INERTIA_FROM_FILE: by default, Bullet recomputed the
        inertia tensor based on mass and volume of the collision shape.
        If you can provide more accurate inertia tensor, use this flag.
        URDF_USE_SELF_COLLISION: by default, Bullet disables self-collision.
        This flag let's you enable it. You can customize the self-collision
        behavior using the following flags:
        URDF_USE_SELF_COLLISION_INCLUDE_PARENT will enable collision between
        child and parent, it is disabled by default. Needs to be used together
        with URDF_USE_SELF_COLLISION flag.
        URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS will discard
        self-collisions between a child link and any of its ancestors
        (parents, parents of parents, up to the base). Needs to be used
        together with URDF_USE_SELF_COLLISION.
        URDF_USE_IMPLICIT_CYLINDER, will use a smooth implicit cylinder.
        By default, Bullet will tesselate the cylinder into a convex hull.
        URDF_ENABLE_SLEEPING, will allow to disable simulation after a body
        hasn't moved for a while. Interaction with active bodies will
        re-enable simulation.
        URDF_INITIALIZE_SAT_FEATURES, will create triangle meshes for convex
        shapes. This will improve visualization and also allow usage of the
        separating axis test (SAT) instead of GJK/EPA. Requires to enableSAT
        using setPhysicsEngineParameter.
        URDF_USE_MATERIAL_COLORS_FROM_MTL, will use the RGB color from the
        Wavefront OBJ file, instead of from the URDF file.
        URDF_ENABLE_CACHED_GRAPHICS_SHAPES, will cache and re-use graphics
        shapes. It will improve loading performance for files with similar
        graphics assets.
        URDF_MAINTAIN_LINK_ORDER, will try to maintain the link order from
        the URDF file. Say in the URDF file, the order is: ParentLink0,
        ChildLink1 (attached to ParentLink0), ChildLink2 (attached to
        ParentLink0). Without this flag, the order could be P0, C2, C1.

    globalScaling : float
        globalScaling will apply a scale factor to the URDF model.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    bodyUniqueID : int
        Non-negative integer value. If the URDF file cannot be loaded, this
        integer will be negative and not a valid body unique id.
    """


def loadSDF(sdfFileName, useMaximalCoordinates, globalScaling, physicsClientId=0):
    """Load multibodies from an SDF file.

    You can load objects from other file formats, such as .bullet, .sdf and
    .mjcf. Those file formats support multiple objects, so the return
    value is a list of object unique ids. The SDF format is explained in
    detail at http://sdformat.org. This function only extracts some essential
    parts of the SDF related to the robot models and geometry, and ignores many
    elements related to cameras, lights and so on. See also the important note
    under :func:`~pybullet_api.loadURDF` related to default joint motor
    settings, and make sure to use :func:`~pybullet_api.setJointMotorControl2`.

    Parameters
    ----------
    sdfFileName : str
        A relative or absolute path to the SDF file on the file system of the
        physics server.

    useMaximalCoordinates : int, optional
        Experimental. See :func:`~pybullet_api.loadURDF` for more details.

    globalScaling : float, optional
        Every object will be scaled using this scale factor (including links,
        link frames, joint attachments and linear joint limits). This has no
        effect on mass, only on the geometry. Use
        :func:`~pybullet_api.changeDynamics` to change the mass if needed.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    objectUniqueIds : list of int
        The list includes the object unique id for each object loaded.
    """


def loadMJCF(mjcfFileName, flags=-1, useMultiBody=-1, physicsClientId=0):
    """Load multibodies from an MJCF file.

    You can load objects from other file formats, such as .bullet, .sdf and
    .mjcf. Those file formats support multiple objects, so the return
    value is a list of object unique ids. This function performs basic import
    of MuJoCo MJCF xml files, used in OpenAI Gym.

    Parameters
    ----------
    mjcfFileName : str
        A relative or absolute path to the MJCF file on the file system of the
        physics server.

    flags : int, optional
        TODO

    useMultiBody : int, optional
        TODO

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    objectUniqueIds : list of int
        The list includes the object unique id for each object loaded.
    """


def saveState(physicsClientId=0):
    """Save the full state of the world to memory.

    When you need deterministic simulation after restoring to a previously
    saved state, all important state information, including contact points,
    need to be stored. The :func:`~pybullet_api.saveWorld` command is not
    sufficient for this. You can use the :func:`~pybullet_api.restoreState`
    command to restore from a snapshot taken using
    :func:`~pybullet_api.saveState` (in-memory) or
    :func:`~pybullet_api.saveBullet` (on disk).

    Parameters
    ----------
    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    stateId : int
        Id of the saved state.
    """


def saveBullet(bulletFileName, physicsClientId=0):
    """Save the full state of the world to a .bullet file.

    When you need deterministic simulation after restoring to a previously
    saved state, all important state information, including contact points,
    need to be stored. The :func:`~pybullet_api.saveWorld` command is not
    sufficient for this. You can use the :func:`~pybullet_api.restoreState`
    command to restore from a snapshot taken using
    :func:`~pybullet_api.saveState` (in-memory) or
    :func:`~pybullet_api.saveBullet` (on disk).

    Parameters
    ----------
    bulletFileName : str
        Filename of the .bullet file that will be created.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def restoreState(stateId=-1, fileName="", physicsClientId=0):
    """Restore the full state of an existing world.

    Either the filename or state id needs to be valid. Note that this function
    will reset the positions and joint angles of objects to the saved state,
    as well as restoring contact point information. You need to make sure the
    objects and constraints are setup before calling it.
    See the
    `saveRestoreState.py <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/saveRestoreState.py>`_
    example.

    Parameters
    ----------
    stateId : int, optional
        Id of the saved state.

    fileName : str, optional
        Filename of the .bullet file.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def removeState(stateUniqueId, physicsClientId=0):
    """Remove a state created using :func:`~pybullet_api.saveState` by its state unique id.

    Parameters
    ----------
    stateUniqueId : int
        Id of the saved state.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def saveWorld(worldFileName, physicsClientId=0):
    """Save a approximate Python file to reproduce the current state of the world.

    .. warning::

        Very preliminary and approximately

    You can create an approximate snapshot of the current world as a PyBullet
    Python file, stored on the server. This can be useful as a basic editing
    feature, setting up the robot, joint angles, object positions and
    environment for example in VR. Later you can just load the PyBullet Python
    file to re-create the world. The python snapshot contains
    :func:`~pybullet_api.loadURDF` commands together with initialization of
    joint angles and object transforms. Note that not all settings are stored
    in the world file.

    Parameters
    ----------
    worldFileName : str
        Filename of the PyBullet file.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def resetBasePositionAndOrientation(bodyUniqueId, posObj, ornObj, physicsClientId=0):
    """Reset the world position and orientation of the base of the object instantaneously, not through physics simulation.

    You can reset the position and orientation of the base (root) of each
    object. It is best only to do this at the start, and not during a running
    simulation, since the command will override the effect of all physics
    simulation. The linear and angular velocity is set to zero. You can use
    :func:`~pybullet_api.resetBaseVelocity` to reset to a non-zero linear
    and/or angular velocity.

    Parameters
    ----------
    bodyUniqueId : int
        Object unique id, as returned from loadURDF.

    posObj : array-like, shape (3,)
        Reset the base (inertial frame) of the object at the specified
        position in world space coordinates [X, Y, Z].

    ornObj : array-like, shape (4,)
        Reset the base (inertial frame) of the object at the specified
        orientation as world space quaternion [X, Y, Z, W].

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


###############################################################################
# Control Physics Engine
###############################################################################


def setGravity(gravX, gravY, gravZ, physicsClientId=0):
    """Set the gravity acceleration (x, y, z).

    By default, there is no gravitational force enabled. setGravity lets
    you set the default gravity force for all objects.

    Parameters
    ----------
    gravX : float
        Gravity force along the X world axis

    gravY : float
        Gravity force along the Y world axis

    gravZ : float
        Gravity force along the Z world axis

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def startStateLogging(loggingType, fileName, objectUniqueIds=(), maxLogDof=-1,
                      bodyUniqueIdA=-1, bodyUniqueIdB=-1, linkIndexA=-2,
                      linkIndexB=-2, deviceTypeFilter=0, logFlags=0,
                      physicsClientId=0):
    """Start logging of state, such as robot base position, orientation, joint positions etc.

    State logging lets you log the state of the simulation, such as the state
    of one or more objects after each simulation step (after each call to
    :func:`~pybullet_api.stepSimulation` or automatically after each simulation
    step when real time simulation is enabled). This allows you to record
    trajectories of objects. There is also the option to log the common state
    of bodies such as base position and orientation, joint positions (angles)
    and joint motor forces.

    All log files generated using this function can be read using C++ or Python
    scripts. See quadruped_playback.py and kuka_with_cube_playback.py for
    Python scripts reading the log files. You can use
    bullet3/examples/Utils/RobotLoggingUtil.cpp/h to read the log files in C++.

    For MP4 video recording you can use the logging option
    STATE_LOGGING_VIDEO_MP4. We plan to implement various other types of
    logging, including logging the state of VR controllers.

    As a special case, we implemented the logging of the Minitaur robot. The
    log file from PyBullet simulation is identical to the real Minitaur
    quadruped log file. See Bullet/examples/pybullet/examples/logMinitaur.py
    for an example.

    .. note::

        Various loggers include their own internal timestamp that starts at
        zero when created. This means that you need to start all loggers at
        the same time, to be in sync. You need to make sure to that the
        simulation is not running in real-time mode, while starting the
        loggers: use pybullet.setRealTimeSimulation(0) before creating the
        loggers.

    Parameters
    ----------
    loggingType : int
        There are various types of logging implemented.

        * STATE_LOGGING_MINITAUR: This will require to load the
          quadruped / quadruped.urdf and object unique id from the quadruped.
          It logs the timestamp, IMU roll/pitch/yaw, 8 leg motor positions
          (q0-q7), 8 leg motor torques (u0-u7), the forward speed of the torso
          and mode (unused in simulation).
        * STATE_LOGGING_GENERIC_ROBOT: This will log a log of the data of
          either all objects or selected ones (if objectUniqueIds is provided).
        * STATE_LOGGING_VIDEO_MP4: this will open an MP4 file and start
          streaming the OpenGL 3D visualizer pixels to the file using an
          ffmpeg pipe. It will require ffmpeg installed. You can also use
          avconv (default on Ubuntu), just create a symbolic link so that
          ffmpeg points to avconv. On Windows, ffmpeg has some issues that
          cause tearing/color artifacts in some cases.
        * STATE_LOGGING_CONTACT_POINTS
        * STATE_LOGGING_VR_CONTROLLERS
        * STATE_LOGGING_PROFILE_TIMINGS: This will dump a timings file in JSON
          format that can be opened using Google Chrome about://tracing LOAD.

    fileName : str
        File name (absolute or relative path) to store the log file data.

    objectUniqueIds : list of int, optional
        If left empty, the logger may log every object, otherwise the logger
        just logs the objects in the objectUniqueIds list.

    maxLogDof : int, optional
        Maximum number of joint degrees of freedom to log (excluding the base
        dofs). This applies to STATE_LOGGING_GENERIC_ROBOT_DATA. Default value
        is 12. If a robot exceeds the number of dofs, it won't get logged at
        all.

    bodyUniqueIdA : int, optional
        Applies to STATE_LOGGING_CONTACT_POINTS. If provided,only log contact
        points involving bodyUniqueIdA.

    bodyUniqueIdB : int, optional
        Applies to STATE_LOGGING_CONTACT_POINTS. If provided,only log contact
        points involving bodyUniqueIdB.

    linkIndexA : int, optional
        Applies to STATE_LOGGING_CONTACT_POINTS. If provided,only log contact
        points involving linkIndexA for bodyUniqueIdA.

    linkIndexB : int, optional
        Applies to STATE_LOGGING_CONTACT_POINTS. If provided,only log contact
        points involving linkIndexB for bodyUniqueIdA.

    deviceTypeFilter : int, optional
        deviceTypeFilter allows you to select what VR devices to log:
        VR_DEVICE_CONTROLLER, VR_DEVICE_HMD ,VR_DEVICE_GENERIC_TRACKER or any
        combination of them. Applies to STATE_LOGGING_VR_CONTROLLERS. Default
        values is VR_DEVICE_CONTROLLER.

    logFlags : int, optional
        STATE_LOG_JOINT_TORQUES, to log joint torques due to joint motors.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    loggingUniqueId : int
        A non-negative int loggingUniqueId, that can be used with
        :func:`~pybullet_api.stopStateLogging`.

    Examples
    --------
    Check out the Python script
    `dumpLog.py <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/dumpLog.py>`_
    to parse the log data.
    """


def stopStateLogging(loggingId, physicsClientId=0):
    """Stop logging of robot state, given a loggingUniqueId.

    Parameters
    ----------
    loggingId : int
        A non-negative int loggingUniqueId, that was created with
        :func:`~pybullet_api.startStateLogging`.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def submitProfileTiming(eventName, physicsClientId=0):
    """Add a custom profile timing that will be visible in performance profile recordings on the physics server.

    On the physics server (in GUI and VR mode) you can press ‘p’ to start
    and/or stop profile recordings.

    This function allows to insert start and stop timings to profile Python
    code. See
    `profileTiming.py <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/profileTiming.py>`_
    example. PyBullet and Bullet have instrumented many functions so you can
    see where the time is spend. You can dump those profile timings in a file
    that can be viewed with Google Chrome in the about://tracing window using
    the LOAD feature. In the GUI, you can press 'p' to start/stop the profile
    dump. In some cases you may want to instrument the timings of your client
    code. You can submit profile timings using PyBullet.

    Parameters
    ----------
    eventName : str
        Name of the event.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def createMultiBody(
        baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=-1,
        basePosition=(0, 0, 0), baseOrientation=(0, 0, 0, 1),
        baseInertialFramePosition=(0, 0, 0),
        baseInertialFrameOrientation=(0, 0, 0, 1), linkMasses=None,
        linkCollisionShapeIndices=None, linkVisualShapeIndices=None,
        linkPositions=None, linkOrientations=None,
        linkInertialFramePositions=None, linkInertialFrameOrientations=None,
        linkParentIndices=None, linkJointTypes=None, linkJointAxis=None,
        useMaximalCoordinates=0, flags=-1, batchPositions=None,
        physicsClientId=0):
    """Create a multi body.

    Although the easiest way to create stuff in the world is using the loading
    functions (loadURDF/SDF/MJCF/Bullet), you can create a multi body with
    this function. See the
    `createMultiBodyLinks.py <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/createMultiBodyLinks.py>`_
    example in the Bullet Physics SDK. The parameters are very similar to URDF
    and SDF parameters.

    You can create a multi body with only a single base without joints/child
    links or you can create a multi body with joints/child links. If you
    provide links, make sure the size of every list is the same
    (len(linkMasses) == len(linkCollisionShapeIndices) etc).

    Examples
    --------

    .. code-block:: python

        cuid = pybullet.createCollisionShape(
            pybullet.GEOM_BOX, halfExtents = [1, 1, 1])
        mass= 0 #static box
        pybullet.createMultiBody(mass, cuid)

    See also createMultiBodyLinks.py, createObstacleCourse.py and
    createVisualShape.py in the Bullet/examples/pybullet/examples folder.

    Parameters
    ----------
    baseMass : float, optional (default: 0)
        Mass of the base, in kg (if using SI units)

    baseCollisionShapeIndex : int, optional
        Unique id from createCollisionShape or -1. You can re-use the
        collision shape for multiple multibodies (instancing).

    baseVisualShapeIndex : int, optional
        Unique id from createVisualShape or -1. You can reuse the
        visual shape (instancing).

    basePosition : array-like, shape (3,), optional
        Cartesian world position of the base.

    baseOrientation : array-like, shape (4,), optional
        Orientation of base as quaternion [x, y, z, w]

    baseInertialFramePosition : array-like, shape (3,), optional
        Local position of inertial frame.

    baseInertialFrameOrientation : array-like, shape (4,), optional
        Local orientation of inertial frame, [x, y, z, w].

    linkMasses : list of float, optional
        List of the mass values, one for each link.

    linkCollisionShapeIndices : list of int, optional
        List of the unique id, one for each link.

    linkVisualShapeIndices : list of int, optional
        List of the visual shape unique id for each link.

    linkPositions : array-like, shape (n_links, 3), optional
        List of local link positions, with respect to parent.

    linkOrientations : array-like, shape (n_links, 4), optional
        List of local link orientations, w.r.t. parent.

    linkInertialFramePositions : array-like, shape (n_links, 3), optional
        List of local inertial frame pos. in link frame.

    linkInertialFrameOrientations : array-like, shape (n_links, 4), optional
        List of local inertial frame orn. in link frame.

    linkParentIndices : list of int, optional
        Link index of the parent link or 0 for the base.

    linkJointTypes : list of int, optional
        List of joint types, one for each link. JOINT_REVOLUTE,
        JOINT_PRISMATIC, JOINT_SPHERICAL and JOINT_FIXED types are supported
        at the moment.

    linkJointAxis : array-like, shape (n_joints, 3), optional
        Joint axis in local frame.

    useMaximalCoordinates : int, optional
        Experimental, best to leave it 0/false.

    flags : int, optional
        Similar to the flags passed in :func:`~pybullet_api.loadURDF`,
        for example URDF_USE_SELF_COLLISION. See
        :func:`~pybullet_api.loadURDF` for explanations.

    batchPositions : array-like, shape (n_multibodies, 3), optional
        Array of base positions, for fast batch creation of many multibodies.
        See `example <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/createMultiBodyBatch.py>`_.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    nnuid : int
        Non-negative unique id or -1 for failure.
    """


###############################################################################
# Transforms: Position and Orientation
###############################################################################


def getQuaternionFromEuler(eulerAngles, physicsClientId=0):
    """Convert Euler [roll, pitch, yaw] as in URDF/SDF convention, to quaternion [x,y,z,w].

    The PyBullet API uses quaternions to represent orientations. Since
    quaternions are not very intuitive for people, there are two functions to
    convert between quaternions and Euler angles.

    Parameters
    ----------
    eulerAngles : array-like, shape (3,)
        The X,Y,Z Euler angles are in radians, accumulating 3 rotations
        expressing the roll around the X, pitch around Y and yaw around the Z
        axis. The extrinsic convention is used for concatenation of rotations.

    physicsClientId : int, optional
        Unused, added for API consistency.

    Returns
    -------
    quaternion : array-like, shape (4,)
        Quaternion [x,y,z,w]
    """


def getEulerFromQuaternion(quaternion, physicsClientId=0):
    """Convert quaternion [x,y,z,w] to Euler [roll, pitch, yaw] as in URDF/SDF convention.

    The PyBullet API uses quaternions to represent orientations. Since
    quaternions are not very intuitive for people, there are two functions to
    convert between quaternions and Euler angles.

    Parameters
    ----------
    quaternion : array-like, shape (4,)
        Quaternion [x,y,z,w]

    physicsClientId : int, optional
        Unused, added for API consistency.

    Returns
    -------
    eulerAngles : array-like, shape (3,)
        The Euler angles are in radians. The rotation order is first roll
        around X, then pitch around Y and finally yaw around Z, as in the
        ROS URDF rpy convention.
    """


def getMatrixFromQuaternion(quaternion, physicsClientId=0):
    """Compute the 3x3 matrix from a quaternion, as a list of 9 values (row-major).

    Parameters
    ----------
    quaternion : array-like, shape (4,)
        Quaternion [x,y,z,w]

    physicsClientId : int, optional
        Unused, added for API consistency.

    Returns
    -------
    mat : array-like, shape (9,)
        9 floats (row-major order), representing the matrix
    """


def getAxisAngleFromQuaternion(quaternion, physicsClientId=0):
    """Compute the axis and angle representation from quaternion.

    Parameters
    ----------
    quaternion : array-like, shape (4,)
        Quaternion [x,y,z,w]

    physicsClientId : int, optional
        Unused, added for API consistency.

    Returns
    -------
    axisAngle : tuple
        Tuple of axis (tuple of 3 floats) and angle (1 float)
    """


def multiplyTransforms(positionA, orientationA, positionB, orientationB, physicsClientId=0):
    r"""Multiply two transform, provided as position and quaternion.

    The operation that will be performed is

    .. math::

        T_A \cdot T_B,

    where :math:`T_A` corresponds to the homogeneous transformation matrix
    that represents transformation A and :math:`T_B` the matrix that
    represents transformation B. This operation is either an extrinsic
    transformation A applied to B or an intrinsic transformation B applied
    to A.

    Parameters
    ----------
    positionA : array-like, shape (3,)
        Position [x,y,z]

    orientationA : array-like, shape (4,)
        Quaternion [x,y,z,w]

    positionB : array-like, shape (3,)
        Position [x,y,z]

    orientationB : array-like, shape (4,)
        Quaternion [x,y,z,w]

    physicsClientId : int, optional
        Unused, added for API consistency.

    Returns
    -------
    positionAB : array-like, shape (3,)
        Position [x,y,z]

    orientationAB : array-like, shape (4,)
        Quaternion [x,y,z,w]
    """


def invertTransform(position, orientation, physicsClientId=0):
    """Invert a transform, provided as [position], [quaternion].

    Parameters
    ----------
    position : array-like, shape (3,)
        Position [x,y,z]

    orientation : array-like, shape (4,)
        Quaternion [x,y,z,w]

    physicsClientId : int, optional
        Unused, added for API consistency.

    Returns
    -------
    position : array-like, shape (3,)
        Inverted position [x,y,z]

    orientation : array-like, shape (4,)
        Inverted quaternion [x,y,z,w]
    """


def getDifferenceQuaternion(quaternionStart, quaternionEnd, physicsClientId=0):
    """Compute the quaternion difference from two quaternions.

    Parameters
    ----------
    quaternionStart : array-like, shape (4,)
        Start orientation [x,y,z,w]

    quaternionEnd : array-like, shape (4,)
        End orientation [x,y,z,w]

    physicsClientId : int, optional
        Unused, added for API consistency.

    Returns
    -------
    quaternion : array-like, shape (4,)
        Difference quaternion [x,y,z,w]
    """


###############################################################################
# Controlling a Robot
###############################################################################


def getNumJoints(bodyUniqueId, physicsClientId=0):
    """Get the number of joints for an object.

    After you load a robot you can query the number of joints using this
    function. For the r2d2.urdf this should return 15.

    Parameters
    ----------
    bodyUniqueId : int
        The body unique id, as returned by loadURDF etc.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    numJoints : int
        Number of joints
    """


def getJointInfo(bodyUniqueId, jointIndex, physicsClientId=0):
    """Get the name and type info for a joint on a body.

    Parameters
    ----------
    bodyUniqueId : int
        The body unique id

    jointIndex : int
        An index in the range [0, pybullet.getNumJoints(bodyUniqueId))

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    jointIndex : int
        The same joint index as the input parameter.

    jointName : str
        The name of the joint, as specified in the URDF (or SDF etc) file

    jointType : int
        Type of the joint, this also implies the number of position and
        velocity variables. JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL,
        JOINT_PLANAR, JOINT_FIXED. See the section
        :doc:`../../controlling_a_robot` for more details.

    qIndex : int
        The first position index in the positional state variables for this
        body

    uIndex : int
        The first velocity index in the velocity state variables for this body

    flags : int
        Reserved

    jointDamping : float
        The joint damping value, as specified in the URDF file

    jointFriction : float
        The joint friction value, as specified in the URDF file

    jointLowerLimit : float
        Positional lower limit for slider and revolute (hinge) joints.

    jointUpperLimit : float
        Positional upper limit for slider and revolute joints. Values ignored
        in case upper limit < lower limit.

    jointMaxForce : float
        Maximum force specified in URDF (possibly other file formats). Note
        that this value is not automatically used. You can use maxForce in
        :func:`~pybullet_api.setJointMotorControl2`.

    jointMaxVelocity : float
        Maximum velocity specified in URDF. Note that the maximum velocity is
        not used in actual motor control commands at the moment.

    linkName : str
        The name of the link, as specified in the URDF (or SDF etc.) file.

    jointAxis : array-like, shape (3,)
        Joint axis in local frame (ignored for JOINT_FIXED).

    parentFramePos : array-like, shape (3,)
        Joint position in parent frame.

    parentFrameOrn : array-like, shape (4,)
        Joint orientation in parent frame (quaternion x,y,z,w).

    parentIndex : int
        Parent link index, -1 for base.
    """



def setJointMotorControl2(bodyUniqueId, jointIndex, controlMode):
    """Set a single joint motor control mode and desired target value.

    .. note::

        setJointMotorControl is obsolete and replaced by setJointMotorControl2
        API. (Or even better use
        :func:`~pybullet_api.setJointMotorControlArray`.)

    We can control a robot by setting a desired control mode for one or more
    joint motors. There is no immediate state change,
    :func:`~pybullet_api.stepSimulation` will process the motors.
    During the :func:`~pybullet_api.stepSimulation` the physics engine will
    simulate the motors to reach the given target value that can be reached
    within the maximum motor forces and other constraints.

    .. note::

        By default, each revolute joint and prismatic joint is motorized using
        a velocity motor. You can disable those default motor by using a
        maximum force of 0. This will let you perform torque control.

    For example:

    .. code-block:: python

        maxForce = 0
        mode = pybullet.VELOCITY_CONTROL
        pybullet.setJointMotorControl2(
            objUid, jointIndex, controlMode=mode, force=maxForce)

    You can also use a small non-zero force to mimic joint friction.

    If you want a wheel to maintain a constant velocity, with a max force you
    can use:

    .. code-block:: python

        maxForce = 500
        pybullet.setJointMotorControl2(
            bodyUniqueId=objUid, jointIndex=0,
            controlMode=pybullet.VELOCITY_CONTROL, targetVelocity=targetVel,
            force=maxForce)

    The actual implementation of the joint motor controller is as a constraint
    for POSITION_CONTROL and VELOCITY_CONTROL, and as an external force for
    TORQUE_CONTROL:

    * POSITION_CONTROL
        * implementation: constraint
        * component: velocity and position constraint
        * constraint error to be minimized:
          error = position_gain*(desired_position-actual_position)+velocity_gain*(desired_velocity-actual_velocity)
    * VELOCITY_CONTROL
        * implementation: constraint
        * component: pure velocity constraint
        * constraint error to be minimized:
          error = desired_velocity - actual_velocity
    * TORQUE_CONTROL
        * implementation: external force

    Generally it is best to start with VELOCITY_CONTROL or POSITION_CONTROL.
    It is much harder to do TORQUE_CONTROL (force control) since simulating
    the correct forces relies on very accurate URDF/SDF file parameters and
    system identification (correct masses, inertias, center of mass location,
    joint friction etc).

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id as returned from loadURDF etc.

    jointIndex : int
        Link index in range [0..getNumJoints(bodyUniqueId) (note that link
        index == joint index)

    controlMode : int
        POSITION_CONTROL (which is in fact CONTROL_MODE_POSITION_VELOCITY_PD),
        VELOCITY_CONTROL, TORQUE_CONTROL and PD_CONTROL. (There is also
        experimental STABLE_PD_CONTROL for stable(implicit) PD control, which
        requires additional preparation. See humanoidMotionCapture.py and
        pybullet_envs.deep_mimc for STABLE_PD_CONTROL examples.)
        TORQUE_CONTROL will apply a torque instantly, so it only is effective
        when calling stepSimulation explicitly.

    targetPosition : float, optional
        In POSITION_CONTROL the targetValue is target position of the joint.

    targetVelocity : float, optional
        In VELOCITY_CONTROL and POSITION_CONTROL  the targetVelocity is the
        desired velocity of the joint, see implementation note below. Note
        that the targetVelocity is not the maximum joint velocity. In
        PD_CONTROL and POSITION_CONTROL/CONTROL_MODE_POSITION_VELOCITY_PD,
        the final target velocity is computed using:
        kp*(erp*(desiredPosition-currentPosition)/dt)+currentVelocity+kd*(m_desiredVelocity - currentVelocity).
        See also examples/pybullet/examples/pdControl.py

    force : float, optional
        In POSITION_CONTROL and VELOCITY_CONTROL this is the maximum motor
        force used to reach the target value. In TORQUE_CONTROL this is the
        force / torque to be applied each simulation step.

    positionGain : float, optional
        See implementation note above.

    velocityGain : float, optional
        See implementation note above.

    maxVelocity : float, optional
        In POSITION_CONTROL this limits the velocity to a maximum.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def setJointMotorControlArray(bodyUniqueId, jointIndices, controlMode):
    """Set an array of motors control mode and desired target value.

    This is similar to :func:`~pybullet_api.setJointMotorControl2`, with
    jointIndices as a list, and optional targetPositions, targetVelocities,
    forces, kds and kps as lists. Using
    :func:`~pybullet_api.setJointMotorControlArray` has the benefit of lower
    calling overhead.
    Instead of making individual calls for each joint, you can pass arrays
    for all inputs to reduce calling overhead dramatically.

    :func:`~pybullet_api.setJointMotorControlArray` takes the same parameters
    as :func:`~pybullet_api.setJointMotorControl2`, except replacing integers
    and floats with lists.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id as returned from :func:`~pybullet_api.loadURDF` etc.

    jointIndices : list of int
        Link index in range [0..getNumJoints(bodyUniqueId) (note that link
        index == joint index)

    controlMode : int
        POSITION_CONTROL (which is in fact CONTROL_MODE_POSITION_VELOCITY_PD),
        VELOCITY_CONTROL, TORQUE_CONTROL and PD_CONTROL. (There is also
        experimental STABLE_PD_CONTROL for stable(implicit) PD control, which
        requires additional preparation. See humanoidMotionCapture.py and
        pybullet_envs.deep_mimc for STABLE_PD_CONTROL examples.)
        TORQUE_CONTROL will apply a torque instantly, so it only is effective
        when calling stepSimulation explicitly.

    targetPositions : list of float, optional
        In POSITION_CONTROL the targetValue is target position of the joint.

    targetVelocities : list of float, optional
        In VELOCITY_CONTROL and POSITION_CONTROL  the targetVelocity is the
        desired velocity of the joint, see implementation note below. Note
        that the targetVelocity is not the maximum joint velocity. In
        PD_CONTROL and POSITION_CONTROL/CONTROL_MODE_POSITION_VELOCITY_PD,
        the final target velocity is computed using:
        kp*(erp*(desiredPosition-currentPosition)/dt)+currentVelocity+kd*(m_desiredVelocity - currentVelocity).
        See also examples/pybullet/examples/pdControl.py

    forces : list of float, optional
        In POSITION_CONTROL and VELOCITY_CONTROL this is the maximum motor
        force used to reach the target value. In TORQUE_CONTROL this is the
        force / torque to be applied each simulation step.

    positionGains : list of float, optional
        See implementation note in :func:pybullet_api.setJointMotorControl2`.

    velocityGains : list of float, optional
        See implementation note in :func:pybullet_api.setJointMotorControl2`.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def setJointMotorControlMultiDof(
        bodyUniqueId, jointIndex, controlMode, targetPosition=None,
        targetVelocity=None, force=None, positionGain=None, velocityGain=None,
        maxVelocity=None, physicsClientId=0):
    """Set a single joint motor control mode and desired target value.

    This function is similar to :func:`~pybullet_api.setJointMotorControl2`,
    but it support the spherical (multiDof) joint. This is used for the
    deep_mimic environment (in pybullet_envs) and humanoidMotionCapture.py
    example. Instead of a single float, targetPosition, targetVelocity and
    force arguments accept a list of 1 float or list of 3 floats to support
    a spherical joint.

    There is no immediate state change, :func:`~pybullet_api.stepSimulation`
    will process the motors.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id as returned from :func:`~pybullet_api.loadURDF` etc.

    jointIndex : int
        Link index in range [0..getNumJoints(bodyUniqueId) (note that link
        index == joint index).

    controlMode : int
        POSITION_CONTROL (which is in fact CONTROL_MODE_POSITION_VELOCITY_PD),
        VELOCITY_CONTROL, TORQUE_CONTROL and PD_CONTROL. (There is also
        experimental STABLE_PD_CONTROL for stable(implicit) PD control, which
        requires additional preparation. See humanoidMotionCapture.py and
        pybullet_envs.deep_mimc for STABLE_PD_CONTROL examples.)

    targetPosition : list of 1 or 3 floats, optional
        In POSITION_CONTROL the target value is target position of the joint.

    targetVelocity : list of 1 or 3 floats, optional
        In VELOCITY_CONTROL and POSITION_CONTROL  the targetVelocity is the
        desired velocity of the joint. Note that the targetVelocity is not the
        maximum joint velocity. In PD_CONTROL and
        POSITION_CONTROL/CONTROL_MODE_POSITION_VELOCITY_PD, the final target
        velocity is computed using:
        kp*(erp*(desiredPosition-currentPosition)/dt)+currentVelocity+kd*(m_desiredVelocity - currentVelocity).
        See also
        `examples/pybullet/examples/pdControl.py <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/pdControl.py>`_

    force : list of 1 or 3 floats, optional
        In POSITION_CONTROL and VELOCITY_CONTROL this is the maximum motor
        force used to reach the target value. In TORQUE_CONTROL this is the
        force/torque to be applied each simulation step.

    positionGain : float, optional
        See implementation note in :func:pybullet_api.setJointMotorControl2`.

    velocityGain : float, optional
        See implementation note in :func:pybullet_api.setJointMotorControl2`.

    maxVelocity : float, optional
        In POSITION_CONTROL this limits the velocity to a maximum.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def setJointMotorControlMultiDofArray(
        bodyUniqueId, jointIndices, controlMode, targetPositions=None,
        targetVelocities=None, forces=None, positionGains=None,
        velocityGains=None, maxVelocities=None, physicsClientId=0):
    """Set control mode and desired target values for multiple motors.

    This is a more efficient version of
    :func:`~pybullet_api.setJointMotorControlMultiDof`, passing in multiple
    control targets to avoid/reduce calling overhead between Python and
    PyBullet C++ extension. See
    `humanoidMotionCapture.py <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/humanoidMotionCapture.py>`_
    for an example.

    There is no immediate state change, :func:`~pybullet_api.stepSimulation`
    will process the motors.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id as returned from :func:`~pybullet_api.loadURDF` etc.

    jointIndices : list of int
        Link indices in range [0..getNumJoints(bodyUniqueId) (note that link
        index == joint index).

    controlMode : int
        POSITION_CONTROL (which is in fact CONTROL_MODE_POSITION_VELOCITY_PD),
        VELOCITY_CONTROL, TORQUE_CONTROL and PD_CONTROL. (There is also
        experimental STABLE_PD_CONTROL for stable(implicit) PD control, which
        requires additional preparation. See humanoidMotionCapture.py and
        pybullet_envs.deep_mimc for STABLE_PD_CONTROL examples.)

    targetPositions : list of floats, optional
        In POSITION_CONTROL the target value is target position of the joint.

    targetVelocities : list of floats, optional
        In VELOCITY_CONTROL and POSITION_CONTROL  the targetVelocity is the
        desired velocity of the joint. Note that the targetVelocity is not the
        maximum joint velocity. In PD_CONTROL and
        POSITION_CONTROL/CONTROL_MODE_POSITION_VELOCITY_PD, the final target
        velocity is computed using:
        kp*(erp*(desiredPosition-currentPosition)/dt)+currentVelocity+kd*(m_desiredVelocity - currentVelocity).
        See also
        `examples/pybullet/examples/pdControl.py <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/pdControl.py>`_

    forces : list of floats, optional
        In POSITION_CONTROL and VELOCITY_CONTROL this is the maximum motor
        force used to reach the target value. In TORQUE_CONTROL this is the
        force/torque to be applied each simulation step.

    positionGains : list of floats, optional
        See implementation note in :func:pybullet_api.setJointMotorControl2`.

    velocityGains : list of floats, optional
        See implementation note in :func:pybullet_api.setJointMotorControl2`.

    maxVelocities : list of floats, optional
        In POSITION_CONTROL this limits the velocity to a maximum.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def getJointState(bodyUniqueId, jointIndex, physicsClientId=0):
    """Get the state (position, velocity etc) for a joint on a body.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id as returned from :func:`~pybullet_api.loadURDF` etc.

    jointIndex : int
        Link index in range [0, pybullet.getNumJoints(bodyUniqueId)]

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    jointPosition : float
        The position value of this joint.

    jointVelocity : float
        The velocity value of this joint.

    jointReactionForces : array-like, shape (6,)
        These are the joint reaction forces, if a torque sensor is enabled for
        this joint it is [Fx, Fy, Fz, Mx, My, Mz]. Without torque sensor, it
        is [0,0,0,0,0,0].

    appliedJointMotorTorque : float
        This is the motor torque applied during the last
        :func:`~pybullet_api.stepSimulation`. Note that this only applies in
        VELOCITY_CONTROL and POSITION_CONTROL. If you use TORQUE_CONTROL then
        the applied joint motor torque is exactly what you provide, so there
        is no need to report it separately.
    """


def getJointStates(bodyUniqueId, jointIndices, physicsClientId=0):
    """Get the state (position, velocity etc) for multiple joints on a body.

    This is the array version of :func:`~pybullet_api.getJointState`. Instead
    of passing in a single jointIndex, you pass in a list of jointIndices.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id as returned from :func:`~pybullet_api.loadURDF` etc.

    jointIndices : list of int
        Link indices in range [0, pybullet.getNumJoints(bodyUniqueId)]

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    A list with the following values for each entry:

    jointPosition : float
        The position value of this joint.

    jointVelocity : float
        The velocity value of this joint.

    jointReactionForces : array-like, shape (6,)
        These are the joint reaction forces, if a torque sensor is enabled for
        this joint it is [Fx, Fy, Fz, Mx, My, Mz]. Without torque sensor, it
        is [0,0,0,0,0,0].

    appliedJointMotorTorque : float
        This is the motor torque applied during the last
        :func:`~pybullet_api.stepSimulation`. Note that this only applies in
        VELOCITY_CONTROL and POSITION_CONTROL. If you use TORQUE_CONTROL then
        the applied joint motor torque is exactly what you provide, so there
        is no need to report it separately.
    """


def resetJointState(bodyUniqueId, jointIndex, targetValue, targetVelocity, physicsClientId=0):
    """Reset the state (position, velocity etc) for a joint on a body instantaneously, not through physics simulation.

    You can reset the state of the joint. It is best only to do this at
    the start, while not running the simulation: resetJointState
    overrides all physics simulation. Note that we only support 1-DOF
    motorized joints at the moment, sliding joint or revolute joints.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id as returned from :func:`~pybullet_api.loadURDF` etc.

    jointIndex : int
        Joint index in range [0..getNumJoints(bodyUniqueId)].

    targetValue : float
        The joint position (angle in radians or position)

    targetVelocity : float
        The joint velocity (angular or linear velocity)

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def getJointStateMultiDof(bodyUniqueId, jointIndex, physicsClientId=0):
    """Get the state (position, velocity etc) for a joint on a body.

    This function supports planar and spherical joints.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id as returned from :func:`~pybullet_api.loadURDF` etc.

    jointIndex : int
        Joint index in range [0..getNumJoints(bodyUniqueId)].

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    jointPosition : list of 1 or 4 floats
        The position value of this joint (as joint angle / position or joint
        orientation quaternion).

    jointVelocity : list of 1 or 3 floats
        The velocity value of this joint.

    jointReactionForces : list of 6 floats
        These are the joint reaction forces, if a torque sensor is enabled for
        this joint it is [Fx, Fy, Fz, Mx, My, Mz]. Without torque sensor, it is
        [0,0,0,0,0,0].

    appliedJointMotorTorque : float
        This is the motor torque applied during the last
        :func:`~pybullet_api.stepSimulation`. Note that this only applies in
        VELOCITY_CONTROL and POSITION_CONTROL. If you use TORQUE_CONTROL then
        the applied joint motor torque is exactly what you provide, so there is
        no need to report it separately.
    """


def getJointStatesMultiDof(bodyUniqueId, jointIndex, physicsClientId=0):
    """Get the states (position, velocity etc) for multiple joint on a body.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id as returned from :func:`~pybullet_api.loadURDF` etc.

    jointIndex : list of int
        Joint indices in range [0..getNumJoints(bodyUniqueId)].

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    jointPositions : list of list of 1 or 4 floats
        The position values of these joints (as joint angle / position or joint
        orientation quaternion).

    jointVelocities : list of list of 1 or 3 floats
        The velocity values of these joints.

    jointReactionForces : list of list of 6 floats
        These are the joint reaction forces, if a torque sensor is enabled for
        these joints it is [Fx, Fy, Fz, Mx, My, Mz]. Without torque sensor, it
        is [0,0,0,0,0,0].

    appliedJointMotorTorque : list of float
        These are the motor torques applied during the last
        :func:`~pybullet_api.stepSimulation`. Note that this only applies in
        VELOCITY_CONTROL and POSITION_CONTROL. If you use TORQUE_CONTROL then
        the applied joint motor torque is exactly what you provide, so there is
        no need to report it separately.
    """


def resetJointStateMultiDof(bodyUniqueId, jointIndex, targetValue, targetVelocity=0, physicsClientId=0):
    """Reset the state (position, velocity etc) for a joint on a body instantaneously, not through physics simulation.

    See `humanoidMotionCapture <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/humanoidMotionCapture.py>`_
    for an example of this function. There is also
    :func:`~pybullet_api.resetJointStatesMultiDof` to reset multiple joints at
    a time.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id as returned from :func:`~pybullet_api.loadURDF` etc.

    jointIndex : int
        Joint index in range [0..getNumJoints(bodyUniqueId)].

    targetValue : list of 1 or 4 floats
        The position value of this joint (as joint angle / position or joint
        orientation quaternion).

    targetVelocity : list of 1 or 3 floats
        The velocity value of this joint.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def resetJointStatesMultiDof(bodyUniqueId, jointIndices, targetValues, targetVelocities=0, physicsClientId=0):
    """Reset the states (position, velocity etc) for multiple joints on a body instantaneously, not through physics simulation.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id as returned from :func:`~pybullet_api.loadURDF` etc.

    jointIndices : list of int
        Joint indices in range [0..getNumJoints(bodyUniqueId)].

    targetValues : list of list of 1 or 4 floats
        The position values of these joints (as joint angle / position or joint
        orientation quaternion).

    targetVelocities : list of list of 1 or 3 floats
        The velocity values of these joints.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def enableJointForceTorqueSensor(bodyUniqueId, jointIndex, enableSensor, physicsClientId=0):
    """Enable or disable a joint force/torque sensor measuring the joint reaction forces.

    You can enable or disable a joint force/torque sensor in each joint.
    Once enabled, if you perform a :func:`~pybullet_api.stepSimulation`,
    the :func:~pybullet_api.getJointState` will report the joint reaction
    forces in the fixed degrees of freedom: a fixed joint will measure all
    6DOF joint forces/torques. A revolute/hinge joint force/torque sensor
    will measure 5DOF reaction forces along all axis except the hinge axis.
    The applied force by a joint motor is available in the
    appliedJointMotorTorque of :func:`~pybullet_api.getJointState`.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id

    jointIndex : int
        Joint index in range [0, pybullet.getNumJoints(bodyUniqueId)]

    enableSensor : int
        1/True to enable, 0/False to disable the force/torque sensor

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def getLinkState(bodyUniqueId, linkIndex, computeLinkVelocity=0, computeForwardKinematics=0, physicsClientId=0):
    """Provides extra information such as the Cartesian world coordinates center of mass (COM) of the link, relative to the world reference frame.

    You can also query the Cartesian world position and orientation for the
    center of mass of each link using this function. It will also report the
    local inertial frame of the center of mass to the URDF link frame, to make
    it easier to compute the graphics/visualization frame.

    The relationship between URDF link frame and the center of mass frame
    (both in world space) is:
    urdfLinkFrame = comLinkFrame * localInertialFrame.inverse().
    For more information about the link and inertial frame, see the
    `ROS URDF tutorial <http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model>`_.

    .. image:: ../_static/link_frames.png
        :alt: Link frames
        :width: 25%
        :align: center

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id as returned by :func:`~pybullet_api.loadURDF` etc.

    linkIndex : int
        Link index.

    computeLinkVelocity : int, optional
        If set to 1, the Cartesian world velocity will be computed and
        returned.

    computeForwardKinematics : int, optional
        If set to 1 (or True), the Cartesian world position/orientation will
        be recomputed using forward kinematics.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    linkWorldPosition : array-like, shape (3,)
        Cartesian position of center of mass

    linkWorldOrientation : array-like, shape (4,)
        Cartesian orientation of center of mass, in quaternion [x,y,z,w]

    localInertialFramePosition : array-like, shape (3,)
        Local position offset of inertial frame (center of mass) expressed in
        the URDF link frame.

    localInertialFrameOrientation : array-like, shape (4,)
        Local orientation (quaternion [x,y,z,w]) offset of the inertial frame
        expressed in URDF link frame.

    worldLinkFramePosition : array-like, shape (3,)
        World position of the URDF link frame.

    worldLinkFrameOrientation : array-like, shape (4,)
        World orientation of the URDF link frame.

    worldLinkLinearVelocity : array-like, shape (3,)
        Cartesian world velocity. Only returned if computeLinkVelocity non-zero.

    worldLinkAngularVelocity : array-like, shape (3,)
        Cartesian world velocity. Only returned if computeLinkVelocity non-zero.
    """


def getLinkStates(bodyUniqueId, linkIndices, computeLinkVelocity=0, computeForwardKinematics=0, physicsClientId=0):
    """Same as :func:`~pybullet_api.getLinkState` except it takes a list of linkIndices.

    This function will return the information for multiple links. Instead of
    linkIndex it will accept linkIndices as a list of int. This can improve
    performance by reducing calling overhead of multiple calls.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id as returned by :func:`~pybullet_api.loadURDF` etc.

    linkIndices : list of int
        Link index.

    computeLinkVelocity : int, optional
        If set to 1, the Cartesian world velocity will be computed and
        returned.

    computeForwardKinematics : int, optional
        If set to 1 (or True), the Cartesian world position/orientation will
        be recomputed using forward kinematics.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    A list of tuples with the following entries:

    linkWorldPosition : array-like, shape (3,)
        Cartesian position of center of mass

    linkWorldOrientation : array-like, shape (4,)
        Cartesian orientation of center of mass, in quaternion [x,y,z,w]

    localInertialFramePosition : array-like, shape (3,)
        Local position offset of inertial frame (center of mass) expressed in
        the URDF link frame.

    localInertialFrameOrientation : array-like, shape (4,)
        Local orientation (quaternion [x,y,z,w]) offset of the inertial frame
        expressed in URDF link frame.

    worldLinkFramePosition : array-like, shape (3,)
        World position of the URDF link frame.

    worldLinkFrameOrientation : array-like, shape (4,)
        World orientation of the URDF link frame.

    worldLinkLinearVelocity : array-like, shape (3,)
        Cartesian world velocity. Only returned if computeLinkVelocity non-zero.

    worldLinkAngularVelocity : array-like, shape (3,)
        Cartesian world velocity. Only returned if computeLinkVelocity non-zero.
    """


def getBaseVelocity(bodyUniqueId, physicsClientId=0):
    """Get the linear and angular velocity of the base of the object in world space coordinates.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id, as returned from the load methods.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    linVel : array-like, shape (3,)
        Linear velocity [x,y,z] in world coordinates.

    angVel : array-like, shape (3,)
        Angular velocity [wx, wy, wz] in world coordinates.
    """


def resetBaseVelocity(objectUniqueId, linearVelocity=None, angularVelocity=None, physicsClientId=0):
    """Reset the linear and/or angular velocity of the base of the object in world space coordinates.

    Parameters
    ----------
    objectUniqueId : int
        Body unique id, as returned from the load methods.

    linearVelocity : array-like, shape (3,), optional
        Linear velocity [x,y,z] in Cartesian world coordinates.

    angularVelocity : array-like, shape (3,), optional
        Angular velocity [wx,wy,wz] in Cartesian world coordinates.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def applyExternalForce(objectUniqueId, linkIndex, forceObj, posObj, flags,
                       physicsClientId=0):
    """Apply external force.

    Note that this method will only work when explicitly stepping the
    simulation using :func:`~pybullet_api.stepSimulation`. In other words, we
    assume `setRealTimeSimulation(0)`. After each simulation step the external
    forces are cleared to zero. If you are using `setRealTimeSimulation(1)`,
    :func:`~pybullet_api.applyExternalForce` will have undefined behavior
    (either 0, 1 or multiple force applications).

    Parameters
    ----------
    objectUniqueId : int
        Object unique id as returned by load methods.

    linkIndex : int
        Link index or -1 for the base.

    forceObj : array-like, shape (3,)
        Force vector to be applied [x,y,z]. See flags for coordinate system.

    posObj : array-like, shape (3,)
        Position on the link where the force is applied.
        See flags for coordinate system.

    flags : int
        Specify the coordinate system of force/position: either WORLD_FRAME
        for Cartesian world coordinates or LINK_FRAME for local link
        coordinates.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def applyExternalTorque(objectUniqueId, linkIndex, torqueObj, flags,
                        physicsClientId=0):
    """Apply external torque.

    Note that this method will only work when explicitly stepping the
    simulation using :func:`~pybullet_api.stepSimulation`. In other words, we
    assume `setRealTimeSimulation(0)`. After each simulation step the external
    torques are cleared to zero. If you are using `setRealTimeSimulation(1)`,
    :func:`~pybullet_api.applyExternalTorque` will have undefined behavior
    (either 0, 1 or multiple torque applications).

    Parameters
    ----------
    objectUniqueId : int
        Object unique id as returned by load methods.

    linkIndex : int
        Link index or -1 for the base.

    torqueObj : array-like, shape (3,)
        Torque vector to be applied [x,y,z]. See flags for coordinate system.

    flags : int
        Specify the coordinate system of torque: either WORLD_FRAME
        for Cartesian world coordinates or LINK_FRAME for local link
        coordinates.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def getNumBodies(physicsClientId=0):
    """Get the number of bodies in the simulation.

    This function will return the total number of bodies in the physics
    server. If you use this function you can query the body unique ids using
    :func:`~pybullet_api.getBodyUniqueId`. Note that all APIs already return
    body unique ids, so you typically never need to use that function if you
    keep track of them.

    Parameters
    ----------
    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    numBodies : int
        Number of bodies in the simulation.
    """


def getBodyInfo(bodyUniqueId):
    """Get the body info, given a body unique id.

    Will return the base name, as extracted from the URDF, SDF, MJCF or
    other file.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    baseName : str
        Name of the base link.

    bodyName : str
        Name of the body.
    """


def getBodyUniqueId(serialIndex, physicsClientId=0):
    """This is used after connecting to server with existing bodies.

    :func:`~pybullet_api.getNumBodies` will return the total number of bodies
    in the physics server.

    If you used :func:`~pybullet_api.getNumBodies` you can query the body
    unique ids using this function. Note that all APIs already return body
    unique ids, so you typically never need to use this function if you keep
    track of them.

    Parameters
    ----------
    serialIndex : int
        Index of the body.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    bodyUniqueId : int
        Body unique id.
    """


def removeBody(bodyUniqueId, physicsClientId=0):
    """Remove a body by its body unique id.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def syncBodyInfo(physicsClientId=0):
    """Update body and constraint/joint information, in case other clients made changes.

    This will synchronize the body information
    (:func:`~pybullet_api.getBodyInfo`) in case of multiple clients connected
    to one physics server changing the world (:func:`~pybullet_api.loadURDF`,
    :func:`~pybullet_api.removeBody` etc.).

    Parameters
    ----------
    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def createConstraint(
        parentBodyUniqueId, parentLinkIndex, childBodyUniqueId,
        childLinkIndex, jointType, jointAxis, parentFramePosition=None,
        childFramePosition=None, physicsClientId=0):
    """Create a constraint between two bodies.

    URDF, SDF and MJCF specify articulated bodies are a tree-structures
    without loops. This function allows you to connect specific links of
    bodies to close those loops. See
    `Bullet/examples/pybullet/examples/quadruped.py
    <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/quadruped.py>`_
    how to connect the legs of a quadruped 5-bar closed loop linkage. In
    addition, you can create arbitrary constraints between objects, and
    between an object and a specific world frame. See
    `Bullet/examples/pybullet/examples/constraint.py
    <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/constraint.py>`_
    for an example.

    It can also be used to control the motion of physics objects, driven by
    animated frames, such as a VR controller. It is better to use constraints,
    instead of setting the position or velocity directly for such purpose,
    since those constraints are solved together with other dynamics
    constraints.

    Parameters
    ----------
    parentBodyUniqueId : int
        Parent body unique id.

    parentLinkIndex : int
        Parent link index (or -1 for the base).

    childBodyUniqueId : int
        Child body unique id, or -1 for no body (specify a non-dynamic
        child frame in world coordinates).

    childLinkIndex : int
        Child link index, or -1 for the base.

    jointType : int
        Joint type: JOINT_PRISMATIC, JOINT_FIXED, JOINT_POINT2POINT, JOINT_GEAR

    jointAxis : array-like, shape (3,)
        Joint axis, in child link frame.

    parentFramePosition : array-like, shape (3,)
        Position of the joint frame relative to parent center of mass frame.

    childFramePosition : array-like, shape (3,)
        Position of the joint frame relative to a given child center of mass
        frame (or world origin if no child specified).

    parentFrameOrientation : array-like, shape (4,), optional
        The orientation of the joint frame relative to parent center of mass
        coordinate frame.

    childFrameOrientation : array-like, shape (4,), optional
        The orientation of the joint frame relative to the child center of
        mass coordinate frame (or world origin frame if no child specified).

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    userConstraintUniqueId : int
        Integer unique id, that can be used to change or remove the constraint.
        See examples/pybullet/examples/mimicJointConstraint.py for an example
        of a JOINT_GEAR and examples/pybullet/examples/minitaur.py for a
        JOINT_POINT2POINT and examples/pybullet/examples/constraint.py for
        JOINT_FIXED.
    """


def changeConstraint(userConstraintUniqueId):
    """Change some parameters of an existing constraint, such as the child pivot.

    Examples
    --------

    See also Bullet/examples/pybullet/examples/constraint.py

    Parameters
    ----------
    userConstraintUniqueId : int
        Unique id as returned by :func:`~pybullet_api.createConstraint`.

    jointChildPivot : array-like, shape (3,), optional
        Updated child pivot, see :func:`~pybullet_api.createConstraint`.

    jointChildFrameOrientation : array-like, shape (4,), optional
        Updated child frame orientation as quaternion.

    maxForce : float, optional
        Maximum force that constraint can apply.

    gearRatio : float, optional
        The ratio between the rates at which the two gears rotate.

    gearAuxLink : int
        In some cases, such as a differential drive, a third (auxilary)
        link is used as reference pose. See `racecar_differential.py
        <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/racecar_differential.py>`_

    relativePositionTarget : float
        The relative position target offset between two gears.

    erp : float
        Constraint error reduction parameter.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def removeConstraint(userConstraintUniqueId, physicsClientId=0):
    """Remove a constraint using its unique id.

    Parameters
    ----------
    userConstraintUniqueId : int
        Unique id as returned by :func:`~pybullet_api.createConstraint`.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def getNumConstraints(physicsClientId=0):
    """Get the number of user-created constraints in the simulation.

    Parameters
    ----------
    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    numConstraints : int
        Total number of constraints created using
        :func:`~pybullet_api.createConstraint`.
    """


def getConstraintUniqueId(serialIndex, physicsClientId=0):
    """Get the unique id of the constraint.

    This function will take a serial index in range 0..getNumConstraints,
    and reports the constraint unique id. Note that the constraint unique
    ids may not be contiguous, since you may remove constraints. The
    input is the integer serial index and optionally a physicsClientId.

    Parameters
    ----------
    serialIndex : int
        An index in range [0.. number of constraints).

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    userConstraintUniqueId : int
        Unique id as returned by :func:`~pybullet_api.createConstraint`.
    """


def getConstraintInfo(constraintUniqueId, physicsClientId=0):
    """Get the user-created constraint info given a constraint unique id.

    Parameters
    ----------
    constraintUniqueId : int
        Unique id as returned by :func:`~pybullet_api.createConstraint`.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    parentBodyUniqueId : int
        See :func:`~pybullet_api.createConstraint`.

    parentJointIndex : int
        See :func:`~pybullet_api.createConstraint`.

    childBodyUniqueId : int
        See :func:`~pybullet_api.createConstraint`.

    childLinkIndex : int
        See :func:`~pybullet_api.createConstraint`.

    constraintType : int
        See :func:`~pybullet_api.createConstraint`.

    jointAxis : array-like, shape (3,)
        See :func:`~pybullet_api.createConstraint`.

    jointPivotInParent : array-like, shape (3,)
        See :func:`~pybullet_api.createConstraint`.

    jointPivotInChild : array-like, shape (3,)
        See :func:`~pybullet_api.createConstraint`.

    jointFrameOrientationParent : array-like, shape (4,)
        See :func:`~pybullet_api.createConstraint`.

    jointFrameOrientationChild : array-like, shape (4,)
        See :func:`~pybullet_api.createConstraint`.

    maxAppliedForce : float
        See :func:`~pybullet_api.createConstraint`.

    gearRatio : float
        See :func:`~pybullet_api.createConstraint`.

    gearAuxLink : int
        See :func:`~pybullet_api.createConstraint`.

    relativePositionTarget : float
        See :func:`~pybullet_api.createConstraint`.

    erp : float
        See :func:`~pybullet_api.createConstraint`.
    """


def getConstraintState(constraintUniqueId, physicsClientId=0):
    """Get the user-created constraint state (applied forces).

    Give a constraint unique id, you can query for the applied constraint
    forces in the most recent simulation step. The input is a constraint
    unique id and the output is a vector of constraint forces, its
    dimension is the degrees of freedom that are affected by the constraint
    (a fixed constraint affects 6 DoF for example).

    Parameters
    ----------
    constraintUniqueId : int
        Unique id as returned by :func:`~pybullet_api.createConstraint`.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    constraintForces : array-like, shape (n_affected_dof,)
        Constraint forces.
    """


def getDynamicsInfo(bodyUniqueId, linkIndex, physicsClientId=0):
    """Get dynamics information such as mass, lateral friction coefficient.

    You can get information about the mass, center of mass, friction and other
    properties of the base and links.

    Parameters
    ----------
    bodyUniqueId : int
        Object unique id, as returned by loadURDF etc.

    linkIndex : int
        Link (joint) index or -1 for the base.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    mass : float
        Mass in kg

    lateralFriction : float
        Friction coefficient

    localInertiaDiagnoal : array-like, shape (3,)
        Local inertia diagonal. Note that links and base are centered around
        the center of mass and aligned with the principal axes of inertia.

    localInertialPos : array-like, shape (3,)
        Position of inertial frame in local coordinates of the joint frame

    localInertialOrn : array-like, shape (4,)
        Orientation of inertial frame in local coordinates of joint frame

    restitution : float
        Coefficient of restitution

    rollingFriction : float
        Rolling friction coefficient orthogonal to contact normal

    spinningFriction : float
        Spinning friction coefficient around contact normal

    contactDamping : float
        -1 if not available. damping of contact constraints.

    contactStiffness : float
        -1 if not available. stiffness of contact constraints.

    bodyType : int
        1: rigid body, 2: multi body, 3: soft body

    collisionMargin : float
        Advanced/internal/unsupported info. Collision margin of the collision
        shape. Collision margins depend on the shape type. It is not
        consistent.
    """


def changeDynamics(bodyUniqueId, linkIndex):
    """Change dynamics information such as mass, lateral friction coefficient.

    You can change the properties such as mass, friction and restitution
    coefficients.

    Parameters
    ----------
    bodyUniqueId : int
        Object unique id, as returned by loadURDF etc.

    linkIndex : int
        Link (joint) index or -1 for the base.

    mass : float, optional
        Mass in kg. Change the mass of the link (or base).

    lateralFriction : float, optional
        Lateral (linear) contact friction.

    spinningFriction : float, optional
        Torsional friction around the contact normal.

    rollingFriction : float, optional
        Torsional friction orthogonal to contact normal (keep this value very
        close to zero, otherwise the simulation can become very unrealistic.

    restitution : float, optional
        Bouncyness of contact. Keep it a bit less than 1, preferably closer
        to 0. The value is initialized to 0 by Bullet if it is not specified
        explicitly.

    linearDamping : float, optional (default: 0.04)
        Linear damping of the link.

    angularDamping : float, optional (default: 0.04)
        Angular damping of the link.

    contactStiffness : float, optional
        Stiffness of the contact constraints, used together with contactDamping.
        The value is initialized to 1e18 (float) or 1e30 (double) by Bullet if
        it is not specified explicitly.

    contactDamping : float, optional
        Damping of the contact constraints for this body/link. Used together
        with contactStiffness. This overrides the value if it was specified in
        the URDF file in the contact section.
        The value is initialized to 0.1 by Bullet if it is not specified
        explicitly.

    frictionAnchor : int, optional
        Enable or disable a friction anchor: friction drift correction
        (disabled by default, unless set in the URDF contact section).

    localInertiaDiagnoal : array-like, shape (3,), optional
        Diagonal elements of the inertia tensor. Note that the base and links
        are centered around the center of mass and aligned with the principal
        axes of inertia so there are no off-diagonal elements in the inertia
        tensor.

    ccdSweptSphereRadius : float, optional
        Radius of the sphere to perform continuous collision detection. See
        Bullet/examples/pybullet/examples/experimentalCcdSphereRadius.py for
        an example.

    contactProcessingThreshold : float, optional
        Contacts with a distance below this threshold will be processed by the
        constraint solver. For example, if contactProcessingThreshold = 0,
        then contacts with distance 0.01 will not be processed as a constraint.

    activationState : int, optional
        When sleeping is enabled, objects that don't move (below a threshold)
        will be disabled as sleeping, if all other objects that influence it
        are also ready to sleep. Options: pybullet.ACTIVATION_STATE_SLEEP,
        pybullet.ACTIVATION_STATE_ENABLE_SLEEPING,
        pybullet.ACTIVATION_STATE_DISABLE_WAKEUP. You can also use
        flags = pybullet.URDF_ENABLE_SLEEPING in loadURDF to enable sleeping.
        See sleeping.py example.

    jointDamping : float, optional
        Joint damping coefficient applied at each joint. This coefficient is
        read from URDF joint damping field. Keep the value close to 0.
        Joint damping force = -damping_coefficient * joint_velocity.

    anisotropicFriction : float, optional
        AnisotropicFriction coefficient to allow scaling of friction in
        different directions.

    maxJointVelocity : float, optional
        Maximum joint velocity for a given joint, if it is exceeded during
        constraint solving, it is clamped. Default maximum joint velocity is
        100 units.

    collisionMargin : float, optional
        Unsupported. change the collision margin. dependent on the shape type,
        it may or may not add some padding to the collision shape.

    jointLowerLimit : float, optional
        Change the lower limit of a joint, also requires jointUpperLimit
        otherwise it is ignored. NOTE that at the moment, the joint limits
        are not updated in 'getJointInfo'!

    jointUpperLimit : float, optional
        Change the upper limit of a joint, also requires jointLowerLimit
        otherwise it is ignored. NOTE that at the moment, the joint limits
        are not updated in 'getJointInfo'!

    jointLimitForce : float, optional
        Change the maximum force applied to satisfy a joint limit.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


###############################################################################
# Deformables and Cloth (FEM, PBD)
###############################################################################


def loadSoftBody(
        basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1], scale=1, mass=None,
        collisionMargin=None,
        useMassSpring=None, useBendingSprings=None, useNeoHookean=None,
        springElasticStiffness=None,
        springDampingStiffness=None,
        springDampingAllDirections=None,
        springBendingStiffness=None,
        NeoHookeanMu=None, NeoHookeanLambda=None, NeoHookeanDamping=None,
        frictionCoeff=None, useFaceContact=None, useSelfCollision=None,
        repulsionStiffness=None,
        physicsClientId=0):
    """Load a softbody from an obj file.

    Note that several parameters assume you use the FEM model and have no
    effect for PBD simulation.

    You have to call

    .. code-block:: python

        pybullet.resetSimulation(pybullet.RESET_USE_DEFORMABLE_WORLD)

    if you want to use soft bodies.

    You can access the vertices of a deformable using :func:`~pybullet_api.getMeshData`.

    Parameters
    ----------
    basePosition : array-like, shape (3,), optional (default: [0, 0, 0])
        Initial position of the deformable object

    baseOrientation : array-like, shape (4,), optional (default: [0, 0, 0, 1])
        Initial orientation (quaternion x,y,z,w) of the deformable object

    scale : float, optional (default: 1)
        Scaling factor to resize the deformable

    mass : float, optional (default: TODO)
        Total mass of the deformable, the mass is equally distributed among
        all vertices

    collisionMargin : float, optional (default: TODO)
        A collision margin extends the deformable, it can help avoiding
        penetrations, especially for thin (cloth) deformables

    useMassSpring : bool, optional (default: False)
        Using mass spring

    useBendingSprings : bool, optional (default: False)
        Create bending springs to control bending of deformables

    useNeoHookean : bool, optional (default: False)
        Enable the Neo Hookean simulation

    springElasticStiffness : float, optional (default: 1)
        Stiffness parameter

    springDampingStiffness : float, optional (default: 0.1)
        Damping parameter

    springDampingAllDirections : bool, optional (default: False)
        Spring damping parameter

    springBendingStiffness : float, optional (default: 0.1)
        Parameters of bending stiffness

    NeoHookeanMu : float, optional (default: 1)
        Parameters of the Neo Hookean model

    NeoHookeanLambda : float, optional (default: 1)
        Parameters of the Neo Hookean model

    NeoHookeanDamping : float, optional (default: 0.1)
        Parameters of the Neo Hookean model

    frictionCoeff : float, optional (default: 0)
        Contact friction for deformables

    useFaceContact : bool, optional (default: False)
        Enable collisions internal to faces, not just at vertices

    useSelfCollision : bool, optional (default: False)
        Enable self collision for a deformable

    repulsionStiffness : float, optional (default: 0.5)
        A parameter that helps avoiding penetration

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    softBodyBodyUniqueId : int
        ID of the loaded soft body
    """


def createSoftBodyAnchor(softBodyBodyUniqueId, nodeIndex, bodyUniqueId=-1, linkIndex=-1, bodyFramePosition=(0, 0, 0), physicsClientId=0):
    """Create an anchor (attachment) between a soft body and a rigid or multi body.

    You can pin vertices of a deformable object to the world, or attach a
    vertex of a deformable to a multi body with this function.
    This will return a constraint unique id. You can remove this constraint
    using :func:`~pybullet_api.removeConstraint`.

    See the `deformable_anchor.py <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/deformable_anchor.py>`_ for an example.

    Parameters
    ----------
    softBodyBodyUniqueId : int
        ID of the soft body

    nodeIndex : int
        Index of the node from the soft body that will be anchored.

    bodyUniqueId : int, optional
        ID of the body to which the soft body will be attached.
        Set to -1 if you want to attach it to the world.

    linkIndex : int, optional
        ID of the link of the body to which the soft body will be
        attached. Set to -1 if you want to attach it to the world
        or the origin of the body.

    bodyFramePosition : array-like, shape (3,), optional
        Position of the anchor point in the body frame.

    physicsClientId : int, optional (default: 0), optional
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    userConstraintUid : int
        Constraint unique ID.
    """


###############################################################################
# Synthetic Camera Rendering
###############################################################################


def computeViewMatrix(cameraEyePosition, cameraTargetPosition, cameraUpVector,
                      physicsClientId=0):
    """Compute a camera viewmatrix from camera eye, target position and up vector.

    Parameters
    ----------
    cameraEyePosition : array-like, shape (3,)
        Eye position in Cartesian world coordinates.

    cameraTargetPosition : array-like, shape (3,)
        Position of the target (focus) point, in Cartesian world coordinates.

    cameraUpVector : array-like, shape (3,)
        Up vector of the camera, in Cartesian world coordinates.

    physicsClientId : int, optional (default: 0)
        Unused, added for API consistency.

    Returns
    -------
    viewMatrix : list
        The 4x4 view matrix, stored as a list of 16 floats.
    """


def computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition, distance, yaw, pitch, roll, upAxisIndex,
        physicsClientId=0):
    """Compute a camera view matrix from camera eye, target position and up vector.

    Parameters
    ----------
    cameraTargetPosition : array-like, shape (3,)
        Target focus point in Cartesian world coordinates.

    distance : float
        Distance from eye to focus point.

    yaw : float
        Yaw angle in degrees left/right around up-axis.

    pitch : float
        Pitch in degrees up/down.

    roll : float
        Roll in degrees around forward vector.

    upAxisIndex : int
        Either 1 for Y or 2 for Z axis up.

    physicsClientId : int, optional (default: 0)
        Unused, added for API consistency.

    Returns
    -------
    viewMatrix : list
        The 4x4 view matrix, stored as a list of 16 floats.
    """


def computeProjectionMatrixFOV(fov, aspect, nearVal, farVal, physicsClientId=0):
    """Compute a camera projection matrix from fov, aspect ratio, near, far values.

    This command also will return a 4x4 projection matrix, using different
    parameters. You can check out OpenGL documentation for the meaning of the
    parameters.

    Parameters
    ----------
    fov : float
        Field of view.

    aspect : float
        Aspect ratio.

    nearVal : float
        Near plane distance.

    farVal : float
        Far plane distance.

    physicsClientId : int, optional (default: 0)
        Unused, added for API consistency.

    Returns
    -------
    projectionMatrix : list
        The 4x4 view matrix, stored as a list of 16 floats.
    """


def computeProjectionMatrix(left, right, bottom, top, near, far, physicsClientId=0):
    """Compute a camera projection matrix from screen left/right/bottom/top/near/far values.

    Parameters
    ----------
    left : float
        Left screen (canvas) coordinate.

    right : float
        Right screen (canvas) coordinate.

    bottom : float
        Bottom screen (canvas) coordinate.

    top : float
        Top screen (canvas) coordinate.

    near : float
        Near plane distance.

    far : float
        Far plane distance.

    physicsClientId : int, optional (default: 0)
        Unused, added for API consistency.

    Returns
    -------
    projectionMatrix : list
        The 4x4 projection matrix, stored as a list of 16 floats.
    """


def getCameraImage(width, height):
    """Render an image.

    The getCameraImage API will return a RGB image, a depth buffer and a
    segmentation mask buffer with body unique ids of visible objects for
    each pixel. Note that PyBullet can be compiled using the numpy option:
    using numpy will improve the performance of copying the camera pixels
    from C to Python. Note: the old renderImage API is obsolete and
    replaced by getCameraImage.

    Parameters
    ----------
    width : int
        Horizontal image resolution in pixels

    height : int
        Vertical image resolution in pixels

    viewMatrix : array-like, shape (16,)
        4x4 view matrix, see computeViewMatrix*

    projectionMatrix : array-like, shape (16,)
        4x4 projection matrix, see computeProjection*

    lightDirection : array-like, shape (3,)
        LightDirection specifies the world position of the light source, the
        direction is from the light source position to the origin of the
        world frame.

    lightColor : array-like, shape (3,)
        Directional light color in [RED,GREEN,BLUE] in range 0..1,  only
        applies to ER_TINY_RENDERER

    lightDistance : float
        Distance of the light along the normalized lightDirection, only
        applies to ER_TINY_RENDERER

    shadow : int
        1 for shadows, 0 for no shadows, only applies to ER_TINY_RENDERER

    lightAmbientCoeff : float
        Light ambient coefficient, only applies to ER_TINY_RENDERER

    lightDiffuseCoeff : float
        Light diffuse coefficient, only applies to ER_TINY_RENDERER

    lightSpecularCoeff : float
        Light specular coefficient, only applies to ER_TINY_RENDERER

    renderer : int
        ER_BULLET_HARDWARE_OPENGL or ER_TINY_RENDERER. Note that DIRECT mode
        has no OpenGL, so it requires ER_TINY_RENDERER.

    flags : int
        ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX, See below in description of
        segmentationMaskBuffer and example code. Use ER_NO_SEGMENTATION_MASK
        to avoid calculating the segmentation mask.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    width : int
        Width image resolution in pixels (horizontal)

    height : int
        Height image resolution in pixels (vertical)

    rgbPixels : list of [char RED,char GREEN,char BLUE, char ALPHA] [0..width*height]
        List of pixel colors in R,G,B,A format, in range [0..255] for each
        color

    depthPixels : list of float [0..width*height]
        Depth buffer. Bullet uses OpenGL to render, and the convention is
        non-linear z-buffer. See
        https://stackoverflow.com/questions/6652253/getting-the-true-z-value-from-the-depth-buffer

        far=1000.//depends on projection matrix, this is default
        near=0.01//depends on projection matrix
        depth = far * near / (far - (far - near) * depthImg)//depthImg is the depth from Bullet 'getCameraImage'

        See also PyBullet https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/pointCloudFromCameraImage.py

    segmentationMaskBuffer : list of int [0..width*height]
        For each pixels the visible object unique id. If
        ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX is used, the
        segmentationMaskBuffer combines the object unique id and link
        index as follows:
        value = objectUniqueId + (linkIndex+1)<<24.
        See `example <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/segmask_linkindex.py>`_.

        So for a free floating body without joints/links, the segmentation
        mask is equal to its body unique id, since its link index is -1.
    """


def isNumpyEnabled(physicsClientId=0):
    """Return True if PyBullet was compiled with NumPy support.

    This makes :func:`~pybullet_api.getCameraImage` faster.

    Parameters
    ----------
    physicsClientId : int, optional (default: 0)
        Unused, added for API consistency.

    Returns
    -------
    isNumpyEnabled : bool
        Whether PyBullet was compiled with NumPy enabled.
    """


def getVisualShapeData(objectUniqueId, flags=-1, physicsClientId=0):
    """Return the visual shape information for one object.

    You can access visual shape information using this function. You could use
    this to bridge your own rendering method with PyBullet simulation, and
    synchronize the world transforms manually after each simulation step. You
    can also use :func:`~pybullet_api.getMeshData`, in particular for
    deformable objects, to receive data about vertex locations.

    The physics simulation uses center of mass as a reference for the Cartesian
    world transforms, in :func:`~pybullet_api.getBasePositionAndOrientation`
    and in :func:`~pybullet_api.getLinkState`. If you implement your own
    rendering, you need to transform the local visual transform to world space,
    making use of the center of mass world transform and the (inverse)
    localInertialFrame. You can access the localInertialFrame using
    :func:`~pybullet_api.getLinkState`.

    Parameters
    ----------
    objectUniqueId : int
        Object unique id, as returned by a load method.

    flags : int, optional
        VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS will also provide textureUniqueId.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    objectUniqueId : int
        Ibject unique id, same as the input.

    linkIndex : int
        Link index or -1 for the base.

    visualGeometryType : int
        Visual geometry type (TBD).

    dimensions : array-like, shape (3,)
        Dimensions (size, local scale) of the geometry.

    meshAssetFileName : str
        Path to the triangle mesh, if any. Typically relative to the URDF,
        SDF or MJCF file location, but could be absolute.

    localVisualFramePosition : array-like, shape (3,)
        Position of local visual frame, relative to link/joint frame.

    localVisualFrameOrientation : array-like, shape (4,)
        Orientation of local visual frame relative to link/joint frame.

    rgbaColor : array-like, shape (4,)
        URDF color (if any specified) in red/green/blue/alpha.

    textureUniqueId : int
        Texture unique id of the shape, or -1 if none.
        Field only exists if using VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS flags.
    """


def changeVisualShape(
        objectUniqueId, linkIndex, shapeIndex=-1, textureUniqueId=-1,
        rgbaColor=None, specularColor=None, flags=-1, physicsClientId=0):
    """Change part of the visual shape information for one object.

    Parameters
    ----------
    objectUniqueId : int
        Object unique id, as returned by load method.

    linkIndex : int
        Link index.

    shapeIndex : int, optional
        Experimental for internal use, recommended ignore shapeIndex or leave
        it -1. Intention is to let you pick a specific shape index to modify,
        since URDF (and SDF etc) can have more than 1 visual shape per link.
        This shapeIndex matches the list ordering returned by
        :func:`~pybullet_api.getVisualShapeData`.

    textureUniqueId : int, optional
        Texture unique id as returned by 'loadTexture' method.

    rgbaColor : array-like, shape (4,), optional
        Color components for RED, GREEN, BLUE and ALPHA, each in range [0..1].
        Alpha has to be 0 (invisible) or 1 (visible) at the moment. Note that
        TinyRenderer doesn't support transparancy, but the GUI/EGL OpenGL3
        renderer does.

    specularColor : array-like, shape (3,), optional
        Specular color components, RED, GREEN and BLUE, can be from 0 to
        large number (>100).

    flags : int, optional
        Can be VISUAL_SHAPE_DOUBLE_SIDED to apply changes to both sides
        of this visual shape.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def loadTexture(textureFilename, physicsClientId=0):
    """Load texture file.

    Load a texture from file and return a non-negative texture unique id
    if the loading succeeds. This unique id can be used with
    :func:`~pybullet_api.changeVisualShape`.

    Parameters
    ----------
    textureFilename : str
        Texture file

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    textureUniqueId : int
        Non-negative texture unique id
    """


def getMeshData(bodyUniqueId, linkIndex=-1, collisionShapeIndex=-1, flags=-1, physicsClientId=0):
    """Get mesh data. Returns vertices etc from the mesh.

    getMeshData is an experimental API to return mesh information (vertices,
    indices) of triangle meshes.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique ID

    linkIndex : int, optional (default: -1)
        Link index, use -1 if there is only one.

    collisionShapeIndex : int, optional (default: -1)
        Index of compound shape, in case of multiple collision shapes in
        the link (see getCollisionShapeData)

    flags : int, optional (default: -1)
        By default, PyBullet will return graphics rendering vertices. Since
        vertices with different normals are duplicated, the can be more
        vertices than in the original mesh. You can receive the simulation
        vertices by using flags = pybullet.MESH_DATA_SIMULATION_MESH.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    n_vertices : int
        Number of vertices

    vertices : array-like, shape (n_vertices, 3)
        Vertices of the mesh
    """


def createCollisionShape(shapeType):
    """Create a collision shape.

    Although the recommended and easiest way to create stuff in the world is
    using the loading functions (loadURDF/SDF/MJCF/Bullet), you can also
    create collision and visual shapes programmatically and use them to create
    a multi body using :func:`~pybullet_api.createMultiBody`. See the
    `createMultiBodyLinks.py
    <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/createMultiBodyLinks.py>`_
    and `createVisualShape.py
    <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/createVisualShape.py>`_
    example in the Bullet Physics SDK.

    Parameters
    ----------
    shapeType : int
        Options: GEOM_SPHERE, GEOM_BOX, GEOM_CAPSULE, GEOM_CYLINDER,
        GEOM_PLANE, GEOM_MESH, GEOM_HEIGHTFIELD

    radius : float, optional (default: 0.5)
        For GEOM_SPHERE, GEOM_CAPSULE, GEOM_CYLINDER

    halfExtents : array-like, shape (3,), optional (default: [1, 1, 1])
        For GEOM_BOX

    height : float, optional (default: 1)
        For GEOM_CAPSULE, GEOM_CYLINDER

    fileName : str, optional
        Filename for GEOM_MESH, currently only Wavefront .obj. Will create
        convex hulls for each object (marked as 'o') in the .obj file.

    meshScale : array-like, shape (3,), optional (default: [1, 1, 1])
        For GEOM_MESH

    planeNormal : array-like, shape (3,), optional (default: [0, 0, 1])
        For GEOM_PLANE

    flags : int, optional
        GEOM_FORCE_CONCAVE_TRIMESH: for GEOM_MESH, this will create a concave
        static triangle mesh. This should not be used with dynamic / moving
        objects, only for static (mass = 0) terrain.

    collisionFramePosition : array-like, shape (3,), optional
        Translational offset of the collision shape with respect to the link
        frame.

    collisionFrameOrientation : array-like, shape (4,), optional
        Rotational offset (quaternion x,y,z,w) of the collision shape with
        respect to the link frame.

    vertices : array-like, shape (n_vertices, 3), optional
        Definition of a heightfield. See the
        `heightfield.py <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/heightfield.py>`_
        example.

    indices : list of int, optional
        Definition of a heightfield

    heightfieldTextureScaling : int, optional
        Texture scaling of a heightfield

    numHeightfieldRows : int, optional
        Definition of a heightfield

    numHeightfieldColumns : int, optional
        Definition of a heightfield

    replaceHeightfieldIndex : int, optional
        Replacing an existing heightfield (updating its heights) (much faster
        than removing and re-creating a heightfield).

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    uid : int
        The return value is a non-negative int unique id for the collision
        shape or -1 if the call failed.
    """


def createCollisionShapeArray(shapeTypes):
    """Create collision shapes. Returns a non-negative (int) unique id, if successfull, negative otherwise.

    This function is the array version of
    :func:`~pybullet_api.createCollisionShape`. For usage, see the snake.py
    or createVisualShapeArray.py examples on how to use it.

    Parameters
    ----------
    shapeTypes : list of int
        Options: GEOM_SPHERE, GEOM_BOX, GEOM_CAPSULE, GEOM_CYLINDER,
        GEOM_PLANE, GEOM_MESH, GEOM_HEIGHTFIELD

    radii : array-like, shape (n_shapes,), optional
        For GEOM_SPHERE, GEOM_CAPSULE, GEOM_CYLINDER

    halfExtents : array-like, shape (n_shapes, 3)
        For GEOM_BOX

    lengths : array-like, shape (n_shapes,), optional
        For GEOM_CAPSULE, GEOM_CYLINDER

    fileNames : list of str, optional
        Filenames for GEOM_MESH, currently only Wavefront .obj. Will create
        convex hulls for each object (marked as 'o') in the .obj file.

    meshScales : array-like, shape (n_shapes, 3), optional
        For GEOM_MESH

    planeNormals : array-like, shape (n_shapes, 3), optional
        For GEOM_PLANE

    flags : int, optional
        GEOM_FORCE_CONCAVE_TRIMESH: for GEOM_MESH, this will create a concave
        static triangle mesh. This should not be used with dynamic / moving
        objects, only for static (mass = 0) terrain.

    collisionFramePositions : array-like, shape (n_shapes, 3), optional
        Translational offset of the collision shape with respect to the link
        frame.

    collisionFrameOrientations : array-like, shape (n_shapes, 4), optional
        Rotational offset (quaternion x,y,z,w) of the collision shape with
        respect to the link frame.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    uid : int
        The return value is a non-negative int unique id for the collision
        shapes or -1 if the call failed.
    """


def removeCollisionShape(collisionShapeId, physicsClientId=0):
    """Remove a collision shape.

    This will remove an existing collision shape, using its collision shape
    unique id. Only useful when the collision shape is not used in a body
    (to perform a :func:`~pybullet_api.getClosestPoints` query).

    Parameters
    ----------
    collisionShapeId : int
        Collision shape

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def createVisualShape(shapeType):
    """Create a visual shape.

    You can create a visual shape in a similar way to creating a collision
    shape, with some additional arguments to control the visual appearance,
    such as diffuse and specular color. When you use the GEOM_MESH type,
    you can point to a Wavefront OBJ file, and the visual shape will parse
    some parameters from the material file (.mtl) and load a texture. Note
    that large textures (above 1024x1024 pixels) can slow down the loading
    and run-time performance.

    Examples
    --------

    See examples/pybullet/examples/addPlanarReflection.py and createVisualShape.py.

    .. image:: ../_static/10000201000007EC0000061A3D679493259030FC.png
        :alt: Example
        :width: 50%
        :align: center

    See `createVisualShape <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/createVisualShape.py>`_,
    `createVisualShapeArray <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/createVisualShapeArray.py>`_
    and `createTexturedMeshVisualShape <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/createTexturedMeshVisualShape.py>`_
    examples.

    Parameters
    ----------
    shapeType : int
        Options: GEOM_SPHERE, GEOM_BOX, GEOM_CAPSULE, GEOM_CYLINDER,
        GEOM_PLANE, GEOM_MESH, GEOM_HEIGHTFIELD

    radius : float, optional (default: 0.5)
        For GEOM_SPHERE, GEOM_CAPSULE, GEOM_CYLINDER

    halfExtents : array-like, shape (3,), optional (default: [1, 1, 1])
        For GEOM_BOX

    length : float, optional (default: 1)
        Only for GEOM_CAPSULE, GEOM_CYLINDER (length = height)

    fileName : str, optional
        Filename for GEOM_MESH, currently only Wavefront .obj. Will create
        convex hulls for each object (marked as 'o') in the .obj file.

    meshScale : array-like, shape (3,), optional (default: [1, 1, 1])
        For GEOM_MESH

    planeNormal : array-like, shape (3,), optional (default: [0, 0, 1])
        For GEOM_PLANE

    flags : int, optional
        GEOM_FORCE_CONCAVE_TRIMESH: for GEOM_MESH, this will create a concave
        static triangle mesh. This should not be used with dynamic / moving
        objects, only for static (mass = 0) terrain.

    rgbaColor : array-like, shape (4,)
        Color components for red, green, blue and alpha, each in range [0..1].

    specularColor : array-like, shape (3,)
        Specular reflection color, red, green, blue components in range [0..1].

    visualFramePosition : array-like, shape (3,), optional
        Translational offset of the visual shape with respect to the link
        frame.

    visualFrameOrientation : array-like, shape (4,), optional
        Rotational offset (quaternion x,y,z,w) of the visual shape with
        respect to the link frame.

    vertices : array-like, shape (n_vertices, 3), optional
        Instead of creating a mesh from obj file, you can provide vertices,
        indices, uvs and normals.

    indices : list of int, optional
        Triangle indices, should be a multiple of 3

    uvs : array-like, shape (n_uvs, 2)
        uv texture coordinates for vertices. Use
        :func:`~pybullet_api.changeVisualShape` to choose the texture image.
        The number of uvs should be equal to number of vertices.

    normals : array-like, shape (n_uvs, 3)
        Vertex normals, number should be equal to number of vertices.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    uid : int
        The return value is a non-negative int unique id for the collision
        shape or -1 if the call failed.
    """


def createVisualShapeArray(shapeTypes):
    """Create visual shapes. Returns a non-negative (int) unique id, if successfull, negative otherwise.

    This is the array version of :func:`~pybullet_api.createVisualShape`. See
    `createVisualShapeArray.py <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/createVisualShapeArray.py>`_
    example.

    Parameters
    ----------
    shapeTypes : list int
        Options: GEOM_SPHERE, GEOM_BOX, GEOM_CAPSULE, GEOM_CYLINDER,
        GEOM_PLANE, GEOM_MESH, GEOM_HEIGHTFIELD

    radii : array-like, shape (n_shapes,), optional
        For GEOM_SPHERE, GEOM_CAPSULE, GEOM_CYLINDER

    halfExtents : array-like, shape (n_shapes, 3), optional
        For GEOM_BOX

    lengths : array-like, shape (n_shapes,), optional
        Only for GEOM_CAPSULE, GEOM_CYLINDER (length = height)

    fileNames : list of str, optional
        Filename for GEOM_MESH, currently only Wavefront .obj. Will create
        convex hulls for each object (marked as 'o') in the .obj file.

    meshScales : array-like, shape (n_shapes, 3), optional
        For GEOM_MESH

    planeNormals : array-like, shape (n_shapes, 3), optional
        For GEOM_PLANE

    flags : int, optional
        GEOM_FORCE_CONCAVE_TRIMESH: for GEOM_MESH, this will create a concave
        static triangle mesh. This should not be used with dynamic / moving
        objects, only for static (mass = 0) terrain.

    rgbaColors : array-like, shape (n_shapes, 4)
        Color components for red, green, blue and alpha, each in range [0..1].

    visualFramePositions : array-like, shape (n_shapes, 3), optional
        Translational offsets of the visual shapes with respect to the link
        frame.

    visualFrameOrientations : array-like, shape (n_shapes, 4), optional
        Rotational offsets (quaternion x,y,z,w) of the visual shapes with
        respect to the link frame.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    uid : int
        The return value is a non-negative int unique id for the collision
        shape or -1 if the call failed.
    """


def stepSimulation(physicsClientId=0):
    """Step the simulation using forward dynamics.

    Performs all the actions in a single forward dynamics simulation step such
    as collision detection, constraint solving and integration. The default
    timestep is 1/240 second, it can be changed using
    :func:`~pybullet_api.setTimeStep` or
    :func:`~pybullet_api.setPhysicsEngineParameter`.

    For experimental/advanced use only: if reportSolverAnalytics is enabled
    through :func:`~pybullet_api.setPhysicsEngineParameter`, information is
    returned as a list of island information. Otherwise there are no return
    values.

    See also :func:`~pybullet_api.setRealTimeSimulation` to automatically let
    the physics server run forward dynamics simulation based on its real-time
    clock.

    Parameters
    ----------
    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    numBodies : lost of int, optional
        The body unique ids in this island.

    numIterationsUsed : int
        The number of solver iterations used.

    remainingResidual : float
        The residual constraint error.
    """

def setRealTimeSimulation(enableRealTimeSimulation, physicsClientId=0):
    """Enable or disable real time simulation (using the real time clock, RTC) in the physics server.

    By default, the physics server will not step the simulation, unless you
    explicitly send a 'stepSimulation' command. This way you can maintain
    control determinism of the simulation. It is possible to run the
    simulation in real-time by letting the physics server automatically step
    the simulation according to its real-time-clock (RTC) using this
    command. If you enable the real-time simulation, you don't need to call
    'stepSimulation'.

    Note that this has no effect in DIRECT mode: in DIRECT mode the physics
    server and client happen in the same thread and you trigger every
    command. In GUI mode and in Virtual Reality mode, and TCP/UDP mode, the
    physics server runs in a separate thread from the client (PyBullet),
    and setRealTimeSimulation allows the physics server thread to add
    additional calls to :func:`~pybullet_api.stepSimulation`.

    Parameters
    ----------
    enableRealTimeSimulation : int
        0 to disable real-time simulation, 1 to enable

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def getBasePositionAndOrientation(objectUniqueId, physicsClientId=0):
    """Get the world position and orientation of the base of the object. (x,y,z) position vector and (x,y,z,w) quaternion orientation.

    See also :func:`~pybullet_api.resetBasePositionAndOrientation` to reset
    the position and orientation of the object.

    Parameters
    ----------
    objectUniqueId : int
        Object unique id, as returned from :func:`~pybullet_api.loadURDF`.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    basePosition : array-like, shape (3,)
        Current position of the base (or root link) of the body in Cartesian
        world coordinates (x, y, z)

    baseOrientation : array-like, shape (4,)
        Current orientation of the base as quaternion (x, y, z, w)
    """


###############################################################################
# Collision Detection Queries
###############################################################################


def getOverlappingObjects(aabbMin, aabbMax, physicsClientId=0):
    """Return all the objects that have overlap with a given axis-aligned bounding box volume (AABB).

    This query will return all the unique ids of objects that have axis aligned
    bounding box overlap with a given axis aligned bounding box. Note that the
    query is conservative and may return additional objects that don't have
    actual AABB overlap. This happens because the acceleration structures have
    some heuristic that enlarges the AABBs a bit (extra margin and extruded
    along the velocity vector).

    Parameters
    ----------
    aabbMin : array-like, shape (3,)
        Minimum coordinates of the aabb.

    aabbMax : array-like, shape (3,)
        Maximum coordinates of the aabb.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    objectUniqueIds : list
        List of overlapping object unique ids.
    """


def getAABB(bodyUniqueId, linkIndex=-1, physicsClientId=0):
    """Get the axis aligned bound box min and max coordinates in world space.

    You can query the axis aligned bounding box (in world space) given an
    object unique id, and optionally a link index. (when you don't pass the
    link index, or use -1, you get the AABB of the base).

    Parameters
    ----------
    bodyUniqueId : int
        Object unique id as returned by creation methods.

    linkIndex : int, optional (default: -1)
        Link index in range [0..getNumJoints(..)] or -1 for the base.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    aabbMin : array-like, shape (3,)
        Minimum coordinates in world frame.

    aabbMax : array-like, shape (3,)
        Maximum coordinates in world frame.

    Examples
    --------
    See also the `getAABB.py <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/getAABB.py>`_
    example.
    """


def getContactPoints(bodyA=-1, bodyB=-1, linkIndexA=-2, linkIndexB=-2, physicsClientId=0):
    """Return existing contact points after the :func:`~pybullet_api.stepSimulation` command.

    Note that if you change the state of the simulation after
    :func:`~pybullet_api.stepSimulation`, the contact points are not updated
    and potentially invalid.

    .. image:: ../_static/contact_points.png
        :alt: Values returned by pybullet.getContactPoints().
        :width: 50%
        :align: center

    Parameters
    ----------
    bodyA : int, optional
        Only report contact points that involve body A.

    bodyB : int, optional
        Only report contact points that involve body B. Important: you need
        to have a valid bodyA if you provide body B.

    linkIndexA : int, optional
        Only report contact points that involve linkIndexA of bodyA.

    linkIndexB : int, optional
        Only report contact points that involve linkIndexB of bodyB.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    A list of tuples of contact points. Each tuple has the following fields:

    contactFlag : int
        Reserved

    bodyUniqueIdA : int
        Body unique id of body A.

    bodyUniqueIdB : int
        Body unique id of body B.

    linkIndexA : int
        Link index of body A, -1 for base.

    linkIndexB : int
        Link index of body B, -1 for base.

    positionOnA : array-like, shape (3,)
        Contact position on A, in Cartesian world coordinates.

    positionOnB : array-like, shape (3,)
        Contact position on B, in Cartesian world coordinates.

    contactNormalOnB : array-like, shape (3,)
        Contact normal on B, pointing towards A.

    contactDistance : float
        Contact distance, positive for separation, negative for penetration.

    normalForce : float
        Normal force applied during the last step.

    lateralFriction1 : float
        Lateral friction force in the lateralFrictionDir1 direction.

    lateralFrictionDir1 : array-like, shape (3,)
        First lateral friction direction.

    lateralFriction2 : float
        Lateral friction force in the lateralFrictionDir2 direction.

    lateralFrictionDir2 : array-like, shape (3,)
        Second lateral friction direction.
    """


def getClosestPoints(bodyA, bodyB, distance, linkIndexA=-2, linkIndexB=-2, physicsClientId=0):
    """Compute the closest points between two objects, if the distance is below a given threshold.

    It is also possible to compute the closest points, independent from
    :func:`~pybullet_api.stepSimulation` or performCollisionDetection. This
    also lets you compute closest points of objects with an arbitrary
    separating distance. In this query there will be no normal forces
    reported.

    Parameters
    ----------
    bodyA : int
        Only report points that involve body A.

    bodyB : int
        Only report points that involve body B.

    distance : float
        If the distance between objects exceeds this maximum distance, no
        points may be returned.

    linkIndexA : int, optional
        Only report points that involve linkIndexA of bodyA.

    linkIndexB : int, optional
        Only report points that involve linkIndexB of bodyB.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    A list of tuples of points. Each tuple has the following fields:

    contactFlag : int
        Reserved

    bodyUniqueIdA : int
        Body unique id of body A.

    bodyUniqueIdB : int
        Body unique id of body B.

    linkIndexA : int
        Link index of body A, -1 for base.

    linkIndexB : int
        Link index of body B, -1 for base.

    positionOnA : array-like, shape (3,)
        Contact position on A, in Cartesian world coordinates.

    positionOnB : array-like, shape (3,)
        Contact position on B, in Cartesian world coordinates.

    contactNormalOnB : array-like, shape (3,)
        Contact normal on B, pointing towards A.

    contactDistance : float
        Contact distance, positive for separation, negative for penetration.

    normalForce : float
        Normal force applied during the last step. This will always be 0.

    lateralFriction1 : float
        Lateral friction force in the lateralFrictionDir1 direction.

    lateralFrictionDir1 : array-like, shape (3,)
        First lateral friction direction.

    lateralFriction2 : float
        Lateral friction force in the lateralFrictionDir2 direction.

    lateralFrictionDir2 : array-like, shape (3,)
        Second lateral friction direction.
    """


def rayTest(rayFromPosition, rayToPosition, physicsClientId=0):
    """Cast a ray and return the first object hit, if any.

    Parameters
    ----------
    rayFromPosition : array-like, shape (3,)
        Start of the ray in world coordinates.

    rayToPosition : array-like, shape (3,)
        End of the ray in world coordinates.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    objectUniqueId : int
        Object unique id of the hit object.

    linkIndex : int
        Link index of the hit object, or -1 if none/parent.

    hit_fraction : float
        Hit fraction along the ray in range [0,1] along the ray.

    hit_position : array-like, shape (3,)
        Hit position in Cartesian world coordinates.

    hit_normal : array-like, shape (3,)
        Hit normal in Cartesian world coordinates.
    """


def rayTestBatch(rayFromPositions, rayToPositions, parentObjectUniqueId=-1,
                 parentLinkIndex=-1, numThreads=1, reportHitNumber=-1,
                 collisionFilterMask=-1, fractionEpsilon=-1):
    """Cast a batch of rays and return the result for each of the rays (first object hit, if any. or -1).

    This is similar to :func:`~pybullet_api.rayTest`, but allows you to provide
    an array of rays, for faster execution. The size of 'rayFromPositions' needs
    to be equal to the size of 'rayToPositions'. You can one ray result per
    ray, even if there is no intersection: you need to use the objectUniqueId
    field to check if the ray has hit anything: if the objectUniqueId is -1,
    there is no hit. In that case, the 'hit fraction' is 1. The maximum number
    of rays per batch is pybullet.MAX_RAY_INTERSECTION_BATCH_SIZE.

    Parameters
    ----------
    rayFromPositions : array-like, shape (n_rays, 3)
        List of start points for each ray, in world coordinates.

    rayToPositions : array-like, shape (n_rays, 3)
        List of end points for each ray in world coordinates.

    parentObjectUniqueId : int, optional
        Ray from/to is in local space of a parent object.

    parentLinkIndex : int, optional
        Ray from/to is in local space of a parent object.

    numThreads : int, optional
        Use multiple threads to compute ray tests (0 = use all threads
        available, positive number = exactly this amoung of threads,
        default =-1 =  single-threaded).

    reportHitNumber : int, optional
        Instead of first closest hit, you can report the n-th hit.

    collisionFilterMask : int, optional
        Only test hits if the bitwise and between collisionFilterMask and body
        collision filter group is non-zero. See
        :func:`~pybullet_api.setCollisionFilterGroupMask` on how to modify the
        body filter mask/group.

    fractionEpsilon : float, optional
        Only useful when using reportHitNumber: ignore duplicate hits if the
        fraction is similar to an existing hit within this fractionEpsilon
        when hitting the same body. For example, a ray may hit many co-planar
        triangles of one body, you may only be interested in one of those hits.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    A tuple for each ray. Each tuple has the following fields:

    objectUniqueId : int
        Object unique id of the hit object.

    linkIndex : int
        Link index of the hit object, or -1 if none/parent.

    hit_fraction : float
        Hit fraction along the ray in range [0,1] along the ray.

    hit_position : array-like, shape (3,)
        Hit position in Cartesian world coordinates.

    hit_normal : array-like, shape (3,)
        Hit normal in Cartesian world coordinates.

    Examples
    --------
    Output is one ray intersection result per input ray, with the same
    information as in above rayTest query. See batchRayTest.py example how to
    use it.
    """


def getCollisionShapeData():
    """Return the collision shape information for one object.

    You can query the collision geometry type and other collision shape
    information of existing body base and links using this query. It works
    very similar to :func:`~pybullet_api.getVisualShapeData`.

    Parameters
    ----------
    objectUniqueId : int
        Object unique id, received from :func:`~pybullet_api.loadURDF` etc.

    linkIndex : int
        Link index or -1 for the base.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    objectUniqueId : int
        Object unique id.

    linkIndex : int
        Link index or -1 for the base.

    geometryType : int
        Geometry type: GEOM_BOX, GEOM_SPHERE, GEOM_CAPSULE, GEOM_MESH,
        GEOM_PLANE.

    dimensions : array-like, shape (3,)
        Depends on geometry type: for GEOM_BOX: extents, for GEOM_SPHERE
        dimensions[0] = radius, for GEOM_CAPSULE and GEOM_CYLINDER,
        dimensions[0] = height (length), dimensions[1] = radius. For GEOM_MESH,
        dimensions is the scaling factor.

    filename : str
        Only for GEOM_MESH: file name (and path) of the collision mesh asset.

    localFramePos : array-like, shape (3,)
        Local position of the collision frame with respect to the center of
        mass/inertial frame.

    localFrameOrn : array-like, shape (4,)
        Local orientation of the collision frame with respect to the inertial
        frame.
    """


def vhacd(
        fileNameIn, fileNameOut, fileNameLog, concavity=0.0025, alpha=0.05,
        beta=0.05, gamma=0.00125, minVolumePerCH=0.0001, resolution=100000,
        maxNumVerticesPerCH=64, depth=20, planeDownsampling=4,
        convexhullDownsampling=4, pca=0, mode=0, convexhullApproximation=1,
        physicsClientId=0):
    """Compute volume hierarchical convex decomposition of an OBJ file.

    PyBullet includes an implementation of Volumetric Hierarchical Approximate
    Decomposition (vhacd), by Khaled Mamou. This can import a concave
    Wavefront .obj file and export a new Wavefront obj file that contains the
    convex decomposed parts. This can be used in PyBullet to efficiently deal
    with concave moving geometry.

    For static (non-moving) concave triangle mesh environments, you can tag
    triangle meshes as concave using a tag in URDF files (<link concave="yes"
    name="baseLink">) or using createCollisionShape with
    flags=pybullet.GEOM_FORCE_CONCAVE_TRIMESH.

    Parameters
    ----------
    fileNameIn : str
        Source (concave) Wavefront obj file name

    fileNameOut : str
        Destination (convex decomposition) Wavefront obj file name

    fileNameLog : str
        Log file name

    concavity : float, optional (default: 0.0025)
        Maximum allowed concavity. Range: [0, 1]

    alpha : float, optional (default: 0.05)
        Controls the bias toward clipping along symmetry planes. Range: [0, 1]

    beta : float, optional (default: 0.05)
        Controls the bias toward clipping along revolution axes. Range: [0, 1]

    gamma : float, optional (default: 0.00125)
        Controls the maximum allowed concavity during the merge stage.
        Range: [0, 1]

    minVolumePerCH : float, optional (default: 0.0001)
        Controls the adaptive sampling of the generated convex-hulls.
        Range: [0, 0.01]

    resolution : int, optional (default: 100,000)
        Maximum number of voxels generated during the voxelization stage.
        Range: [10,000, 16,000,000]

    maxNumVerticesPerCH : int, optional (default: 64)
        Controls the maximum number of triangles per convex-hull.
        Range: [4, 1024]

    depth : int, optional (default: 20)
        Maximum number of clipping stages. During each split stage, parts
        with a concavity higher than the user defined threshold are clipped
        according the best clipping plane. Range: [1, 32]

    planeDownsampling : int, optional (default: 4)
        Controls the granularity of the search for the "best" clipping plane
        Range: [1, 16]

    convexhullDownsampling : int, optional (default: 4)
        Controls the precision of the convex-hull generation process during
        the clipping plane selection stage. Range: [1, 16]

    pca : int, optional (default: 0)
        Enable/disable normalizing the mesh before applying the convex
        decomposition.

    mode : int, optional (default: 0)
        0: voxel-based approximate convex decomposition,
        1: tetrahedron-based approximate convex decomposition

    convexhullApproximation : int, optional (default: 1)
        Enable/disable approximation when computing convex hulls

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
        Note: vhacd decomposition happens on the client side at the moment.

    Examples
    --------

    .. code-block:: python

        import pybullet as p
        import pybullet_data as pd
        import os

        p.connect(p.DIRECT)
        name_in = os.path.join(pd.getDataPath(), "duck.obj")
        name_out = "duck_vhacd2.obj"
        name_log = "log.txt"
        p.vhacd(name_in, name_out, name_log)
    """


def setCollisionFilterGroupMask(
        bodyUniqueId, linkIndexA, collisionFilterGroup, collisionFilterMask,
        physicsClientId=0):
    """Set the collision filter group and the mask for a body.

    Each body is part of a group. It collides with other bodies if their
    group matches the mask, and vice versa. The following check is performed
    using the group and mask of the two bodies involved. It depends on the
    collision filter mode.

    Parameters
    ----------
    bodyUniqueId : int
        ID of the body to be configured.

    linkIndexA : int
        Link index of the body to be configured.

    collisionFilterGroup : int
        Bitwise group of the filter.

    collisionFilterMask : int
        Bitwise mask of the filter.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def setCollisionFilterPair(
        bodyUniqueIdA, bodyUniqueIdB, linkIndexA, linkIndexB, enableCollision,
        physicsClientId=0):
    """Enable or disable collision detection between two object links.

    There is a plugin API to write your own collision filtering implementation
    as well, see the `collisionFilterPlugin
    <https://github.com/bulletphysics/bullet3/tree/master/examples/SharedMemory/plugins/collisionFilterPlugin>`_
    implementation.

    Parameters
    ----------
    bodyUniqueIdA : int
        ID of body A to be filtered.

    bodyUniqueIdB : int
        ID of body B to be filtered, A==B implies self-collision.

    linkIndexA : int
        Link index of body A.

    linkIndexB : int
        Link index of body B.

    enableCollision : int
        1 to enable collision, 0 to disable collision.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


###############################################################################
# Inverse Dynamics, Kinematics
###############################################################################


def calculateInverseDynamics(bodyUniqueId, objPositions, objVelocities, objAccelerations, physicsClientId=0):
    """Given an object id, joint positions, joint velocities and joint accelerations, compute the joint forces using inverse dynamics.

    This function will compute the forces needed to reach the given joint
    accelerations, starting from specified joint positions and velocities.
    The inverse dynamics is computed using the recursive Newton Euler algorithm
    (RNEA).

    Note that when multidof (spherical) joints are involved, the
    function uses a different code path and is a bit slower.

    Also note that it ignores the joint/link damping, while forward dynamics
    includes those damping terms. So if you want to compare the inverse
    dynamics and forward dynamics, make sure to set those damping terms to zero
    using :func:`~pybullet_api.changeDynamics` with jointDamping and link
    damping through linearDamping and angularDamping.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id, as returned by loadURDF etc.

    objPositions : array-like, shape (n_dof,)
        Joint positions (angles) for each degree of freedom (DoF). Note that
        fixed joints have 0 degrees of freedom. The base is skipped/ignored in
        all cases (floating base and fixed base).

    objVelocities : array-like, shape (n_dof,)
        Joint velocities for each degree of freedom (DoF).

    objAccelerations : array-like, shape (n_dof,)
        Desired joint accelerations for each degree of freedom (DoF).

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    jointForcesOutput : array-like, shape (n_dof,)
        Joint forces for each degree of freedom.
    """


def calculateMassMatrix(bodyUniqueId, objPositions, physicsClientId=0):
    """Compute the mass matrix for an object and its chain of bodies.

    This function will compute the system inertia for an articulated body given
    its joint positions. The composite rigid body algorithm (CBRA) is used to
    compute the mass matrix.

    Note that when multidof (spherical) joints are involved, the function will
    use a different code path that is a bit slower.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id, as returned by loadURDF etc.

    objPositions : array-like, shape (n_dof,)
        JointPositions for each link.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    massMatrix : array-like, shape (n_dof, n_dof)
        Square mass matrix.
    """


def calculateInverseKinematics(bodyUniqueId, endEffectorLinkIndex, targetPosition):
    """Inverse Kinematics bindings.

    You can compute the joint angles that makes the end-effector reach a given
    target position in Cartesian world space. Internally, Bullet uses an
    improved version of Samuel Buss Inverse Kinematics library. At the moment
    only the Damped Least Squares method with or without Null Space control is
    exposed, with a single end-effector target. Optionally you can also specify
    the target orientation of the end effector. In addition, there is an option
    to use the null-space to specify joint limits and rest poses. This optional
    null-space support requires all 4 lists (lowerLimits, upperLimits,
    jointRanges, restPoses), otherwise regular IK will be used. See also
    inverse_kinematics.py example in Bullet/examples/pybullet/examples folder
    for details.

    By default, the IK will refine the solution until the distance between
    target end effector and actual end effector is below a residual threshold
    (1e-4) or the maximum number of iterations is reached.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id.

    endEffectorLinkIndex : int
        End effector link index.

    targetPosition : array-like, shape (3,)
        Target position of the end effector (its link coordinate, not center
        of mass coordinate!). By default this is in Cartesian world space,
        unless you provide currentPosition joint angles.

    targetOrientation : array-like, shape (4,), optional
        Target orientation in Cartesian world space, quaternion [x,y,z,w]. If
        not specified, pure position IK will be used.

    lowerLimits : array-like, shape (n_dof,), optional
        Null-space IK requires all 4 lists (lowerLimits, upperLimits,
        jointRanges, restPoses). Otherwise regular IK will be used. Only
        provide limits for joints that have them (skip fixed joints), so the
        length is the number of degrees of freedom. Note that lowerLimits,
        upperLimits, jointRanges can easily cause conflicts and instability
        in the IK solution. Try first using a wide range and limits, with just
        the rest pose.

    upperLimits : array-like, shape (n_dof,), optional
        Null-space IK requires all 4 lists (lowerLimits, upperLimits,
        jointRanges, restPoses). Otherwise regular IK will be used.
        lowerLimit and upperLimit specify joint limits

    jointRanges : array-like, shape (n_dof,), optional
        Null-space IK requires all 4 lists (lowerLimits, upperLimits,
        jointRanges, restPoses). Otherwise regular IK will be used.

    restPoses : array-like, shape (n_dof,), optional
        Null-space IK requires all 4 lists (lowerLimits, upperLimits,
        jointRanges, restPoses). Otherwise regular IK will be used. Favor an
        IK solution closer to a given rest pose.

    jointDamping : array-like, shape (n_dof,), optional
        Allows to tune the IK solution using joint damping factors.

    solver : int, optional
        pybullet.IK_DLS or pybullet.IK_SDLS, Damped Least Squares or Selective
        Damped Least Squares, as described in the paper by Samuel Buss
        "Selectively Damped Least Squares for Inverse Kinematics".

    currentPositions : array-like, shape (n_dof,), optional
        List of joint positions. By default PyBullet uses the joint positions
        of the body. If provided, the targetPosition and targetOrientation is
        in local space!

    maxNumIterations : int, optional
        Refine the IK solution until the distance between target and actual
        end effector position is below this threshold, or the maxNumIterations
        is reached. Default is 20 iterations.

    residualThreshold : float, optional
        Refine the IK solution until the distance between target and actual
        end effector position is below this threshold, or the maxNumIterations
        is reached.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    jointPositions : array-like, shape (n_dof,)
        Joint positions for each degree of freedom. The base and fixed joints
        are skipped.
    """


def calculateInverseKinematics2(bodyUniqueId, endEffectorLinkIndices, targetPositions):
    """Compute the inverse kinematics given multiple links, current joint positions, and target positions for the end effectors.

    Similar to :func:`~pybullet_api.calculateInverseKinematics`, but it takes a
    list of end-effector indices and their target positions (no orientations at
    the moment).

    Parameters
    ----------
    bodyUniqueId : int
        Body unique id.

    endEffectorLinkIndices : list of int
        End effector link indices.

    targetPositions : array-like, shape (3,)
        Target position of the end effector (its link coordinate, not center
        of mass coordinate!). By default this is in Cartesian world space,
        unless you provide currentPosition joint angles.

    lowerLimits : array-like, shape (n_dof,), optional
        Null-space IK requires all 4 lists (lowerLimits, upperLimits,
        jointRanges, restPoses). Otherwise regular IK will be used. Only
        provide limits for joints that have them (skip fixed joints), so the
        length is the number of degrees of freedom. Note that lowerLimits,
        upperLimits, jointRanges can easily cause conflicts and instability
        in the IK solution. Try first using a wide range and limits, with just
        the rest pose.

    upperLimits : array-like, shape (n_dof,), optional
        Null-space IK requires all 4 lists (lowerLimits, upperLimits,
        jointRanges, restPoses). Otherwise regular IK will be used.
        lowerLimit and upperLimit specify joint limits

    jointRanges : array-like, shape (n_dof,), optional
        Null-space IK requires all 4 lists (lowerLimits, upperLimits,
        jointRanges, restPoses). Otherwise regular IK will be used.

    restPoses : array-like, shape (n_dof,), optional
        Null-space IK requires all 4 lists (lowerLimits, upperLimits,
        jointRanges, restPoses). Otherwise regular IK will be used. Favor an
        IK solution closer to a given rest pose.

    jointDamping : array-like, shape (n_dof,), optional
        Allows to tune the IK solution using joint damping factors.

    solver : int, optional
        pybullet.IK_DLS or pybullet.IK_SDLS, Damped Least Squares or Selective
        Damped Least Squares, as described in the paper by Samuel Buss
        "Selectively Damped Least Squares for Inverse Kinematics".

    currentPosition : array-like, shape (n_dof,), optional
        List of joint positions. By default PyBullet uses the joint positions
        of the body. If provided, the targetPosition and targetOrientation is
        in local space!

    maxNumIterations : int, optional
        Refine the IK solution until the distance between target and actual
        end effector position is below this threshold, or the maxNumIterations
        is reached. Default is 20 iterations.

    residualThreshold : float, optional
        Refine the IK solution until the distance between target and actual
        end effector position is below this threshold, or the maxNumIterations
        is reached.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def calculateJacobian(
        bodyUniqueId, linkIndex, localPosition, objPositions, objVelocities,
        objAccelerations, physicsClientId=0):
    """Compute the jacobian for a specified local position on a body and its kinematics.

    This function will compute the translational and rotational jacobians for
    a point on a link, e.g. x_dot = J * q_dot. The returned jacobians are
    slightly different depending on whether the root link is fixed or floating.
    If floating, the jacobians will include columns corresponding to the root
    link degrees of freedom; if fixed, the jacobians will only have columns
    associated with the joints. The function call takes the full description
    of the kinematic state, this is because
    :func:`~pybullet_api.calculateInverseDynamics` is actually called first
    and the desired jacobians are extracted from this; therefore, it is
    reasonable to pass zero vectors for joint velocities and accelerations
    if desired.

    Parameters
    ----------
    bodyUniqueId : int
        Body unique ID

    linkIndex : int
        Link index for the jacobian.

    localPosition : array-like, shape (3,)
        The point on the specified link to compute the jacobian for, in link
        local coordinates around its center of mass.

    objPositions : array-like, shape (n_dof,)
        Joint positions (angles).

    objVelocities : array-like, shape (n_dof,)
        Joint velocities.

    objAccelerations : array-like, shape (n_dof,)
        Desired joint accelerations.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    linearJacobian : array-like, shape (3, n_dof)
        The translational jacobian, x_dot = J_t * q_dot.

    angularJacobian : array-like, shape (3, n_dof)
        The rotational jacobian, r_dot = J_r * q_dot.
    """


###############################################################################
# Virtual Reality
###############################################################################


def getVREvents(deviceTypeFilter, allAnalogAxes, physicsClientId=0):
    """Get Virtual Reality events, for example to track VR controllers position/buttons.

    Returns a list of events for a selected VR device that changed state since
    the last call to this function. When not providing any deviceTypeFilter,
    the default is to only report VR_DEVICE_CONTROLLER state. You can choose
    any combination of devices including VR_DEVICE_CONTROLLER, VR_DEVICE_HMD
    (Head Mounted Device) and VR_DEVICE_GENERIC_TRACKER (such as the HTC Vive
    Tracker).

    Note that VR_DEVICE_HMD and VR_DEVICE_GENERIC_TRACKER only report position
    and orientation events.

    Examples
    --------
    See Bullet/examples/pybullet/examples/vrEvents.py for an example of VR
    drawing and Bullet/examples/pybullet/examples/vrTracker.py to track HMD
    and generic tracker.

    Parameters
    ----------
    deviceTypeFilter : int, optional
        Default is VR_DEVICE_CONTROLLER. You can also choose VR_DEVICE_HMD
        or VR_DEVICE_GENERIC_TRACKER or any combination of them.

    allAnalogAxes : int, optional
        1 for all analogue axes, 0 with just a single axis

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    controllerId : int
        Controller index (0..MAX_VR_CONTROLLERS).

    controllerPosition : array-like, shape (3,)
        Controller position, in world space Cartesian coordinates.

    controllerOrientation : array-like, shape (4,)
        Controller orientation quaternion [x,y,z,w] in world space.

    controllerAnalogueAxis : float
        Analogue axis value.

    numButtonEvents : int
        Number of button events since last call to this function.

    numMoveEvents : int
        Number of move events since last call to this function.

    buttons : list of int (OpenVR has a maximum of 64 buttons)
        Button states: VR_BUTTON_IS_DOWN (currently held down),
        VR_BUTTON_WAS_TRIGGERED (went down at least once since last cal to
        this function, VR_BUTTON_WAS_RELEASED (was released at least once
        since last call to this function). Note that only VR_BUTTON_IS_DOWN
        reports actual current state. For example if the button went down and
        up, you can tell from the RELEASE/TRIGGERED flags, even though IS_DOWN
        is still false. Note that in the log file, those buttons are packed
        with 10 buttons in 1 integer (3 bits per button).

    deviceType : int
        Type of device: VR_DEVICE_CONTROLLER, VR_DEVICE_HMD or
        VR_DEVICE_GENERIC_TRACKER.

    allAnalogAxes : list of 10 floats, optional (only if explicitly requested!)
        Currently, MAX_VR_ANALOGUE_AXIS is 5, for each axis x and y value.
    """


def setVRCameraState(
        rootPosition=None, rootOrientation=None, trackObject=-2,
        trackObjectFlag=-1, physicsClientId=0):
    """Set properties of the VR Camera such as its root transform for teleporting or to track objects (camera inside a vehicle for example).

    This function allows to set the camera root transform offset position and
    orientation. This allows to control the position of the VR camera in the
    virtual world. It is also possible to let the VR Camera track an object,
    such as a vehicle.

    Parameters
    ----------
    rootPosition : array-like, shape (3,), optional
        Camera root position.

    rootOrientation : array-like, shape (4,), optional
        Camera root orientation in quaternion [x,y,z,w] format.

    trackObject : int, optional
        The object unique id to track.

    trackObjectFlag : int, optional
        Option: VR_CAMERA_TRACK_OBJECT_ORIENTATION (if enabled, both position
        and orientation is tracked).

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


###############################################################################
# Debug GUI, Lines, Text, Parameters
###############################################################################


def addUserDebugLine(lineFromXYZ, lineToXYZ):
    """Draw a user debug line.

    You can add a 3d line specified by a 3d starting point (from) and end
    point (to), a color [red, green, blue], a line width and a duration in
    seconds.

    Parameters
    ----------
    lineFromXYZ : array-like, shape (3,)
        Starting point of the line in Cartesian world coordinates.

    lineToXYZ : array-like, shape (3,)
        End point of the line in Cartesian world coordinates.

    lineColorRGB : array-like, shape (3,), optional
        RGB color [Red, Green, Blue] each component in range [0..1].

    lineWidth : float, optional
        Line width (limited by OpenGL implementation).

    lifeTime : float, optional
        Use 0 for permanent line, or positive time in seconds (afterwards
        the line with be removed automatically).

    parentObjectUniqueId : int, optional
        Draw line in local coordinates of a parent object/link.

    parentLinkIndex : int, optional
        Draw line in local coordinates of a parent object/link.

    replaceItemUniqueId : int, optional (default: -1)
        Replace an existing line (to improve performance and to avoid
        flickering of remove/add) See also the
        `f10_racecar.py <https://github.com/bulletphysics/pybullet_robots/blob/master/f10_racecar.py>`_
        example. -1 indicates that a new item should be generated.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    userDataId : int
        A non-negative unique id, that lets you remove the line using
        :func:`~pybullet_api.removeUserDebugItem`. (when using
        `replaceItemUniqueId` it will return replaceItemUniqueId).
    """


def addUserDebugText(text, textPosition, textColorRGB=(0, 0, 0), textSize=None, lifeTime=None, textOrientation=None, parentObjectUniqueId=None, parentLinkIndex=None, replaceItemUniqueId=None, physicsClientId=0):
    """Add a user debug text.

    You can add some 3d text at a specific location using a color and size.

    Examples
    --------

    See also pybullet/examples/debugDrawItems.py

    Parameters
    ----------
    text : str
        Text represented as a string (array of characters)

    textPosition : array-like, shape (3,)
        3d position of the text in Cartesian world coordinates [x,y,z]

    textColorRGB : array-like, shape (3,), optional
        RGB color [Red, Green, Blue] each component in range [0..1]

    textSize : float, optional
        Text size

    lifeTime : float, optional
        Use 0 for permanent text, or positive time in seconds (afterwards
        the text with be removed automatically)

    textOrientation : array-like, shape (4,), optional
        By default, debug text will always face the camera, automatically
        rotation. By specifying a text orientation (quaternion), the
        orientation will be fixed in world space or local space (when parent
        is specified). Note that a different implementation/shader is used
        for camera facing text, with different appearance: camera facing text
        uses bitmap fonts, text with specified orientation uses TrueType
        fonts.

    parentObjectUniqueId : int, optional
        Draw line in local coordinates of a parent object/link.

    parentLinkIndex : int, optional
        Draw line in local coordinates of a parent object/link.

    replaceItemUniqueId : int, optional
        Replace an existing text item (to avoid flickering of remove/add).

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    userDataId : int
        A non-negative unique id, that lets you remove the line using
        :func:`~pybullet_api.removeUserDebugItem`. (when using
        `replaceItemUniqueId` it will return replaceItemUniqueId).
    """


def addUserDebugParameter(paramName, rangeMin, rangeMax, startValue, physicsClientId=0):
    """Add a user debug parameter, such as a slider, that can be controlled using a GUI.

    This function lets you add custom sliders and buttons to tune parameters.
    It will return a unique id. This lets you read the value of the parameter
    using :func:`~pybullet_api.readUserDebugParameter`.

    Examples
    --------

    .. code-block::

        pybullet.addUserDebugParameter("button", 1, 0, 1)
        pybullet.addUserDebugParameter("my_slider", 3, 5, 4)

    .. image:: ../_static/100002010000010B000000921EE57157B545E05F.png
        :alt: Debug Parameters
        :width: 50%
        :align: center

    Parameters
    ----------
    paramName : str
        Name of the parameter

    rangeMin : float
        Minimum value. If Minimum value > maximum value a button instead of
        slider will appear.

    rangeMax : float
        Maximum value.

    startValue : float
        Starting value.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    itemUniqueId : int
        A non-negative unique id, that lets you read the parameter with
        :func:`~pybullet_api.readUserDebugParameter`.
    """


def removeAllUserParameters(physicsClientId=0):
    """Remove all user debug parameters (sliders, buttons).

    Parameters
    ----------
    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def removeUserDebugItem(itemUniqueId, physicsClientId=0):
    """Remove a user debug draw item, giving its unique id.

    Parameters
    ----------
    itemUniqueId : int
        Unique id of the debug item to be removed (line, text etc).

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def removeAllUserDebugItems(physicsClientId=0):
    """Remove all user debug draw items (text, lines, etc.).

    Parameters
    ----------
    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """



def readUserDebugParameter(itemUniqueId, physicsClientId=0):
    """Read the current value of a user debug parameter, given the user debug item unique id.

    Parameters
    ----------
    itemUniqueId : int
        The unique id returned by :func:`~pybullet_api.addUserDebugParameter`.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    value : float
        The most up-to-date reading of the parameter for a slider. For a
        button, the value increases by 1 with each button press.
    """


def resetDebugVisualizerCamera(
        cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition,
        physicsClientId=0):
    """For the 3D OpenGL Visualizer, set the camera distance, yaw, pitch and target position.

    You can reset the 3D OpenGL debug visualizer camera distance (between eye
    and camera target position), camera yaw and pitch and camera target position.

    .. warning::

        The return arguments of :func:`~pybullet_api.getDebugVisualizerCamera`
        are in a different order than :func:`~pybullet_api.resetDebugVisualizerCamera`.
        Will be fixed in a future API revision (major new version).

    Parameters
    ----------
    cameraDistance : float
        Distance from eye to camera target position

    cameraYaw : float
        Camera yaw angle (in degrees) left/right

    cameraPitch : float
        Camera pitch angle (in degrees) up/down

    cameraTargetPosition : array-like, shape (3,)
        Focus point of the camera

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def setDebugObjectColor(objectUniqueId, linkIndex, objectDebugColorRGB=None, physicsClientId=0):
    """Override the wireframe debug drawing color for a particular object unique id / link index.

    The built-in OpenGL visualizers have a wireframe debug rendering feature:
    press 'w' to toggle. The wireframe has some default colors. You can
    override the color of a specific object and link with this function.

    Parameters
    ----------
    objectUniqueId : int
        Unique id of the object

    linkIndex : int
        Link index

    objectDebugColorRGB : array-like, shape (3,)
        Debug color in [Red, Green, Blue]. If not provided, the custom color will be removed.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def addUserData(
        bodyUniqueId, key, value, linkIndex=-1, visualShapeIndex=-1,
        physicsClientId=0):
    """Adds or updates a user data entry. Returns user data identifier.

    In a nutshell, add, remove and query user data, at the moment text strings,
    attached to any link of a body. See the `userData.py
    <https://github.com/erwincoumans/bullet3/blob/master/examples/pybullet/examples/userData.py>`_
    example on how to use it. It will return a userDataId. Note that you can
    also add user data in a urdf file.

    Parameters
    ----------
    bodyUniqueId : int
        Unique id of the object.

    key : str
        Data key.

    value : str
        Value.

    linkIndex : int, optional (default: -1)
        Index of the link to which the data is associated.

    visualShapeIndex : int, optional (default: -1)
        Index of the visual shape to which the data is associated.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    userDataId : int
        User data identifier.
    """


def getUserData(userDataId, physicsClientId=0):
    """Returns the user data value.

    This function will receive user data given the userDataId returned by
    :func:`~pybullet_api.addUserData`. See userData.py for example usage.

    Parameters
    ----------
    userDataId : int
        User data id.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    userData : str
        User data.
    """


def syncUserData(bodyUniqueId, physicsClientId=0):
    """Update user data, in case other clients made changes.

    This function synchronizes the user data in case multiple clients change
    the user data.

    Parameters
    ----------
    bodyUniqueId or bodyUniqueIds : int or list of int
        Body unique id(s).

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def removeUserData(userDataId, physicsClientId=0):
    """Removes a user data entry.

    Parameters
    ----------
    userDataId : int
        User data identifier.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def getUserDataId(bodyUniqueId, key, linkIndex=-1, visualShapeIndex=-1, physicsClientId=0):
    """Retrieves the userDataId given the key and optionally link and visual shape index.

    Parameters
    ----------
    bodyUniqueId : int
        Unique id of the object.

    key : str
        Data key.

    linkIndex : int, optional (default: -1)
        Index of the link to which the data is associated.

    visualShapeIndex : int, optional (default: -1)
        Index of the visual shape to which the data is associated.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    userDataId : int
        User data id.
    """

def getNumUserData(bodyUniqueId, physicsClientId=0):
    """Retrieves the number of user data entries in a body.

    Parameters
    ----------
    bodyUniqueId : int
        Unique id of the object.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    numUserData : int
        Number of user data entries.
    """

def getUserDataInfo(bodyUniqueId, userDataIndex, physicsClientId=0):
    """Retrieves the key and the identifier of a user data as (userDataId, key, bodyUniqueId, linkIndex, visualShapeIndex).

    Parameters
    ----------
    bodyUniqueId : int
        Unique id of the object.

    userDataIndex : int
        Index of user data entry.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    userDataId : int
        User data id.

    key : str
        Data key.

    bodyUniqueId : int
        Unique id of the object.

    linkIndex : int, optional (default: -1)
        Index of the link to which the data is associated.

    visualShapeIndex : int, optional (default: -1)
        Index of the visual shape to which the data is associated.
    """


def configureDebugVisualizer(
        flag, enable, lightPosition=None, shadowMapResolution=None,
        shadowMapWorldSize=None, remoteSyncTransformInterval=-1,
        shadowMapIntensity=-1, rgbBackground=None, physicsClientId=0):
    """For the 3D OpenGL Visualizer, enable/disable GUI, shadows.

    You can configure some settings of the built-in OpenGL visualizer, such as
    enabling or disabling wireframe, shadows and GUI rendering. This is useful
    since some laptops or Desktop GUIs have performance issues with our
    OpenGL 3 visualizer.

    Examples
    --------

    .. code-block:: python

        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_WIREFRAME, 1)

    Parameters
    ----------
    flag : int
        The feature to enable or disable, such as COV_ENABLE_WIREFRAME,
        COV_ENABLE_SHADOWS,COV_ENABLE_GUI, COV_ENABLE_VR_PICKING,
        COV_ENABLE_VR_TELEPORTING, COV_ENABLE_RENDERING,
        COV_ENABLE_TINY_RENDERER, COV_ENABLE_VR_RENDER_CONTROLLERS,
        COV_ENABLE_KEYBOARD_SHORTCUTS, COV_ENABLE_MOUSE_PICKING,
        COV_ENABLE_Y_AXIS_UP (Z is default world up axis),
        COV_ENABLE_RGB_BUFFER_PREVIEW, COV_ENABLE_DEPTH_BUFFER_PREVIEW,
        COV_ENABLE_SEGMENTATION_MARK_PREVIEW

    enable : int
        0 or 1

    lightPosition : array-like, shape (3,), optional
        Position of the light for the visualizer

    shadowMapResolution : int, optional
        Size of the shadow map texture, typically a power of 2 for many
        GPUs. Default is 4096. Modern GPUs can handle 16384 or 32768 or
        higher.

    shadowMapWorldSize : int, optional
        Size of the shadow map in world space (units in meter, default
        is 10)

    remoteSyncTransformInterval : int, optional
        TODO

    shadowMapIntensity : float, optional
        TODO

    rgbBackground : array-like, shape (3,), optional
        RGB background color.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """


def getDebugVisualizerCamera(physicsClientId=0):
    """Get information about the 3D visualizer camera, such as width, height, view matrix, projection matrix etc.

    .. warning::

        The return values are in a different order than
        :func:`~pybullet_api.resetDebugVisualizerCamera`. Will be fixed in a
        future API revision (major new version).

    Parameters
    ----------
    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    cameraWidth : int
        Width of camera image.

    cameraHeight : int
        Height of camera image.

    viewMatrix : array-like, shape (16,)
        Camera view matrix.

    projMatrix : array-like, shape (16,)
        Camera projection matrix.

    camUp : array-like, shape (3,)
        Up direction of camera.

    camFwd : array-like, shape (3,)
        Forward direction of camera.

    hor : array-like, shape (3,)
        Horizontal direction.

    vert : array-like, shape (3,)
        Vertical direction.

    cameraYaw : float
        Camera yaw angle (in degrees) left/right.

    cameraPitch : float
        Camera pitch angle (in degrees) up/down.

    cameraDistance : float
        Distance from eye to camera target position.

    cameraTargetPosition : array-like, shape (3,)
        Focus point of the camera.
    """


def getKeyboardEvents(physicsClientId=0):
    """Get keyboard events, keycode and state.

    You can receive all keyboard events that happened since the last time
    you called this function. Each event has a keycode and a state. The state
    is a bit flag combination of KEY_IS_DOWN, KEY_WAS_TRIGGERED and
    KEY_WAS_RELEASED. If a key is going from 'up' to 'down' state, you receive
    the KEY_IS_DOWN state, as well as the KEY_WAS_TRIGGERED state. If a key
    was pressed and released, the state will be KEY_IS_DOWN and
    KEY_WAS_RELEASED.

    Some special keys are defined: B3G_F1 … B3G_F12, B3G_LEFT_ARROW,
    B3G_RIGHT_ARROW, B3G_UP_ARROW, B3G_DOWN_ARROW, B3G_PAGE_UP, B3G_PAGE_DOWN,
    B3G_PAGE_END, B3G_HOME, B3G_DELETE, B3G_INSERT, B3G_ALT, B3G_SHIFT,
    B3G_CONTROL, B3G_RETURN.

    Examples
    --------

    .. code-block:: python

        qKey = ord('q')
        keys = p.getKeyboardEvents()
        if qKey in keys and keys[qKey] & p.KEY_WAS_TRIGGERED:
            break

    Parameters
    ----------
    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    events : dict
        The output is a dictionary of keycode 'key' and keyboard state 'value'.
    """


def getMouseEvents(physicsClientId=0):
    """Get mouse events, event type and button state.

    Similar to :func:`~pybullet_api.getKeyboardEvents`, you can get the mouse
    events that happened since the last call to getMouseEvents. All the mouse
    move events are merged into a single mouse move event with the most
    up-to-date position. In addition, all mouse button events for a given
    button are merged. If a button went down and up, the state will be
    'KEY_WAS_TRIGGERED '. We reuse KEY_WAS_TRIGGERED / KEY_IS_DOWN /
    KEY_WAS_RELEASED for the mouse button states.

    Examples
    --------

    See `createVisualShape.py <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/createVisualShape.py>`_
    for an example of mouse events, to select/color objects.

    Parameters
    ----------
    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    eventType : int
        MOUSE_MOVE_EVENT=1, MOUSE_BUTTON_EVENT=2

    mousePosX : float
        x-coordinates of the mouse pointer

    mousePosY : float
        y-coordinates of the mouse pointer

    buttonIndex : int
        Button index for left/middle/right mouse button

    buttonState : int
        Flag KEY_WAS_TRIGGERED / KEY_IS_DOWN / KEY_WAS_RELEASED
    """


###############################################################################
# Plugins
###############################################################################


def loadPlugin(pluginPath, postFix, physicsClientId=0):
    """Load a plugin, could implement custom commands etc.

    Parameters
    ----------
    pluginPath : str
        Path, location on disk where to find the plugin.

    postFix : str
        Postfix name of the plugin that is appended to each API.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    pluginUniqueId : int
        If this identifier is negative, the plugin is not loaded. Once a plugin
        is loaded, you can send commands to the plugin using this identifier.
    """


def executePluginCommand(
        pluginUniqueId, textArgument="", intArgs=[], floatArgs=[],
        physicsClientId=0):
    """Execute a command, implemented in a plugin.

    Parameters
    ----------
    pluginUniqueId : int
        The unique id of the plugin, returned by
        :func:`~pybullet_api.loadPlugin`.

    textArgument : str, optional
        Text argument, to be interpreted by plugin.

    intArgs : list of int, optional
        List of integers, to be interpreted by plugin.

    floatArgs : list of float, optional
        List of floats, to be interpreted by plugin.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.

    Returns
    -------
    status : int or list
        Either result flag (-1 if the execution failed) or data given as
        a list of result flag, data type, and tuple of integers.
    """


def unloadPlugin(pluginUniqueId, physicsClientId=0):
    """Unload a plugin, given the pluginUniqueId.

    You can browse the `plugin implementation
    <https://github.com/bulletphysics/bullet3/tree/master/examples/SharedMemory/plugins>`_
    of PyBullet to get an idea what is possible.

    Parameters
    ----------
    pluginUniqueId : int
        The unique id of the plugin, returned by
        :func:`~pybullet_api.loadPlugin`.

    physicsClientId : int, optional (default: 0)
        If you connect to multiple physics servers, you can pick which one.
    """
