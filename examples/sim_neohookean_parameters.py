# requirements: numpy, pybullet, tqdm, pillow
import numpy as np
import pybullet
import pybullet_data
import tqdm
from PIL import Image


repulsion_stiffness = 8000
nus = [0.49]  # Poisson's ratio; from paper: [0.2, 0.3, 0.4, 0.49]
# NOTE lower young's moduli are typically faster
Es = [100]  # Young's modulus; [50, 100, 200, 500]
dampings = [0.005]  # [0.003, 0.005, 0.01, 0.02, 0.1]
dt = 1.0 / 480.0
images_at_steps = []
#images_at_steps = (np.arange(0.3, 1.1, 0.1) / dt).astype(int)

pybullet.connect(pybullet.GUI)
#pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
#pybullet.setAdditionalSearchPath("../data/")
pybullet.setAdditionalSearchPath("examples/")
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 0)
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_MOUSE_PICKING, 0)

# TODO what does it do?
pybullet.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)

for E in Es:
    for nu in nus:
        for damping in dampings:
            print(f"E = {E}, nu = {nu}")
            mu = 0.5 * E / (1.0 + nu)
            lmbda = E * nu / ((1.0 + nu) * (1.0 - 2.0 * nu))
            print(f"mu = {mu:.3f}, lambda = {lmbda:.3f}, damping = {damping}")

            E_str = str(E).replace(".", "_")
            nu_str = str(nu).replace(".", "_")
            damping_str = str(damping).replace(".", "_")
            identifier = f"E_{E_str}__nu_{nu_str}__damp_{damping_str}"
            print(f"identifier: {identifier}")

            pybullet.resetSimulation(pybullet.RESET_USE_DEFORMABLE_WORLD)
            pybullet.resetDebugVisualizerCamera(4, 0, -10, [0, 0, 0])
            pybullet.setTimeStep(dt)
            #pybullet.setPhysicsEngineParameter(numSubSteps=16)
            pybullet.setRealTimeSimulation(0)
            pybullet.setGravity(0, 0, -9.81)
            pybullet.loadURDF("plane.urdf", (0, 0, 0))

            objects = [
                ("box.vtk", "box.vtk", (-3, 0, 1), 0.5),
                ("torus_textured.obj", "torus.vtk", (0, 0, 1), 1),
                ("ball.obj", "ball.vtk", (3, 0, 1), 1),
            ]

            for obj, vtk, pos, scale in objects:
                obj_id = pybullet.loadSoftBody(
                    obj, simFileName=vtk,
                    mass=0.3, scale=scale,
                    frictionCoeff=0.5,
                    useNeoHookean=1, NeoHookeanMu=mu, NeoHookeanLambda=lmbda, NeoHookeanDamping=damping,
                    collisionMargin=0.0006, useSelfCollision=1,
                    repulsionStiffness=repulsion_stiffness,
                    basePosition=pos)

            ts = tqdm.tqdm(list(enumerate(np.arange(0, 1, dt))))
            for i, t in ts:
                ts.set_description(f"step = {i}, t = {t:.3f}")

                if (i + 1) in images_at_steps:
                    width, height, image, depth, seg = pybullet.getCameraImage(
                        1024, 768, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL,
                        flags=pybullet.ER_NO_SEGMENTATION_MASK)
                    im = Image.fromarray(image)
                    im.save(f"result/{identifier}_{i}.png")

                if not pybullet.isConnected():
                    break
                pybullet.stepSimulation()
