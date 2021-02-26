import gmsh
from math import pi

gmsh.initialize()

gmsh.model.add("box")

gmsh.option.setNumber("Mesh.Algorithm3D", 1)
gmsh.option.setNumber("Mesh.MeshSizeMin", 1)
gmsh.option.setNumber("Mesh.MeshSizeMax", 1)
gmsh.option.setNumber("Mesh.MeshSizeFactor", 0.1)

x, y, z = -0.5, -0.5, -0.5
dx, dy, dz = 1, 1, 1
gmsh.model.occ.add_box(x, y, z, dx, dy, dz, tag=1)

gmsh.model.occ.synchronize()

gmsh.model.addPhysicalGroup(3, [1], 1)
gmsh.model.setPhysicalName(3, 1, "The volume")

gmsh.model.mesh.generate(3)
gmsh.write("box.vtk")

gmsh.fltk.run()
gmsh.finalize()
