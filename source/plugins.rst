*******
Plugins
*******

PyBullet allows you to write plugins in C or C++ to add customize features.
Some core features of PyBullet are written as plugins, such as PD control,
rendering, gRPC server, collision filtering and Virtual Reality sync. Most
plugins that are core part of PyBullet are statically linked by default, so
you don't need to manually load and unload them.

On Linux, the eglPlugin is an example of a plugin that ships with PyBullet by
default. It can be enabled to use hardware OpenGL 3.x rendering without a X11
context, for example for cloud rendering on the Google Cloud Platform. See the
`eglRenderTest.py <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/eglRenderTest.py>`_
example how to use it.

PyBullet also comes with a fileIOPlugin that can load files from a zip file
directly and allows file caching. See the
`fileIOPlugin.py <https://github.com/erwincoumans/bullet3/blob/master/examples/pybullet/examples/fileIOPlugin.py>`_
example how to use it.
