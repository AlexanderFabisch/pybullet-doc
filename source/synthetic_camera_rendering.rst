**************************
Synthetic Camera Rendering
**************************

PyBullet has both a build-in OpenGL GPU visualizer and a build-in CPU renderer
based on TinyRenderer. This makes it very easy to render images from an
arbitrary camera position. On Linux, you can also enable hardware accelerated
OpenGL rendering without a X11 context, for example for cloud rendering on the
Google Cloud Platform or in a
`Colab <https://colab.research.google.com/drive/1u6j7JOqM05vUUjpVp5VNk0pd8q-vqGlx>`_.
See the `eglRenderTest.py <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/eglRenderTest.py>`_
example how to use it, as described in the 'Plugins' section.

The synthetic camera is specified by two 4 by 4 matrices: the view matrix and
the projection matrix. Since those are not very intuitive, there are some
helper methods to compute the view and projection matrix from understandable
parameters.

Check out `this article <http://ksimek.github.io/2013/08/13/intrinsic/>`_
about intrinsic camera matrix with links to OpenGL camera information.
