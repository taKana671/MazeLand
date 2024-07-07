from distutils.core import setup, Extension
from Cython.Build import cythonize
import numpy


ext = Extension(
    'create_maze2d_cy',
    ['create_maze2d_cy.pyx'],
    include_dirs=[numpy.get_include()]
)

setup(
    name='create_maze2d_cy',
    ext_modules=cythonize(ext)
)
