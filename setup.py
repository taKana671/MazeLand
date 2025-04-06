from distutils.core import setup, Extension
from Cython.Build import cythonize
import numpy


extensions = [
    Extension(
        '*',
        ['maze_algorithm/cymaze/*.pyx'],
        include_dirs=[numpy.get_include()],
        define_macros=[("NPY_NO_DEPRECATED_API", "NPY_1_7_API_VERSION")]
    )
]

setup(
    ext_modules=cythonize(
        extensions,
        compiler_directives={'profile': False},
    )
)