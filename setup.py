from setuptools import setup, Extension
import numpy as np
from Cython.Build import cythonize

extensions = [
    Extension(
        "pointcloud",
        ["pointcloud.pyx"],
        extra_compile_args=["-O3", "-ffast-math"],
        define_macros=[('NPY_NO_DEPRECATED_API', 'NPY_1_7_API_VERSION')]
    )
]

setup(
    ext_modules=cythonize(extensions, compiler_directives={
        'language_level': "3",
        'boundscheck': False,
        'wraparound': False
    }),
    include_dirs=[np.get_include()]
)
