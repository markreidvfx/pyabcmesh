from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension
import os

ALEMBIC_PREFIX = os.environ.get("ALEMBIC_PREFIX", "")
ILMBASE_PREFEX =  os.environ.get("ILMBASE_PREFEX", "")
OPENEXR_PREFIX =  os.environ.get("OPENEXR_PREFIX", "")
HDF5_PREFIX =  os.environ.get("HDF5_PREFIX", "")
# for smart pointers if alembic built with boost
BOOST_PREFIX =  os.environ.get("BOOST_PREFIX", "")

ALEMBIC_INCLUDE_DIR = os.path.join(ALEMBIC_PREFIX, "include")
ALEMBIC_LIB_DIR = os.path.join(ALEMBIC_PREFIX, "lib")

ILMBASE_INCLUDE_DIR = os.path.join(ILMBASE_PREFEX, "include", "OpenEXR")
ILMBASE_LIB_DIR = os.path.join(ILMBASE_PREFEX, "lib")

OPENEXR_INCLUDE_DIR = os.path.join(OPENEXR_PREFIX, "include", "OpenEXR")
OPENEXR_LIB_DIR = os.path.join(OPENEXR_PREFIX, "lib")

HDF5_INCLUDE_DIR = os.path.join(HDF5_PREFIX, "include")
HDF5_LIB_DIR = os.path.join(HDF5_PREFIX, "lib")

BOOST_INCLUDE_DIR = os.path.join(BOOST_PREFIX, "include")
BOOST_LIB_DIR =  os.path.join(BOOST_PREFIX, "lib")

sourcefiles = [
"abcmesh/abcmesh.pyx",
"abcmesh/core.cpp"
]
extensions = [Extension("abcmesh",
    sourcefiles,
    language="c++",
    include_dirs = [
        ALEMBIC_INCLUDE_DIR,
        ILMBASE_INCLUDE_DIR,
        OPENEXR_INCLUDE_DIR,
        HDF5_INCLUDE_DIR,
        BOOST_INCLUDE_DIR,
        "abcmesh"
        ],
    libraries = [
        "Alembic",
        "Imath",
        "Half",
        "Iex",
        "hdf5",
        "hdf5_hl",
    ],
    library_dirs = [
        ALEMBIC_LIB_DIR,
        ILMBASE_LIB_DIR,
        OPENEXR_LIB_DIR,
        HDF5_LIB_DIR,
        BOOST_LIB_DIR,]
)]

setup(
    name = 'PyAbcMesh',
    version='0.1.0',
    description='Alembic bindings that provide fast access to mesh data via buffer protocol',
    author="Mark Reid",
    author_email="mindmark@gmail.com",
    url="https://github.com/markreidvfx/pyabcmesh",
    license='MIT',
  ext_modules = cythonize(extensions),
)
