from libcpp.string cimport string
from libcpp.vector cimport vector
from libc cimport math

from cython cimport view
cimport cython

cimport alembic
from alembic cimport IPolyMeshSchema
from alembic cimport IV2fGeomParam
from alembic cimport IN3fGeomParam

import os

cdef extern from "core.h" nogil:
    cdef alembic.IArchive open_abc(string path) except +
    cdef void read_objects(alembic.IObject object, vector[alembic.IPolyMesh] &mesh_list, vector[alembic.ICamera] &camera_list)
    cdef alembic.M44d get_final_matrix(alembic.IObject &iObj, double seconds)

    cdef IPolyMeshSchema.Sample get_mesh_sampler(alembic.IPolyMesh mesh, double seconds)
    cdef IV2fGeomParam.Sample get_iv2_sampler(alembic.IV2fGeomParam param, double seconds)
    cdef IN3fGeomParam.Sample get_in3_sampler(alembic.IN3fGeomParam param, double seconds)
    cdef alembic.CameraSample get_camera_sampler(alembic.ICamera camera, double seconds)

    cdef int get_size_p3f(alembic.P3fArraySamplePtr &item)
    cdef float * get_pointer_p3f(alembic.P3fArraySamplePtr &item)

    cdef int get_size_n3f(alembic.N3fArraySamplePtr &item)
    cdef float * get_pointer_n3f(alembic.N3fArraySamplePtr &item)

    cdef int get_size_v2f(alembic.V2fArraySamplePtr &item)
    cdef float *get_pointer_v2f(alembic.V2fArraySamplePtr &item)

    cdef int get_size_int32(alembic.Int32ArraySamplePtr &item)
    cdef int *get_pointer_int32(alembic.Int32ArraySamplePtr &item)

    cdef int get_size_uint32(alembic.UInt32ArraySamplePtr &item)
    cdef unsigned int *get_pointer_uint32(alembic.UInt32ArraySamplePtr &item)

def open(str path):
    if not os.path.exists(path):
      raise IOError("No such file or directory: '%s'" % path)
    return AbcFile(path)

cdef get_matrix(alembic.IObject obj, double seconds):
    cdef alembic.M44d m = get_final_matrix(obj, seconds)
    cdef double[:,:] src =  <double[:4, :4]> &m[0][0]
    cdef float[:,:] dst = view.array(shape=(4, 4), itemsize=sizeof(float), format="f")
    # transpose for numpy
    for y in range(4):
        for x in range(4):
            dst[y][x] = src[x][y]

    return dst

cdef class AbcFile(object):
    cdef alembic.IArchive archive
    cdef readonly list mesh_list
    cdef readonly list camera_list
    cdef readonly double start_time
    cdef readonly double end_time

    def __init__(self, str path):
        cdef vector[alembic.IPolyMesh] mesh_list
        cdef vector[alembic.ICamera] camera_list
        cdef AbcMesh mesh
        cdef AbcCamera camera
        self.archive = open_abc(path)
        self.mesh_list = []
        self.camera_list = []

        alembic.GetArchiveStartAndEndTime(self.archive, self.start_time, self.end_time)

        read_objects(self.archive.getTop(), mesh_list, camera_list)

        for i in range(mesh_list.size()):
            mesh = AbcMesh.__new__(AbcMesh)
            mesh.mesh = mesh_list[i]
            self.mesh_list.append(mesh)

        for i in range(camera_list.size()):
            camera = AbcCamera.__new__(AbcCamera)
            camera.camera = camera_list[i]
            self.camera_list.append(camera)

def frustum(float left, float right, float bottom, float top, float near, float far):
    assert(right != left)
    assert(bottom != top)
    assert(near != far)

    m = view.array(shape=(4, 4), itemsize=sizeof(float), format="f")
    for y in range(4):
        for x in range(4):
            m[y][x] = 0.0

    m[0][0] = +2.0 * near / (right - left)
    m[0][2] = (right + left) / (right - left)
    m[1][1] = +2.0 * near / (top - bottom)
    m[1][3] = (top + bottom) / (top - bottom)
    m[2][2] = -(far + near) / (far - near)
    m[2][3] = -2.0 * near * far / (far - near)
    m[3][2] = -1.0
    return m

cdef double degrees(double value):
    return value * 180 / math.M_PI

cdef class AbcCamera(object):
    cdef alembic.ICamera camera
    cdef public double time

    def __cinit__(self):
        self.time = 0.0

    def __init__(self):
        raise TypeError("cannot initialize from python")

    def perspective_matrix(self, aspect = 1.0, znear = None, zfar = None):
        if znear is None:
            znear = self.znear
        if zfar is None:
            zfar = self.zfar

        assert(znear != zfar)

        cdef double a = aspect
        cdef double znear_d = znear
        cdef double zfar_d = zfar
        cdef double fovy = self.fovy

        cdef double h = math.tan(fovy / 360.0 * math.M_PI) * znear_d
        cdef double w = h * a
        return frustum(-w, w, -h, h, znear_d, zfar_d)

    property name:
        def __get__(self):
            return self.camera.getName()

    property full_name:
        def __get__(self):
            return self.mesh.getFullName()

    property world_matrix:
        def __get__(self):
            return get_matrix(self.camera, self.time)

    property  znear:
        def __get__(self):
            cdef alembic.CameraSample sampler = get_camera_sampler(self.camera, self.time)
            cdef double near = sampler.getNearClippingPlane()
            return near
    property  zfar:
        def __get__(self):
            cdef alembic.CameraSample sampler = get_camera_sampler(self.camera, self.time)
            cdef double far = sampler.getFarClippingPlane()
            return far

    property fovy:
        def __get__(self):
            cdef alembic.CameraSample sampler = get_camera_sampler(self.camera, self.time)

            cdef double focal_length = sampler.getFocalLength()
            cdef double vertical_aperture = sampler.getVerticalAperture()
            # * 10.0 since vertical film aperture is in cm
            cdef double fovy = 2.0 * degrees(math.atan(vertical_aperture * 10.0 /
                                             (2.0 * focal_length)))
            return fovy
    property focal_length:
        def __get__(self):
            cdef alembic.CameraSample sampler = get_camera_sampler(self.camera, self.time)
            return sampler.getFocalLength()

    property fovx:
        def __get__(self):
            cdef alembic.CameraSample sampler = get_camera_sampler(self.camera, self.time)

            cdef double focal_length = sampler.getFocalLength()
            cdef double horizontal_aperture = sampler.getHorizontalAperture()
            # * 10.0 since vertical film aperture is in cm
            cdef double fovx = 2.0 * degrees(math.atan(horizontal_aperture * 10.0 /
                                             (2.0 * focal_length)))
            return fovx

def bounds_to_bbox(float[:,:] bounds):
    cdef float[:,:] bbox = view.array(shape=(8, 4), itemsize=sizeof(float), format="f")

    cdef float minx = bounds[0][0]
    cdef float miny = bounds[0][1]
    cdef float minz = bounds[0][2]

    cdef float maxx = bounds[1][0]
    cdef float maxy = bounds[1][1]
    cdef float maxz = bounds[1][2]

    bbox[0][0], bbox[0][1], bbox[0][2], bbox[0][3] = minx, miny, minz, 1.0
    bbox[1][0], bbox[1][1], bbox[1][2], bbox[1][3] = minx, miny, maxz, 1.0
    bbox[2][0], bbox[2][1], bbox[2][2], bbox[2][3] = minx, maxy, minz, 1.0
    bbox[3][0], bbox[3][1], bbox[3][2], bbox[3][3] = minx, maxy, maxz, 1.0
    bbox[4][0], bbox[4][1], bbox[4][2], bbox[4][3] = maxx, miny, minz, 1.0
    bbox[5][0], bbox[5][1], bbox[5][2], bbox[5][3] = maxx, miny, maxz, 1.0
    bbox[6][0], bbox[6][1], bbox[6][2], bbox[6][3] = maxx, maxy, minz, 1.0
    bbox[7][0], bbox[7][1], bbox[7][2], bbox[7][3] = maxx, maxy, maxz, 1.0
    return bbox

def bbox_to_bounds(float[:,:] bbox):

    cdef float[:,:] bounds = view.array(shape=(2, 3), itemsize=sizeof(float), format="f")

    cdef float minx = bbox[0][0]
    cdef float miny = bbox[0][1]
    cdef float minz = bbox[0][2]

    cdef float maxx = minx
    cdef float maxy = miny
    cdef float maxz = minz

    for i in range(1, 8):
        minx = min(minx, bbox[i][0])
        miny = min(miny, bbox[i][1])
        minz = min(minz, bbox[i][2])

        maxx = max(maxx, bbox[i][0])
        maxy = max(maxy, bbox[i][1])
        maxz = max(maxz, bbox[i][2])

    bounds[0][0] = minx
    bounds[0][1] = miny
    bounds[0][2] = minz

    bounds[1][0] = maxx
    bounds[1][1] = maxy
    bounds[1][2] = maxz
    return bounds

cdef class AbcMesh(object):
    cdef alembic.IPolyMesh mesh
    cdef public double time

    def __cinit__(self):
        self.time = 0.0

    def __init__(self):
        raise TypeError("cannot initialize from python")

    property name:
        def __get__(self):
            return self.mesh.getName()

    property full_name:
        def __get__(self):
            return self.mesh.getFullName()

    property world_matrix:
        def __get__(self):
            return get_matrix(self.mesh, self.time)

    property bounds:
        def __get__(self):
            cdef alembic.IPolyMeshSchema schema  = self.mesh.getSchema()
            cdef IPolyMeshSchema.Sample sampler = get_mesh_sampler(self.mesh, self.time)
            cdef alembic.Box3d box3d = sampler.getSelfBounds()

            cdef float[:,:] bounds = view.array(shape=(2, 3), itemsize=sizeof(float), format="f")

            bounds[0][0] = box3d.min.x
            bounds[0][1] = box3d.min.y
            bounds[0][2] = box3d.min.z

            bounds[1][0] = box3d.max.x
            bounds[1][1] = box3d.max.y
            bounds[1][2] = box3d.max.z
            return bounds

    property bbox:
        def __get__(self):
            return bounds_to_bbox(self.bounds)

    property face_counts:
        def __get__(self):
            cdef alembic.IPolyMeshSchema schema  = self.mesh.getSchema()
            cdef IPolyMeshSchema.Sample sampler = get_mesh_sampler(self.mesh, self.time)
            cdef alembic.Int32ArraySamplePtr face_counts = sampler.getFaceCounts()

            cdef int face_count_size = get_size_int32(face_counts)
            cdef int *face_count_ptr  = get_pointer_int32(face_counts)

            cdef int[:] face_count_view = <int[:face_count_size]> face_count_ptr

            return face_count_view.copy()

    property face_indices:
        def __get__(self):
            cdef alembic.IPolyMeshSchema schema  = self.mesh.getSchema()
            cdef IPolyMeshSchema.Sample sampler = get_mesh_sampler(self.mesh, self.time)
            cdef alembic.Int32ArraySamplePtr face_indices = sampler.getFaceIndices()

            cdef int indices_count = get_size_int32(face_indices)
            cdef int *indices_ptr =  get_pointer_int32(face_indices)

            cdef int[:] view = <int[:indices_count]> indices_ptr

            return view.copy()

    property vertices:
        def __get__(self):
            cdef alembic.IPolyMeshSchema schema  = self.mesh.getSchema()
            cdef IPolyMeshSchema.Sample sampler = get_mesh_sampler(self.mesh, self.time)
            cdef alembic.P3fArraySamplePtr verts = sampler.getPositions()

            cdef int vert_count = get_size_p3f(verts)
            cdef float *verts_ptr = get_pointer_p3f(verts)

            cdef float[:, :] view = <float[:vert_count, :3]>verts_ptr

            return  view.copy()

    property vertices_vec4:
        @cython.boundscheck(False)
        def __get__(self):
            cdef alembic.IPolyMeshSchema schema  = self.mesh.getSchema()
            cdef IPolyMeshSchema.Sample sampler = get_mesh_sampler(self.mesh, self.time)

            cdef alembic.P3fArraySamplePtr verts = sampler.getPositions()

            cdef int vert_count = get_size_p3f(verts)
            cdef float *verts_ptr = get_pointer_p3f(verts)

            cdef float[:, :] verts_view = <float[:vert_count, :3]>verts_ptr

            cdef float[:, :] verts_vec4 = view.array(shape=(vert_count, 4), itemsize=sizeof(float), format="f")
            with nogil:
                for i in range(verts_view.shape[0]):
                    verts_vec4[i][0] = verts_view[i][0]
                    verts_vec4[i][1] = verts_view[i][1]
                    verts_vec4[i][2] = verts_view[i][2]
                    verts_vec4[i][3] = 1.0

            return verts_vec4

    property uv_indices:
        def __get__(self):
            cdef alembic.IPolyMeshSchema schema  = self.mesh.getSchema()
            cdef IV2fGeomParam param = schema.getUVsParam()

            if not param.valid():
                return None

            cdef IV2fGeomParam.Sample sampler = get_iv2_sampler(param, self.time)

            if not sampler.valid():
                return None

            cdef alembic.UInt32ArraySamplePtr indices = sampler.getIndices()

            cdef int indices_count = get_size_uint32(indices)
            cdef unsigned int *indices_ptr = get_pointer_uint32(indices)
            cdef unsigned int[:] view = <unsigned int[:indices_count]> indices_ptr

            return view.copy()

    property uv_values:
        def __get__(self):
            cdef alembic.IPolyMeshSchema schema  = self.mesh.getSchema()
            cdef IV2fGeomParam param = schema.getUVsParam()

            if not param.valid():
                return None

            cdef IV2fGeomParam.Sample sampler = get_iv2_sampler(param, self.time)

            if not sampler.valid():
                return None

            cdef alembic.V2fArraySamplePtr values = sampler.getVals()

            cdef int value_count = get_size_v2f(values)
            cdef float *values_ptr = get_pointer_v2f(values)

            cdef float[:,:] view = <float[:value_count,:2]> values_ptr

            return view.copy()

    property normal_indices:
        def __get__(self):
            cdef alembic.IPolyMeshSchema schema  = self.mesh.getSchema()
            cdef IN3fGeomParam param = schema.getNormalsParam()
            if not param.valid():
                return None

            cdef IN3fGeomParam.Sample sampler = get_in3_sampler(param, self.time)
            if not sampler.valid():
                return None

            cdef alembic.UInt32ArraySamplePtr indices = sampler.getIndices()

            cdef int indices_count = get_size_uint32(indices)
            cdef unsigned int *indices_ptr = get_pointer_uint32(indices)

            cdef unsigned int[:] view = <unsigned int[:indices_count]> indices_ptr

            return view.copy()

    property normal_values:
        def __get__(self):
            cdef alembic.IPolyMeshSchema schema  = self.mesh.getSchema()
            cdef IN3fGeomParam param = schema.getNormalsParam()
            if not param.valid():
                return None

            cdef IN3fGeomParam.Sample sampler = get_in3_sampler(param, self.time)
            if not sampler.valid():
                return None

            cdef alembic.N3fArraySamplePtr values = sampler.getVals()

            cdef int value_count = get_size_n3f(values)
            cdef float *values_ptr = get_pointer_n3f(values)

            cdef float[:,:] view = <float[:value_count,:3]> values_ptr

            return view.copy()
