from libcpp.string cimport string
from libcpp.vector cimport vector

from cython cimport view
cimport cython

cimport alembic
from alembic cimport IPolyMeshSchema
from alembic cimport IV2fGeomParam
from alembic cimport IN3fGeomParam

cdef extern from "core.h" nogil:
    cdef alembic.IArchive open_abc(string path)
    cdef void read_mesh_objects(alembic.IObject object, vector[alembic.IPolyMesh] &mesh_list)
    cdef alembic.M44d get_final_matrix(alembic.IObject &iObj, double seconds)

    cdef IPolyMeshSchema.Sample get_mesh_sampler(alembic.IPolyMesh mesh, double seconds)
    cdef IV2fGeomParam.Sample get_iv2_sampler(alembic.IV2fGeomParam param, double seconds)
    cdef IN3fGeomParam.Sample get_in3_sampler(alembic.IN3fGeomParam param, double seconds)

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
    return AbcFile(path)

cdef class AbcFile(object):
    cdef alembic.IArchive archive
    cdef public list mesh_list
    cdef readonly double start_time
    cdef readonly double end_time

    def __init__(self, str path):
        cdef vector[alembic.IPolyMesh] mesh_list
        cdef AbcMesh mesh
        self.archive = open_abc(path)
        self.mesh_list =[]

        alembic.GetArchiveStartAndEndTime(self.archive, self.start_time, self.end_time)

        read_mesh_objects(self.archive.getTop(), mesh_list)

        for i in range(mesh_list.size()):
            mesh = AbcMesh.__new__(AbcMesh)
            mesh.mesh = mesh_list[i]
            self.mesh_list.append(mesh)

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

    property matrix:
        def __get__(self):
            cdef alembic.M44d m = get_final_matrix(self.mesh, self.time)
            cdef double[:,:] src =  <double[:4, :4]> &m[0][0]
            cdef double[:,:] dst = view.array(shape=(4, 4), itemsize=sizeof(double), format="d")
            # transpose for numpy
            for y in range(4):
                for x in range(4):
                    dst[y][x] = src[x][y]

            return dst

    property bbox:
        def __get__(self):
            cdef alembic.IPolyMeshSchema schema  = self.mesh.getSchema()
            cdef IPolyMeshSchema.Sample sampler = get_mesh_sampler(self.mesh, self.time)
            cdef alembic.Box3d bounds = sampler.getSelfBounds()

            cdef double[:,:] bbox = view.array(shape=(2, 4), itemsize=sizeof(double), format="d")

            bbox[0][0] = bounds.min.x
            bbox[0][1] = bounds.min.y
            bbox[0][2] = bounds.min.z
            bbox[0][3] = 1.0

            bbox[1][0] = bounds.max.x
            bbox[1][1] = bounds.max.y
            bbox[1][2] = bounds.max.z
            bbox[1][3] = 1.0

            return bbox

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
