from libcpp cimport bool
from libcpp.string cimport string

cdef extern from "Alembic/AbcGeom/All.h" namespace "Alembic::AbcGeom" nogil:
    cdef cppclass M44d:
        double* operator[](int)

    cdef cppclass M33d:
        double* operator[](int)

    cdef cppclass V3d:
        double x
        double y
        double z

    cdef cppclass Box3d:
        bool isEmpty()
        V3d min
        V3d max

    cdef cppclass P3fArraySamplePtr:
        pass

    cdef cppclass N3fArraySamplePtr:
        pass

    cdef cppclass V2fArraySamplePtr:
        pass

    cdef cppclass Int32ArraySamplePtr:
        pass

    cdef cppclass UInt32ArraySamplePtr:
        pass

    cdef cppclass IV2fGeomParam:
        bool valid()
        cppclass Sample:
            UInt32ArraySamplePtr getIndices()
            V2fArraySamplePtr getVals()
            bool isIndexed()
            bool valid()

    cdef cppclass IN3fGeomParam:
        bool valid()
        cppclass Sample:
            UInt32ArraySamplePtr getIndices()
            N3fArraySamplePtr getVals()
            bool isIndexed()
            bool valid()

    cdef cppclass IObject:
        string &getName()
        string &getFullName()

    cdef cppclass ISampleSelector:
        pass

    cdef cppclass IPolyMeshSchema:
        cppclass Sample:
            P3fArraySamplePtr &getPositions()
            Int32ArraySamplePtr &getFaceIndices()
            Int32ArraySamplePtr &getFaceCounts()
            Box3d getSelfBounds()

        IN3fGeomParam getNormalsParam()
        IV2fGeomParam getUVsParam()

    cdef cppclass FilmBackXformOp:
        string getHint()

    cdef cppclass CameraSample:
        double getFocalLength()
        double getVerticalAperture()
        double getHorizontalAperture()
        double getNearClippingPlane()
        double getFarClippingPlane()
        double getLensSqueezeRatio()
        void getScreenWindow(double & oTop, double & oBottom, double & oLeft, double & oRight)
        M33d getFilmBackMatrix()
        size_t getNumOps()
        FilmBackXformOp getOp(size_t index)

    cdef cppclass IPolyMesh(IObject):
        IPolyMeshSchema getSchema()

    cdef cppclass ICameraSchema:
        CameraSample getValue(ISampleSelector iSS);

    cdef cppclass ICamera(IObject):
        pass

    cdef cppclass IArchive:
        IObject getTop()
        bool valid();

    cdef void GetArchiveStartAndEndTime(IArchive &iArchive, double &oStartTime, double & oEndTime)
