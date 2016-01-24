#ifndef CORE_H
#define CORE_H

#include <Alembic/AbcGeom/All.h>
#include <Alembic/AbcCoreFactory/All.h>

#include <string>
#include <vector>

using namespace Alembic::AbcGeom;
namespace AbcF = ::Alembic::AbcCoreFactory;

IArchive open_abc(std::string path);
void read_objects(IObject object, std::vector<IPolyMesh> &mesh_list, std::vector<ICamera> &camera_list);
M44d get_final_matrix(const IObject &iObj, chrono_t seconds);
IPolyMeshSchema::Sample get_mesh_sampler(IPolyMesh mesh, double seconds);

IV2fGeomParam::Sample get_iv2_sampler(IV2fGeomParam param, double seconds);
IN3fGeomParam::Sample get_in3_sampler(IN3fGeomParam param, double seconds);
CameraSample get_camera_sampler(ICamera camera, double seconds);

int get_size_p3f(P3fArraySamplePtr &item);
float * get_pointer_p3f(P3fArraySamplePtr &item);

int get_size_n3f(N3fArraySamplePtr &item);
float * get_pointer_n3f(N3fArraySamplePtr &item);

int get_size_v2f(V2fArraySamplePtr &item);
float * get_pointer_v2f(V2fArraySamplePtr &item);

int get_size_int32(Int32ArraySamplePtr &item);
int *get_pointer_int32(Int32ArraySamplePtr &item);

int get_size_uint32(UInt32ArraySamplePtr &item);
unsigned int *get_pointer_uint32(UInt32ArraySamplePtr &item);


#endif // CORE_H
