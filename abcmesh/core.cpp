#include "core.h"

IArchive open_abc(std::string path)
{
    AbcF::IFactory factory;
    factory.setPolicy(Abc::ErrorHandler::kNoisyNoopPolicy);
    AbcF::IFactory::CoreType coreType;
    IArchive archive = factory.getArchive(path, coreType);
    return archive;
}

void read_objects(IObject object, std::vector<IPolyMesh> &mesh_list, std::vector<ICamera> &camera_list)
{
    const size_t child_count = object.getNumChildren();
    for (size_t i = 0; i < child_count; ++i) {
        const ObjectHeader& child_header = object.getChildHeader(i);
        if (IPolyMesh::matches(child_header)) {
            mesh_list.push_back(IPolyMesh(object, child_header.getName()));
        }
        if (ICamera::matches(child_header)) {
               camera_list.push_back(ICamera(object, child_header.getName()));
       }

        read_objects(object.getChild(i), mesh_list, camera_list);
    }

}

static void accumXform( M44d &xf, const IObject obj, chrono_t seconds )
{
    if (IXform::matches( obj.getHeader()))  {
        IXform x(obj, kWrapExisting);
        XformSample xs;
        ISampleSelector sel(seconds);
        x.getSchema().get(xs, sel);
        xf *= xs.getMatrix();
    }
}


M44d get_final_matrix(const IObject &iObj, chrono_t seconds)
{
    M44d xf;
    xf.makeIdentity();
    IObject parent = iObj.getParent();

    while (parent) {
        accumXform(xf, parent, seconds);
        parent = parent.getParent();
    }

    return xf;
}

IPolyMeshSchema::Sample get_mesh_sampler(IPolyMesh mesh, double seconds)
{
	IPolyMeshSchema schema = mesh.getSchema();
	ISampleSelector sel(seconds);
	IPolyMeshSchema::Sample sampler;
	schema.get(sampler, sel);
	return sampler;
}

IV2fGeomParam::Sample get_iv2_sampler(IV2fGeomParam param, double seconds)
{
	 ISampleSelector sel(seconds);
	 IV2fGeomParam::Sample sample(param.getIndexedValue(sel));
	 return sample;
}
IN3fGeomParam::Sample get_in3_sampler(IN3fGeomParam param, double seconds)
{
	ISampleSelector sel(seconds);
	IN3fGeomParam::Sample sample(param.getIndexedValue(sel));
	 return sample;
}

CameraSample get_camera_sampler(ICamera camera, double seconds)
{
	ISampleSelector sel(seconds);
	ICameraSchema schema = camera.getSchema();
	return schema.getValue(sel);
}

int get_size_p3f(P3fArraySamplePtr &item)
{
	return item->size();
}
float * get_pointer_p3f(P3fArraySamplePtr &item)
{
	return (float*)&item->get()[0];
}

int get_size_n3f(N3fArraySamplePtr &item)
{
	return item->size();
}
float * get_pointer_n3f(N3fArraySamplePtr &item)
{
	return (float*)&item->get()[0];
}


int get_size_v2f(V2fArraySamplePtr &item)
{
	return item->size();
}
float * get_pointer_v2f(V2fArraySamplePtr &item)
{
	return (float*)&item->get()[0];
}

int get_size_int32(Int32ArraySamplePtr &item){
	return item->size();
}
int * get_pointer_int32(Int32ArraySamplePtr &item)
{
	return (int*)&item->get()[0];
}

int get_size_uint32(UInt32ArraySamplePtr &item)
{
	return item->size();
}

unsigned int *get_pointer_uint32(UInt32ArraySamplePtr &item)
{
	return (unsigned int*)&item->get()[0];
}
