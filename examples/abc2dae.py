import abcmesh
import sys
import numpy as np
import time

import collada
import pdb

abc = abcmesh.open(sys.argv[1])

dae = collada.Collada()
effect = collada.material.Effect("effect0", [], "phong", diffuse=(1,0,0), specular=(0,1,0))
mat = collada.material.Material("material0", "mymaterial", effect)
dae.effects.append(effect)
dae.materials.append(mat)

matnode = collada.scene.MaterialNode("materialref", mat, inputs=[])

nodes = []

for abc_mesh in abc.mesh_list[:1]:
    #continue
    start = time.time()

    face_indices = np.array(abc_mesh.face_indices)
    face_counts = np.array(abc_mesh.face_counts)
    vertices = np.array(abc_mesh.vertices)

    normal_indices = np.array(abc_mesh.normal_indices)
    normal_values = np.array(abc_mesh.normal_values)

    uv_indices = np.array(abc_mesh.uv_indices)
    uv_values = np.array(abc_mesh.uv_values)

    indices = []

    current_index = 0
    for fc in face_counts:
        for i in range(fc):
            # revese order
            index = current_index + fc - i - 1
            #index = current_index + fc

            indices.extend([face_indices[index],
                            normal_indices[index],
                            uv_indices[index]])

        current_index += fc

    vert_src   = collada.source.FloatSource("verts-array",   vertices,      ('X', 'Y', 'Z'))
    normal_src = collada.source.FloatSource("normal-array",  normal_values, ('X', 'Y', 'Z'))
    uv_src     = collada.source.FloatSource("uvset1",      uv_values,     ('S', 'T'))

    geom = collada.geometry.Geometry(dae, 'geometry0', abc_mesh.name, [vert_src, normal_src, uv_src])
    input_list = collada.source.InputList()
    input_list.addInput(0, 'VERTEX',   "#verts-array")
    input_list.addInput(1, 'NORMAL',   "#normal-array")
    input_list.addInput(2, 'TEXCOORD', "#uvset1")

    poly_set = geom.createPolylist(np.array(indices), face_counts, input_list, 'materialref')
    geom.primitives.append(poly_set)
    dae.geometries.append(geom)

    geomnode = collada.scene.GeometryNode(geom, [matnode])
    node = collada.scene.Node(abc_mesh.name, children=[geomnode])

    nodes.append(node)

#cameras = []
for abc_camera in abc.camera_list[:1]:
    cam = collada.camera.PerspectiveCamera("cameraShape1",
                                           abc_camera.znear,
                                           abc_camera.zfar,
                                           xfov = abc_camera.fovx,
                                           yfov = abc_camera.fovy)

    dae.cameras.append(cam)
    camera_node  =  collada.scene.CameraNode(cam)
    mat = np.array(abc_camera.world_matrix)
    mat.shape = (16)
    transform = collada.scene.MatrixTransform(mat)
    transform_node = collada.scene.Node("%s" % abc_camera.name,
                                        children=[camera_node],
                                        transforms=[transform])

    nodes.extend([transform_node, camera_node])



myscene = collada.scene.Scene("myscene", nodes)
dae.scenes.append(myscene)
dae.scene = myscene
dae.write('test.dae')
    #triset = geom.createTriangleSet(np.array(mesh.face_indices), input_list, "materialref")
