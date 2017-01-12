import abcmesh
import numpy as np
from numpy.linalg import inv
import math

def create_camera(camera, seconds=0.0):
    """
    convert abc camera to rib camera
    """

    camera.time = seconds
    world_matrix = np.array(camera.world_matrix).T

    scale = np.array( ((1, 0, 0, 0),
                       (0, 1, 0, 0),
                       (0, 0,-1, 0),
                       (0, 0, 0, 1)));
    world_matrix = np.dot(scale, world_matrix)
    world_matrix = inv(world_matrix)
    world_matrix.shape = (16)

    ratio = 1920.0/1080
    fovy = camera.fovy

    rib = """
    Projection "perspective" "fov" [{fov}]
    Transform [{transfrom}]
    """.format(fov=fovy, transfrom=" ".join([str(item) for item in world_matrix]))
    return rib

def create_subdivision_mesh(mesh, seconds=0.0):
    """
    converts mesh to catmull-clark SubdivisionMesh
    """
    mesh.time = seconds
    print mesh.name
    yield 'SubdivisionMesh "catmull-clark"'
    yield ' [%s]' % ' '.join(map(str, np.array(mesh.face_counts)))
    yield ' [%s]' % ' '.join(map(str, np.array(mesh.face_indices)))
    yield ' ["interpolateboundary"] [0 0] [] []'
    yield ' "P" [%s]' % '  '.join('%f %f %f' % tuple(P) for P in np.array(mesh.vertices))

    uv_values = mesh.uv_values

    if uv_values:
        uv_indices = mesh.uv_indices
        yield ' "facevarying float s" [%s]' % ' '.join(str(uv_values[i][0]) for i in uv_indices)
        yield ' "facevarying float t" [%s]' % ' '.join(str(1.0 - uv_values[i][1]) for i in uv_indices)

if __name__ == "__main__":
    import sys
    import subprocess
    abc = abcmesh.open(sys.argv[1])

    cameras = []
    meshs = []

    f = open("out.rib", 'w')
    #f.write('Display "output.tif" "TIFF" "rgba"\n')
    f.write('Quantize "rgba" 0 0 0 0\n')
    f.write('Display "output.exr" "openexr" "rgba" "string type" "half" "string compression" "zips"\n')
    f.write('Format %d %d 1\n' % (1920, 1080))
    seconds = 0
    f.write(create_camera(abc.camera_list[0], seconds))
    f.write('WorldBegin\n')
    f.write('Surface "abc_shader"\n')
    for mesh in abc.mesh_list[:]:
        f.write("\n".join(create_subdivision_mesh(mesh, seconds)))
        f.write("\n")

    f.write('WorldEnd\n')
    f.close()

    f = open("abc_shader.sl", 'w')
    f.write("""surface abc_shader(){Ci = color(s, t, 0);}""")
    # f.write("""surface abc_shader(){Ci = color texture("out.tex");}""")
    f.close()

    subprocess.check_call(['shader', "abc_shader.sl"])
    # subprocess.check_call(['prman', '-progress', "out.rib"])
    subprocess.check_call(['prman', '-d', 'it', '-progress', "out.rib"])
