import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import math
# from mpl_toolkits.mplot3d import Axes3D
# from quaternion import as_euler_angles
from scipy.spatial.transform import Rotation as R
# from plyfile import PlyData, PlyElement
import tkinter as tk     # from tkinter import Tk for Python 3.x
import tkinter.filedialog as fd # askopenfilename


"visualize np array of points as point cloud"
def open3DPick(file, density):
    mesh = o3d.io.read_triangle_mesh(file, density)
    "convert mesh to point cloud and optionally visualize"
    #visualize Mesh
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([.5, .5, .5])
    # o3d.visualization.draw_geometries([mesh])
    pcd = mesh.sample_points_uniformly(number_of_points=density)
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    index = vis.get_picked_points()    
    pcd_array = np.asarray(pcd.points)
    return index, pcd_array, pcd

"visualize np array of points as point cloud"
def open3DNoPick(file, density):
    mesh = o3d.io.read_triangle_mesh(file, density)
    "convert mesh to point cloud and optionally visualize"
    #visualize Mesh
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([.5, .5, .5])
    # o3d.visualization.draw_geometries([mesh])
    pcd = mesh.sample_points_uniformly(number_of_points=density)
    return pcd

"Move whole structure to origin. if else statements allow us to decide which segment we are placing"
def moveToOrigin(points, label):
    x = label[0]
    y = label[1]
    z = label[2]
    out = []
    for plane in points:
        temp = []
        # for point in plane:
        new_x = plane[0] -x
        new_y = plane[1] -y
        new_z = plane[2] -z
        temp.append(new_x)
        temp.append(new_y)
        temp.append(new_z)
        out.append(temp)
    return out

"calculate angle to move points to z axis. Could probably combine with rotate_struct_1"
def getAngle_X(centroid2):
    # get angle to rotate whole skull from the naseon now
    angle = math.atan(centroid2[2]/centroid2[1]) #is in radians
    angle = (-1*angle)
    return angle

"rotate any given points to x axis. x,y,z are lists of points, angle is the angle to rotate by "
def rotate_struct_1(points, angle):
    for plane in points:
        for value in plane:
            # Solve for new values using that angle and system of equations
            y = value[1] * math.cos(angle) - value[2] * math.sin(angle)
            z = value[1] * math.sin(angle) + value[2] * math.cos(angle)
            value[1] = y
            value[2] = z
    return points

"calculate angle to move points to z axis. Could probably combine with rotate_struct_2"
def getAngle_Z(centroid2):
    # get angle to rotate whole skull from the naseon now
    angle = math.atan(centroid2[0]/centroid2[1]) #is in radians
    # angle = (-*angle)
    return angle

"rotate any given points to z axis. x,y,z are lists of points, angle is the angle to rotate by "
def rotate_struct_2(points, angle):
    for plane in points:
        for value in plane:
            x = value[0] * math.cos(angle) - value[1] * math.sin(angle) 
            y = value[1] * math.cos(angle) + value[0] * math.sin(angle)
            value[0] = x
            value[1] = y
    return points

"find angle between 2 input planes. Need to understand which angles we really \
care about and which ones come first "
def calcPlaneAngle(a1, b1, c1, a2, b2, c2):
    d = ( a1 * a2 + b1 * b2 + c1 * c2 )
    e1 = math.sqrt( a1 * a1 + b1 * b1 + c1 * c1)
    e2 = math.sqrt( a2 * a2 + b2 * b2 + c2 * c2)
    d = d / (e1 * e2)
    angle = math.degrees(math.acos(d))
    print("Angle is " + str(angle) + " degree")
    return angle


"function generates equation of plane based on 3 points. Plane is a list of lists of xyz points. \""
def definePlaneEquation(plane, label): 
    p1 = np.array(plane[0])
    p2 = np.array(plane[1])
    p3 = np.array(plane[2])
    # These two vectors are in the plane
    v1 = p3 - p1
    v2 = p2 - p1
    # the cross product is a vector normal to the plane
    cp = np.cross(v1, v2)
    cp = cp / np.linalg.norm(cp)  # Normalize the vector

    a, b, c = cp
    # This evaluates a * x3 + b * y3 + c * z3 which equals d
    d = np.dot(cp, p3)
    print('The equation of plane {4} is {0}x + {1}y + {2}z = {3}'.format(a, b, c, d, label))

    return a,b,c

def calculate_plane_from_normal(n, x, y, z_offset=0):
    # For a plane with normal vector (A, B, C) and passing through the origin, 
    # the plane equation is: Ax + By + Cz = 0
    # We can solve for z: z = -(Ax + By) / C
    z = -(n[0]*x + n[1]*y) / n[2] + z_offset
    return z

"data analysis function"
def get_euler_angles(planeA, planeB):
    # Normalize the plane normal vectors
    planeA = planeA / np.linalg.norm(planeA)
    planeB = planeB / np.linalg.norm(planeB)

    # Compute the axis of rotation
    # This works by finding the plane spanned by plane A and plane B
    # i.e., if we put the tail ends of the two vectors together, we find the plane that they make as the vectors extend to infinity
    # the cross product of then represents the normal vector of this plane, which is the axis we need to rotate by 
    axis = np.cross(planeA, planeB)
    axis = axis / np.linalg.norm(axis)

    # Compute the angle of rotation
    # dot product of two vectors is by definition equal to the cos of theta between them
    # dot product of planeA, planeB = cos(theta) --> rearrange to solve for theta
    angle = np.arccos(np.dot(planeA, planeB))

    # Skew-symmetric matrix  is a square matrix whose transpose equals its own negative. 
    # need this to solve rodrigyez formula to get rotation matrix    
    s_s_m = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])

    # Compute the rotation matrix R using rodriguez formula
    # I dont really understand the math behind this, but it works and is proven for like 200 years
    # https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
    r_mat = np.eye(3) + np.sin(angle) * s_s_m + (1 - np.cos(angle)) * np.dot(s_s_m, s_s_m)

    # Compute Euler angles using the rotation matrix
    # This is from ChatGPT  euler angles suffer from gimbal lock so computing the singluar is a way to prevent this (in the else statement, set z=0 as there are infinte rotations possible otherwise)
    # If not singular represents the mathetical way to convert rotation matrix to euler angles using zyx convention as is standard
    sy = np.sqrt(r_mat[0, 0] ** 2 + r_mat[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(r_mat[2, 1], r_mat[2, 2])
        y = np.arctan2(-r_mat[2, 0], sy)
        z = np.arctan2(r_mat[1, 0], r_mat[0, 0])
    else:
        x = np.arctan2(-r_mat[1, 2], r_mat[1, 1])
        y = np.arctan2(-r_mat[2, 0], sy)
        z = 0

    # return the angles of interest and rotation matrix to later calculate the rotated plane
    return x, y, z, r_mat

#function to plot the planes to visualize results
def plot_planes(planeA, planeB, planeC):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    x = np.linspace(-1, 1, 10)
    y = np.linspace(-1, 1, 10)
    x, y = np.meshgrid(x, y)

    zA = calculate_plane_from_normal(planeA, x, y)
    zB = calculate_plane_from_normal(planeB, x, y)
    zC = calculate_plane_from_normal(planeC, x, y, z_offset=1)

    ax.plot_surface(x, y, zA, alpha=0.5, rstride=100, cstride=100, color='blue')
    ax.plot_surface(x, y, zB, alpha=0.5, rstride=100, cstride=100, color='red')
    ax.plot_surface(x, y, zC, alpha=0.5, facecolors='green', rstride=100, cstride=100, shade=False)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()
   
def fileSelect():
    root = tk.Tk()
    files = fd.askopenfilenames(parent=root, title='Choose a file')
    file_list = list(files)
    print(file_list)
    return (file_list)

def roundPlanes(a,b,c):
    a = round(a/3, 2)
    b = round(b/3, 2)
    c = round(c/3, 2)
    equation = (f'{a}x + {b}y + {c}z')
    return(equation)

def create_triangle_mesh(points, color=[1, 0, 0]):
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(points)
    mesh.triangles = o3d.utility.Vector3iVector([[0, 1, 2]])
    mesh.compute_triangle_normals()
    mesh.paint_uniform_color(color)
    pcd = mesh.sample_points_uniformly(number_of_points=100000)
    
    return pcd

def main_loop(preop, postop, density):
    preop_prox_seg_index, preop_arr, preop_cloud = open3DPick(preop, density)
    target_1_A = preop_arr[preop_prox_seg_index[0]]
    target_1_B = preop_arr[preop_prox_seg_index[1]]
    target_1_C = preop_arr[preop_prox_seg_index[2]] 
    target_1_plane = [target_1_A,target_1_B,target_1_C] 
    
    target_2_A = preop_arr[preop_prox_seg_index[3]]
    target_2_B = preop_arr[preop_prox_seg_index[4]]
    target_2_C = preop_arr[preop_prox_seg_index[5]]
    target_2_plane = [target_2_A,target_2_B,target_2_C] 
    
    plan_distance_seg_1 = np.linalg.norm(target_1_A - target_2_A)
    
    target_3_A = preop_arr[preop_prox_seg_index[6]]
    target_3_B = preop_arr[preop_prox_seg_index[7]]
    target_3_C = preop_arr[preop_prox_seg_index[8]] 
    target_3_plane = [target_3_A,target_3_B,target_3_C] 
    
    plan_distance_seg_2 = np.linalg.norm(target_2_A - target_3_A)

    if segments == 3:
        target_4_A = preop_arr[preop_prox_seg_index[9]]
        target_4_B = preop_arr[preop_prox_seg_index[10]]
        target_4_C = preop_arr[preop_prox_seg_index[11]]
        target_4_plane = [target_4_A,target_4_B,target_4_C] 
        plan_distance_seg_3 = np.linalg.norm(target_3_A - target_4_A)


    postop_prox_seg_index, postop_arr, postop_cloud = open3DPick(postop, density)
    postOpProxA_1 = postop_arr[postop_prox_seg_index[0]]
    postOpProxB_1 = postop_arr[postop_prox_seg_index[1]]
    postOpProxC_1 = postop_arr[postop_prox_seg_index[2]]
    plane_post_prox_1 = [postOpProxA_1,postOpProxB_1,postOpProxC_1] 
    
    postOpDistA_1 = postop_arr[postop_prox_seg_index[3]] 
    postOpDistB_1 = postop_arr[postop_prox_seg_index[4]]   
    postOpDistC_1 = postop_arr[postop_prox_seg_index[5]] 
    plane_post_dist_1 = [postOpDistA_1,postOpDistB_1,postOpDistC_1] 

    post_distance_seg_1 = np.linalg.norm(postOpProxA_1 - postOpDistA_1)

    postOpProxA_2 = postop_arr[postop_prox_seg_index[6]]
    postOpProxB_2 = postop_arr[postop_prox_seg_index[7]]
    postOpProxC_2 = postop_arr[postop_prox_seg_index[8]]
    plane_post_prox_2 = [postOpProxA_2,postOpProxB_2,postOpProxC_2] 
    
    postOpDistA_2 = postop_arr[postop_prox_seg_index[9]] 
    postOpDistB_2 = postop_arr[postop_prox_seg_index[10]]   
    postOpDistC_2 = postop_arr[postop_prox_seg_index[11]] 
    plane_post_dist_2 = [postOpDistA_2,postOpDistB_2,postOpDistC_2] 
    
    post_distance_seg_2 = np.linalg.norm(postOpProxA_2 - postOpDistA_2)
    
    if segments == 3:
        postOpProxA_3 = postop_arr[postop_prox_seg_index[12]]
        postOpProxB_3 = postop_arr[postop_prox_seg_index[13]]
        postOpProxC_3 = postop_arr[postop_prox_seg_index[14]]
        plane_post_prox_3 = [postOpProxA_3,postOpProxB_3,postOpProxC_3] 
        
        postOpDistA_3 = postop_arr[postop_prox_seg_index[15]] 
        postOpDistB_3 = postop_arr[postop_prox_seg_index[16]]   
        postOpDistC_3 = postop_arr[postop_prox_seg_index[17]] 
        plane_post_dist_3 = [postOpDistA_3,postOpDistB_3,postOpDistC_3] 
        post_distance_seg_3 = np.linalg.norm(postOpProxA_3 - postOpDistA_3)

    
    segemnt_1_plan = [target_1_plane,target_2_plane]
    segment_1_post = [plane_post_prox_1,plane_post_dist_1]
    
    segment_1 = [segemnt_1_plan, segment_1_post]
    
    segment_2_plan = [target_2_plane,target_3_plane]
    segment_2_post = [plane_post_prox_2,plane_post_dist_2]
    
    segment_2 = [segment_2_plan, segment_2_post]

    final_segments = [segment_1, segment_2 ]
    
    if segments==3:
        segment_3_plan = [target_3_plane, target_4_plane]
        segment_3_post = [plane_post_prox_3,plane_post_dist_3]
        segment_3 = [segment_3_plan, segment_3_post]
        final_segments = [segment_1, segment_2, segment_3]

        ###started editing here for registration stuff

    for segment in final_segments:
        ### PLOT HERE segment[0] and segment[1]
        translated_segment_plan = moveToOrigin(segment[0], segment[0][0][0])
        angle1_preop = getAngle_X(translated_segment_plan[1][1])

        translated_segment_postop = moveToOrigin(segment[1], segment[1][0][0])
        angle1_postop = getAngle_X(translated_segment_postop[1][1])
        ### PLOT HERE segment[0] and segment[1]

        rotate_1_preop = rotate_struct_1(translated_segment_plan, angle1_preop)
        rotate_1_postop = rotate_struct_1(translated_segment_postop, angle1_postop)
        ### PLOT HERE rotate_1_preop and rotate_1_postop
        ### align on axis
        angle2_preop = getAngle_Z(rotate_1_preop[0][1])
        angle2_postop = getAngle_Z(rotate_1_postop[1][1])

        print('Fixing on other axis ')
        rotate_2_preop = rotate_struct_2(rotate_1_preop, angle2_preop)
        rotate_2_postop = rotate_struct_2(rotate_1_postop, angle2_postop)
        ### PLOT HERE rotate_2_preop and rotate_2_postop
        
        
        " put segment 1 (plane 1) on origin and rotate centroid so that all other points \
        this is just doing the movement of the centroid line line up"

        #equation of plane of points around 1st centroid with sengment 1 on origin 
        preProxA1,preProxB1,preProxC1 = definePlaneEquation(rotate_2_preop[0],'one')
        preDistA1,preDistB1,preDistC1 = definePlaneEquation(rotate_2_preop[1],'one')

        postProxA1,postProxB1,postProxC1 = definePlaneEquation(rotate_2_postop[0],'one')
        postDistA1,postDistB1,postDistC1 = definePlaneEquation(rotate_2_postop[1],'one')

    ######## normal plane should be xz plane- used three random points from xz plane. Need d value 
        # (distance from xz plane.) In this case, d = 1/2 the y component of the segment not on the origin

        y_pre = rotate_2_preop[1][1][1] /2
        y_post = rotate_2_postop[1][1][1] /2
        ######## subtract normal plane - plane 1 and normal plane - plane 2 to get angles of interest
        normMinusPlaneProxPre = calcPlaneAngle(preProxA1,preProxB1,preProxC1,0,y_pre,0)
        normMinusPlaneDistPre = calcPlaneAngle(preDistA1,preDistB1,preDistC1,0,y_pre,0)

        normMinusPlaneProxPost = calcPlaneAngle(postProxA1,postProxB1,postProxC1,0,y_post,0)
        normMinusPlaneDistPost = calcPlaneAngle(postDistA1,postDistB1,postDistC1,0,y_post,0)

        deltaProx = normMinusPlaneProxPost-normMinusPlaneProxPre
        deltaDist = normMinusPlaneDistPost-normMinusPlaneDistPre

        print(deltaProx)
        print(deltaDist)

        v1 = np.array([preProxA1,preProxB1,preProxC1])
        v2 = np.array([postProxA1,postProxB1,postProxC1])
        
        phi1, theta1, psi1, r1 = get_euler_angles(v2, v1)
        v3 = np.dot(r1, v2)
        a_prox, b_prox, c_prox = v3
        plot_planes(v1, v2, v3)
        
        v4 = np.array([preDistA1,preDistB1,preDistC1])
        v5 = np.array([postDistA1,postDistB1,postDistC1])
        phi2,theta2, psi2, r2 = get_euler_angles(v5, v4)
        v6 = np.dot(r2, v5)
        a_dist, b_dist, c_dist = v6
        plot_planes(v4, v5, v6)

        pre_prox_equation = roundPlanes(preProxA1, preProxB1, preProxC1)
        pre_dist_equation = roundPlanes(preDistA1, preDistB1, preDistC1)
        
        post_prox_equation = roundPlanes(postProxA1, postProxB1, postProxC1)
        post_dist_equation = roundPlanes(postDistA1, postDistB1, postDistC1)
        
        proximal_new_equation = roundPlanes(a_prox, b_prox, c_prox)
        distal_new_equation = roundPlanes(a_dist, b_dist, c_dist)
        
        pre_mesh_prox = create_triangle_mesh(segment[0][0], color = [1,0,0])
        pre_mesh_dist = create_triangle_mesh(segment[0][1], color = [0,0,1])

        post_mesh_prox = create_triangle_mesh(segment[1][0], color = [1,0,0])
        post_mesh_dist = create_triangle_mesh(segment[1][1],color = [0,0,1])

        ply_Planes.append(pre_mesh_prox)
        ply_Planes.append(pre_mesh_dist)
        ply_Planes.append(post_mesh_prox)
        ply_Planes.append(post_mesh_dist)
        
        angles.append(round(np.degrees(phi1), 2))
        angles.append(round(np.degrees(theta1),2 ))
        angles.append(round(np.degrees(psi1),2))
        
        angles.append(round(np.degrees(phi2),2))
        angles.append(round(np.degrees(theta2),2))
        angles.append(round(np.degrees(psi2),2))
        
        original_planes.append(pre_prox_equation)
        original_planes.append(pre_dist_equation)
        original_planes.append(post_prox_equation)
        original_planes.append(post_dist_equation)
        rotated_planes.append(proximal_new_equation)
        rotated_planes.append(distal_new_equation)
        # distances.append(distance_1)
        # distances.append(distance_2)
        
    return preop_cloud, postop_cloud
   
density = 1000000

file_list = fileSelect()

preop = file_list[0]
postop = file_list[1]

# segments = input('select number of segments')
segments = 2
print(f'Number of segments is  {segments}')

out = '/Users/georgienahass/Desktop/'
filename = 'osteoplane.csv'

original_planes = []
rotated_planes = []
angles = []
ply_Planes = []
distances = []
if segments == 3:
    with open(out + filename,'w') as fout:
        line = ['segment 1 proximal preop plane','segment 1 distal preop plane','segment 1 proximal postop plane','segment 1 distal postop plane',
            'segment 2 proximal preop plane','segment 2 distal preop plane','segment 2 proximal postop plane','segment 2 distal postop plane',
            'segment 3 proximal preop plane','segment 3 distal preop plane','segment 3 proximal postop plane','segment 3 distal postop plane',
            'NEW segment 1 proximal postop plane','NEW segment 1 distal postop plane',
            'NEW segment 2 proximal postop plane','NEW segment 2 distal postop plane',   
            'NEW segment 3 proximal postop plane','NEW segment 3 distal postop plane',  
            'phi proximal segment 1','theta proximal segment 1','psi proximal segment 1',
            'phi distal segment 1','theta distal segment 1','psi distal segment 1',
            'phi proximal segment 2','theta proximal segment 2','psi proximal segment 2',
            'phi distal segment 2','theta distal segment 2','psi distal segment 2',
            'phi proximal segment 3','theta proximal segment 3','psi proximal segment 3',
            'phi distal segment 3','theta distal segment 3','psi distal segment 3',
            'distance segment 1 preop', 'distance segment 2 preop', 'distance segment 3 preop',
            'distance segment 1 postop', 'distance segment 2 postop', 'distance segment 3 postop',            ] 
        fout.write("%s\n" % ",".join(line))
    
    preop_arr, postop_arr = main_loop(preop, postop, density)
    
    low_res_preop = open3DNoPick(preop, 100000)
    low_res_posteop = open3DNoPick(postop, 100000)

    o3d.visualization.draw_geometries([low_res_preop, ply_Planes[0], ply_Planes[1] ,ply_Planes[4] , ply_Planes[5] ,ply_Planes[8] , ply_Planes[9]])
    o3d.visualization.draw_geometries([low_res_posteop, ply_Planes[2], ply_Planes[3] ,ply_Planes[6] , ply_Planes[7] ,ply_Planes[10] , ply_Planes[11]])

    with open(out + filename,'a') as fout:
        line = [original_planes[0], original_planes[1], original_planes[2], original_planes[3],  
            original_planes[4], original_planes[5], original_planes[6], original_planes[7], 
            original_planes[8], original_planes[9], original_planes[10], original_planes[11], 
            rotated_planes[0],  rotated_planes[1],  
            rotated_planes[2],  rotated_planes[3], 
            rotated_planes[4],  rotated_planes[5], 
            angles[0], angles[1],  angles[2],
            angles[3], angles[4],  angles[5],
            angles[6], angles[7],  angles[8],
            angles[9], angles[10],  angles[11],
            angles[12], angles[13],  angles[14],
            angles[15], angles[16],  angles[17],
            distances[0], distances[2], distances[4],
            distances[1], distances[3], distances[5]]
        line = [str(l) for l in line]
        fout.write("%s\n" % ",".join(line))
        
if segments == 2:
    with open(out + filename,'w') as fout:
        line = ['segment 1 proximal preop plane','segment 1 distal preop plane','segment 1 proximal postop plane','segment 1 distal postop plane',
            'segment 2 proximal preop plane','segment 2 distal preop plane','segment 2 proximal postop plane','segment 2 distal postop plane',
            'NEW segment 1 proximal postop plane','NEW segment 1 distal postop plane',
            'NEW segment 2 proximal postop plane','NEW segment 2 distal postop plane',   
            'phi proximal segment 1','theta proximal segment 1','psi proximal segment 1',
            'phi distal segment 1','theta distal segment 1','psi distal segment 1',
            'phi proximal segment 2','theta proximal segment 2','psi proximal segment 2',
            'phi distal segment 2','theta distal segment 2','psi distal segment 2',
            'distance segment 1 preop', 'distance segment 2 preop', 
            'distance segment 1 postop', 'distance segment 2 postop' ]
        fout.write("%s\n" % ",".join(line))
        
    _, _ = main_loop(preop, postop, density)
    
    low_res_preop = open3DNoPick(preop, 100000)
    low_res_posteop = open3DNoPick(postop, 100000)
    
    
    o3d.visualization.draw_geometries([low_res_preop, ply_Planes[0], ply_Planes[1], ply_Planes[4], ply_Planes[5]])
    o3d.visualization.draw_geometries([low_res_posteop, ply_Planes[2], ply_Planes[3], ply_Planes[6], ply_Planes[7]])

    with open(out + filename,'a') as fout:
            line = [original_planes[0], original_planes[1], original_planes[2], original_planes[3],  
                original_planes[4], original_planes[5], original_planes[6], original_planes[7], 
                rotated_planes[0],  rotated_planes[1],  
                rotated_planes[2],  rotated_planes[3], 
                angles[0], angles[1],  angles[2],
                angles[3], angles[4],  angles[5],
                angles[6], angles[7],  angles[8],
                angles[9], angles[10],  angles[11],
                distances[0], distances[2],
                distances[1], distances[3]]
            line = [str(l) for l in line]
            fout.write("%s\n" % ",".join(line))






# "data analysis function"
# def getData(V1_func, V2_func):
#     theta = np.arccos(np.dot(V1_func, V2_func)/ (np.linalg.norm(V1_func) * np.linalg.norm(V2_func)))

#     axis = np.cross(V1_func, V2_func)
#     axis1 = axis / np.linalg.norm(axis)
 
#     rotVec = theta * axis1
#     rot = R.from_rotvec(rotVec)
#     rot_matrix = rot.as_matrix()
#     rotated_vector_norm  = rot.apply(V2_func)
#     rotated_vector_inverse  = rot.apply(V2_func, inverse=True)

#     theta_norm_init = np.cross(rotated_vector_norm,V1_func)  
#     theta_norm = abs(np.rint(theta_norm_init.astype(int)))
    
#     theta_inverse_init = np.cross(rotated_vector_inverse,V1_func)  
#     theta_inverse = abs(np.rint(theta_inverse_init).astype(int))

#     if np.all(theta_norm==0) == True:
#         print('theta norm 0')
#         print(V1_func)
#         print(V2_func)
#         print(rotated_vector_norm)
#         angles_var = rot.as_euler('xyz', degrees=True)
#         # roll, pitch, yaw = rotation_matrix_to_RPY(angles)
#         print('PRINTING ANGLES PRINT ANGLES')
#         print(angles_var)
#         return rotated_vector_norm[0], rotated_vector_norm[1], rotated_vector_norm[2],  angles_var[0], angles_var[1], angles_var[2]
    
#     elif np.all(theta_inverse==0) == True:
#         print('theta inverse 0')
#         print(V1_func)
#         print(V2_func)
#         print(rotated_vector_inverse)
#         angles_var = rot.as_euler('xyz',degrees=True)
#         # roll, pitch, yaw = rotation_matrix_to_RPY(angles)
#         print('PRINTING ANGLES PRINT ANGLES')
#         print(angles_var)
#         print(angles_var[0], angles_var[2])
#         return rotated_vector_inverse[0], rotated_vector_inverse[1], rotated_vector_inverse[2],angles_var[0], angles_var[1], angles_var[2]








# if segments == 2:
#     with open(out + filename,'w') as fout:
#         line = ['segment 1 proximal preop plane','segment 1 distal preop plane','segment 1 proximal postop plane','segment 1 distal postop plane',
#             'segment 2 proximal preop plane','segment 2 distal preop plane','segment 2 proximal postop plane','segment 2 distal postop plane',
#             'NEW segment 1 proximal postop plane','NEW segment 1 distal postop plane',
#             'NEW segment 2 proximal postop plane','NEW segment 2 distal postop plane',   
#             'phi proximal segment 1','theta proximal segment 1','psi proximal segment 1',
#             'phi distal segment 1','theta distal segment 1','psi distal segment 1',
#             'phi proximal segment 2','theta proximal segment 2','psi proximal segment 2',
#             'phi distal segment 2','theta distal segment 2','psi distal segment 2']
#         fout.write("%s\n" % ",".join(line))

#     for i in range(segments):
#         preop_prox_seg_index = []
#         preop_prox_seg_index, preop_arr = open3DPick(preop, density)
            
#         preOpProxA = preop_arr[preop_prox_seg_index[0]]
#         preOpProxB = preop_arr[preop_prox_seg_index[1]]
#         preOpProxC = preop_arr[preop_prox_seg_index[2]]
#         preOpDistA = preop_arr[preop_prox_seg_index[3]]
#         preOpDistB = preop_arr[preop_prox_seg_index[4]]
#         preOpDistC = preop_arr[preop_prox_seg_index[5]]

#         plane_1_pre = [preOpProxA,preOpProxB,preOpProxC] 
#         plane_2_pre = [preOpDistA,preOpDistB,preOpDistC] 

#         preOpPoints = [preOpProxA, preOpDistA]

#         postop_prox_seg_index, postop_arr = open3DPick(postop, density)
#         postOpProxA = postop_arr[postop_prox_seg_index[0]]
#         postOpProxB = postop_arr[postop_prox_seg_index[1]]
#         postOpProxC = postop_arr[postop_prox_seg_index[2]]
#         postOpDistA = postop_arr[postop_prox_seg_index[3]] 
#         postOpDistB = postop_arr[postop_prox_seg_index[4]]   
#         postOpDistC = postop_arr[postop_prox_seg_index[5]] 
        
#         plane_1_post = [postOpProxA,postOpProxB,postOpProxC] 
#         plane_2_post = [postOpDistA,postOpDistB,postOpDistC] 

#         postOpPoints = [postOpProxA, postOpDistA]

#         " put segment 1 (plane 1) on origin and rotate centroid so that all other points \
#         this is just doing the movement of the centroid line line up"

#         #equation of plane of points around 1st centroid with sengment 1 on origin 
#         preProxA1,preProxB1,preProxC1,preProxD1,preProxP1,preProxP2,preProxP3 = definePlaneEquation(plane_1_pre,'one')
#         preDistA1,preDistB1,preDistC1,preDistD1,preDistP1,preDistP2,preDistP3 = definePlaneEquation(plane_2_pre,'one')

#         postProxA1,postProxB1,postProxC1,postProxD1,postProxP1,postProxP2,postProxP3 = definePlaneEquation(plane_1_post,'one')
#         postDistA1,postDistB1,postDistC1,postDistD1,postDistP1,postDistP2,postDistP3 = definePlaneEquation(plane_2_post,'one')

#         v1 = np.array([preProxA1,preProxB1,preProxC1])
#         v2 = np.array([postProxA1,postProxB1,postProxC1])
        
#         a_prox,b_prox,c_prox, phi1, theta1, psi1 = getData(v1, v2)
        
#         print(phi1, theta1, psi1)
        
#         v3 = np.array([preDistA1,preDistB1,preDistC1])
#         v4 = np.array([postDistA1,postDistB1,postDistC1])

#         a_dist,b_dist,c_dist, phi2,theta2, psi2 = getData(v3, v4)
        
#         #final data preparation for storage  no more data processing
        
#         preProxA1 = round(preProxA1/3, 2)
#         preProxB1 = round(preProxB1/3, 2)
#         preProxC1 = round(preProxC1/3, 2)
#         pre_prox_equation = (f'{preProxA1}x + {preProxB1}y + {preProxC1}z = -{preProxD1}')

#         preDistA1 = round(preDistA1/3, 2)
#         preDistB1 = round(preDistB1/3, 2)
#         preDistC1 = round(preDistC1/3, 2)
#         pre_dist_equation = (f'{preDistA1}x + {preDistB1}y + {preDistC1}z = -{preDistD1}')
        
#         postProxA1 = round(postProxA1/3, 2)
#         postProxB1 = round(postProxB1/3, 2)
#         postProxC1 = round(postProxC1/3, 2)
#         post_prox_equation = (f'{postProxA1}x + {postProxB1}y + {postProxC1}z = -{postProxD1}')

        
#         postDistA1 = round(postDistA1/3, 2)
#         postDistB1 = round(postDistB1/3, 2)
#         postDistC1 = round(postDistC1/3, 2)
#         post_dist_equation = (f'{postDistA1}x + {postDistB1}y + {postDistC1}z = -{postDistD1}')
        
#         a_prox = round(a_prox/3, 2)
#         b_prox = round(b_prox/3, 2)
#         c_prox = round(c_prox/3, 2)
#         proximal_new_equation = (f'{a_prox}x + {b_prox}y + {c_prox}z')

#         a_dist = round(a_dist/3, 2)
#         b_dist = round(b_dist/3, 2)
#         c_dist = round(c_dist/3, 2)  
#         distal_new_equation = (f'{a_dist}x + {b_dist}y + {c_dist}z')
        
#         print('LOOK here pre, post, correct, prox dist')
#         print('proximal xyz')
#         print(preOpProxA)
#         print('proximal')
#         print(pre_prox_equation)
#         print(post_prox_equation)
#         print(proximal_new_equation)
#         print('distal')
#         print('distal xyz')
#         print(preOpDistA)
#         print(pre_dist_equation)
#         print(post_dist_equation)
#         print(distal_new_equation)

    
#         angles.append(round((phi1), 2))
#         angles.append(round((theta1),2 ))
#         angles.append(round((psi1),2))
        
#         angles.append(round(math.degrees(phi2),2))
#         angles.append(round(math.degrees(theta2),2))
#         angles.append(round(math.degrees(psi2),2))
        
#         original_planes.append(pre_prox_equation)
#         original_planes.append(pre_dist_equation)
#         original_planes.append(post_prox_equation)
#         original_planes.append(post_dist_equation)
#         rotated_planes.append(proximal_new_equation)
#         rotated_planes.append(distal_new_equation)



# The new equation of the plane is given by a_new*x + b_new*y + c_new*z + d = 0



