import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import math
# from mpl_toolkits.mplot3d import Axes3D
from quaternion import as_euler_angles
from scipy.spatial.transform import Rotation as R
from plyfile import PlyData, PlyElement
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
    return index, pcd_array

"shitty function to make scatter plots. Tons of problems "
def plotter(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,aa): #:#  ,k):
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    x = [a[0], b[0], c[0], d[0], e[0],f[0],g[0],h[0],j[0],i[0],k[0],l[0]]  #],m[0]]#,n[0],o[0],p[0]]#,k[0]]
    y = [a[1], b[1], c[1],d[1], e[1],f[1],g[1],h[1],j[1],i[1],k[1],l[1]]    #,m[1]]#,n[1],o[1],p[1]            ]#,k[1]]
    z = [a[2], b[2], c[2] ,d[2], e[2],f[2],g[2],h[2],j[2],i[2],k[2],l[2]]  #,m[2]]#,n[2],o[2],p[2]       ]#,k[2]]

    x2 = [m[0], n[0], o[0], p[0], q[0], r[0], s[0], t[0], u[0], v[0], w[0], aa[0]]
    y2 = [m[1], n[1], o[1], p[1], q[1], r[1], s[1], t[1], u[1], v[1], w[1], aa[1]]
    z2 = [m[2], n[2], o[2], p[2], q[2], r[2], s[2], t[2], u[2], v[2], w[2], aa[2]]


    x1 = [a[0], d[0], g[0], j[0]]
    y1 = [a[1], d[1], g[1], j[1]]
    z1 = [a[2], d[2], g[2], j[2]]
    
    x4 = [m[0], p[0], s[0], v[0]]
    y4 = [m[1], p[1], s[1], v[1]]
    z4 = [m[2], p[2], s[2], v[2]]
    ax.scatter(x,y,z, c='r',s=100)
    ax.scatter(x2,y2,z2, c='b',s=100)

    ax.plot(x1,y1,z1, color='r')
    ax.plot(x4,y4,z4, color='b')

    plt.show()
  
"function generates equation of plane based on 3 points. Plane is a list of lists of xyz points. \
Label just is a hacky way to make the print statement nicer"  
def definePlaneEquation(plane, label): 
    p1 = np.array(plane[0])
    p2 = np.array(plane[1])
    p3 = np.array(plane[2])
    # These two vectors are in the plane
    v1 = p3 - p1
    v2 = p2 - p1
    # the cross product is a vector normal to the plane
    cp = np.cross(v1, v2)
    a, b, c = cp
    # This evaluates a * x3 + b * y3 + c * z3 which equals d
    d = np.dot(cp, p3)
    print('The equation of plane {4} is {0}x + {1}y + {2}z = {3}'.format(a, b, c, d, label))

    return a,b,c,d,p1,p2,p3

def rotation_matrix_to_RPY(r):
    # Extract rotation matrix elements
    x = np.arctan2(r[2][1], r[2][2])       
    y = np.arctan2(r[2][1] * -1 , math.sqrt(r[2][1]**2 + r[2][2]**2))     
    z = np.arctan2(r[1][0], r[0][0])

    return x, y, z



"data analysis function"
def getData(V1_func, V2_func):
    theta = np.arccos(np.dot(V1_func, V2_func)/ (np.linalg.norm(V1_func) * np.linalg.norm(V2_func)))

    axis = np.cross(V1_func, V2_func)
    axis1 = axis / np.linalg.norm(axis)
 
    rotVec = theta * axis1
    rot = R.from_rotvec(rotVec)
    rotated_vector_norm  = rot.apply(V2_func)
    rotated_vector_inverse  = rot.apply(V2_func, inverse=True)

    theta_norm = np.cross(rotated_vector_norm,V1_func)  
    theta_norm = abs(np.rint(theta_norm.astype(int)))
    theta_inverse = np.cross(rotated_vector_inverse,V1_func)  
    theta_inverse = abs(np.rint(theta_inverse).astype(int))

    if np.all(theta_norm==0) == True:
        print('theta norm 0')
        print(V1_func)
        print(V2_func)
        print(rotated_vector_norm)
        angles_var = rot.as_euler('xyz', degrees=True)
        # roll, pitch, yaw = rotation_matrix_to_RPY(angles)
        print('PRINTING ANGLES PRINT ANGLES')
        print(angles_var)
        return rotated_vector_norm[0], rotated_vector_norm[1], rotated_vector_norm[2],  angles_var[0], angles_var[1], angles_var[2]
    
    elif np.all(theta_inverse==0) == True:
        print('theta inverse 0')
        print(V1_func)
        print(V2_func)
        print(rotated_vector_inverse)
        angles_var = rot.as_euler('xyz',degrees=True)
        # roll, pitch, yaw = rotation_matrix_to_RPY(angles)
        print('PRINTING ANGLES PRINT ANGLES')
        print(angles_var)
        print(angles_var[0], angles_var[2])

        return rotated_vector_inverse[0], rotated_vector_inverse[1], rotated_vector_inverse[2],angles_var[0], angles_var[1], angles_var[2]
   
def fileSelect():
    root = tk.Tk()
    files = fd.askopenfilenames(parent=root, title='Choose a file')
    file_list = list(files)
    print(file_list)
    return (file_list)

   
density = 1000000

file_list = fileSelect()

# preop = "/Users/georgienahass/Downloads/planned-fibula_test.stl"
# postop = "/Users/georgienahass/Downloads/post-op_STL_for_analysis.stl"

preop = file_list[0]
postop = file_list[1]

# segments = input('select number of segments')
segments = 3
print(f'Number of segments is  {segments}')

out = '/Users/georgienahass/Desktop/'
filename = 'osteoplane.csv'

original_planes = []
rotated_planes = []
deltas = []
angles = []
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
            'phi distal segment 3','theta distal segment 3','psi distal segment 3'] 
        fout.write("%s\n" % ",".join(line))

    for i in range(segments):
        preop_prox_seg_index = []
        preop_prox_seg_index, preop_arr = open3DPick(preop, density)
            
        preOpProxA = preop_arr[preop_prox_seg_index[0]]
        preOpProxB = preop_arr[preop_prox_seg_index[1]]
        preOpProxC = preop_arr[preop_prox_seg_index[2]]
        preOpDistA = preop_arr[preop_prox_seg_index[3]]
        preOpDistB = preop_arr[preop_prox_seg_index[4]]
        preOpDistC = preop_arr[preop_prox_seg_index[5]]

        plane_1_pre = [preOpProxA,preOpProxB,preOpProxC] 
        plane_2_pre = [preOpDistA,preOpDistB,preOpDistC] 

        preOpPoints = [preOpProxA, preOpDistA]

        postop_prox_seg_index, postop_arr = open3DPick(postop, density)
        postOpProxA = postop_arr[postop_prox_seg_index[0]]
        postOpProxB = postop_arr[postop_prox_seg_index[1]]
        postOpProxC = postop_arr[postop_prox_seg_index[2]]
        postOpDistA = postop_arr[postop_prox_seg_index[3]] 
        postOpDistB = postop_arr[postop_prox_seg_index[4]]   
        postOpDistC = postop_arr[postop_prox_seg_index[5]] 
        
        plane_1_post = [postOpProxA,postOpProxB,postOpProxC] 
        plane_2_post = [postOpDistA,postOpDistB,postOpDistC] 

        postOpPoints = [postOpProxA, postOpDistA]

        " put segment 1 (plane 1) on origin and rotate centroid so that all other points \
        this is just doing the movement of the centroid line line up"

        #equation of plane of points around 1st centroid with sengment 1 on origin 
        preProxA1,preProxB1,preProxC1,preProxD1,preProxP1,preProxP2,preProxP3 = definePlaneEquation(plane_1_pre,'one')
        preDistA1,preDistB1,preDistC1,preDistD1,preDistP1,preDistP2,preDistP3 = definePlaneEquation(plane_2_pre,'one')

        postProxA1,postProxB1,postProxC1,postProxD1,postProxP1,postProxP2,postProxP3 = definePlaneEquation(plane_1_post,'one')
        postDistA1,postDistB1,postDistC1,postDistD1,postDistP1,postDistP2,postDistP3 = definePlaneEquation(plane_2_post,'one')

        v1 = np.array([preProxA1,preProxB1,preProxC1])
        v2 = np.array([postProxA1,postProxB1,postProxC1])
        
        a_prox,b_prox,c_prox, phi1, theta1, psi1 = getData(v1, v2)
        
        print(phi1, theta1, psi1)
        
        v3 = np.array([preDistA1,preDistB1,preDistC1])
        v4 = np.array([postDistA1,postDistB1,postDistC1])

        a_dist,b_dist,c_dist, phi2,theta2, psi2 = getData(v3, v4)
        
        #final data preparation for storage  no more data processing
        
        preProxA1 = round(preProxA1/3, 2)
        preProxB1 = round(preProxB1/3, 2)
        preProxC1 = round(preProxC1/3, 2)
        pre_prox_equation = (f'{preProxA1}x + {preProxB1}y + {preProxC1}z = -{preProxD1}')

        preDistA1 = round(preDistA1/3, 2)
        preDistB1 = round(preDistB1/3, 2)
        preDistC1 = round(preDistC1/3, 2)
        pre_dist_equation = (f'{preDistA1}x + {preDistB1}y + {preDistC1}z = -{preDistD1}')
        
        postProxA1 = round(postProxA1/3, 2)
        postProxB1 = round(postProxB1/3, 2)
        postProxC1 = round(postProxC1/3, 2)
        post_prox_equation = (f'{postProxA1}x + {postProxB1}y + {postProxC1}z = -{postProxD1}')

        
        postDistA1 = round(postDistA1/3, 2)
        postDistB1 = round(postDistB1/3, 2)
        postDistC1 = round(postDistC1/3, 2)
        post_dist_equation = (f'{postDistA1}x + {postDistB1}y + {postDistC1}z = -{postDistD1}')
        
        a_prox = round(a_prox/3, 2)
        b_prox = round(b_prox/3, 2)
        c_prox = round(c_prox/3, 2)
        proximal_new_equation = (f'{a_prox}x + {b_prox}y + {c_prox}z')

        a_dist = round(a_dist/3, 2)
        b_dist = round(b_dist/3, 2)
        c_dist = round(c_dist/3, 2)  
        distal_new_equation = (f'{a_dist}x + {b_dist}y + {c_dist}z')
        
        print('LOOK here pre, post, correct, prox dist')
        print('proximal xyz')
        print(preOpProxA)
        print('proximal')
        print(pre_prox_equation)
        print(post_prox_equation)
        print(proximal_new_equation)
        print('distal')
        print('distal xyz')
        print(preOpDistA)
        print(pre_dist_equation)
        print(post_dist_equation)
        print(distal_new_equation)

    
        angles.append(round((phi1), 2))
        angles.append(round((theta1),2 ))
        angles.append(round((psi1),2))
        
        angles.append(round((phi2),2))
        angles.append(round((theta2),2))
        angles.append(round((psi2),2))
        
        original_planes.append(pre_prox_equation)
        original_planes.append(pre_dist_equation)
        original_planes.append(post_prox_equation)
        original_planes.append(post_dist_equation)
        rotated_planes.append(proximal_new_equation)
        rotated_planes.append(distal_new_equation)

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
                angles[15], angles[16],  angles[17]]
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
            'phi distal segment 2','theta distal segment 2','psi distal segment 2']
        fout.write("%s\n" % ",".join(line))

    for i in range(segments):
        preop_prox_seg_index = []
        preop_prox_seg_index, preop_arr = open3DPick(preop, density)
            
        preOpProxA = preop_arr[preop_prox_seg_index[0]]
        preOpProxB = preop_arr[preop_prox_seg_index[1]]
        preOpProxC = preop_arr[preop_prox_seg_index[2]]
        preOpDistA = preop_arr[preop_prox_seg_index[3]]
        preOpDistB = preop_arr[preop_prox_seg_index[4]]
        preOpDistC = preop_arr[preop_prox_seg_index[5]]

        plane_1_pre = [preOpProxA,preOpProxB,preOpProxC] 
        plane_2_pre = [preOpDistA,preOpDistB,preOpDistC] 

        preOpPoints = [preOpProxA, preOpDistA]

        postop_prox_seg_index, postop_arr = open3DPick(postop, density)
        postOpProxA = postop_arr[postop_prox_seg_index[0]]
        postOpProxB = postop_arr[postop_prox_seg_index[1]]
        postOpProxC = postop_arr[postop_prox_seg_index[2]]
        postOpDistA = postop_arr[postop_prox_seg_index[3]] 
        postOpDistB = postop_arr[postop_prox_seg_index[4]]   
        postOpDistC = postop_arr[postop_prox_seg_index[5]] 
        
        plane_1_post = [postOpProxA,postOpProxB,postOpProxC] 
        plane_2_post = [postOpDistA,postOpDistB,postOpDistC] 

        postOpPoints = [postOpProxA, postOpDistA]

        " put segment 1 (plane 1) on origin and rotate centroid so that all other points \
        this is just doing the movement of the centroid line line up"

        #equation of plane of points around 1st centroid with sengment 1 on origin 
        preProxA1,preProxB1,preProxC1,preProxD1,preProxP1,preProxP2,preProxP3 = definePlaneEquation(plane_1_pre,'one')
        preDistA1,preDistB1,preDistC1,preDistD1,preDistP1,preDistP2,preDistP3 = definePlaneEquation(plane_2_pre,'one')

        postProxA1,postProxB1,postProxC1,postProxD1,postProxP1,postProxP2,postProxP3 = definePlaneEquation(plane_1_post,'one')
        postDistA1,postDistB1,postDistC1,postDistD1,postDistP1,postDistP2,postDistP3 = definePlaneEquation(plane_2_post,'one')

        v1 = np.array([preProxA1,preProxB1,preProxC1])
        v2 = np.array([postProxA1,postProxB1,postProxC1])
        
        a_prox,b_prox,c_prox, phi1, theta1, psi1 = getData(v1, v2)
        
        print(phi1, theta1, psi1)
        
        v3 = np.array([preDistA1,preDistB1,preDistC1])
        v4 = np.array([postDistA1,postDistB1,postDistC1])

        a_dist,b_dist,c_dist, phi2,theta2, psi2 = getData(v3, v4)
        
        #final data preparation for storage  no more data processing
        
        preProxA1 = round(preProxA1/3, 2)
        preProxB1 = round(preProxB1/3, 2)
        preProxC1 = round(preProxC1/3, 2)
        pre_prox_equation = (f'{preProxA1}x + {preProxB1}y + {preProxC1}z = -{preProxD1}')

        preDistA1 = round(preDistA1/3, 2)
        preDistB1 = round(preDistB1/3, 2)
        preDistC1 = round(preDistC1/3, 2)
        pre_dist_equation = (f'{preDistA1}x + {preDistB1}y + {preDistC1}z = -{preDistD1}')
        
        postProxA1 = round(postProxA1/3, 2)
        postProxB1 = round(postProxB1/3, 2)
        postProxC1 = round(postProxC1/3, 2)
        post_prox_equation = (f'{postProxA1}x + {postProxB1}y + {postProxC1}z = -{postProxD1}')

        
        postDistA1 = round(postDistA1/3, 2)
        postDistB1 = round(postDistB1/3, 2)
        postDistC1 = round(postDistC1/3, 2)
        post_dist_equation = (f'{postDistA1}x + {postDistB1}y + {postDistC1}z = -{postDistD1}')
        
        a_prox = round(a_prox/3, 2)
        b_prox = round(b_prox/3, 2)
        c_prox = round(c_prox/3, 2)
        proximal_new_equation = (f'{a_prox}x + {b_prox}y + {c_prox}z')

        a_dist = round(a_dist/3, 2)
        b_dist = round(b_dist/3, 2)
        c_dist = round(c_dist/3, 2)  
        distal_new_equation = (f'{a_dist}x + {b_dist}y + {c_dist}z')
        
        print('LOOK here pre, post, correct, prox dist')
        print('proximal xyz')
        print(preOpProxA)
        print('proximal')
        print(pre_prox_equation)
        print(post_prox_equation)
        print(proximal_new_equation)
        print('distal')
        print('distal xyz')
        print(preOpDistA)
        print(pre_dist_equation)
        print(post_dist_equation)
        print(distal_new_equation)

    
        angles.append(round((phi1), 2))
        angles.append(round((theta1),2 ))
        angles.append(round((psi1),2))
        
        angles.append(round(math.degrees(phi2),2))
        angles.append(round(math.degrees(theta2),2))
        angles.append(round(math.degrees(psi2),2))
        
        original_planes.append(pre_prox_equation)
        original_planes.append(pre_dist_equation)
        original_planes.append(post_prox_equation)
        original_planes.append(post_dist_equation)
        rotated_planes.append(proximal_new_equation)
        rotated_planes.append(distal_new_equation)

    with open(out + filename,'a') as fout:
            line = [original_planes[0], original_planes[1], original_planes[2], original_planes[3],  
                original_planes[4], original_planes[5], original_planes[6], original_planes[7], 
                rotated_planes[0],  rotated_planes[1],  
                rotated_planes[2],  rotated_planes[3], 
                angles[0], angles[1],  angles[2],
                angles[3], angles[4],  angles[5],
                angles[6], angles[7],  angles[8],
                angles[9], angles[10],  angles[11]]
            line = [str(l) for l in line]
            fout.write("%s\n" % ",".join(line))


# The new equation of the plane is given by a_new*x + b_new*y + c_new*z + d = 0



