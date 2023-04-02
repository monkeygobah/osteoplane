import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import math
# from mpl_toolkits.mplot3d import Axes3D
from quaternion import as_euler_angles
from scipy.spatial.transform import Rotation as R
from plyfile import PlyData, PlyElement

"calculate centroid of 3 points "
def getCentroid(point_1, point_2, point_3):
    center_x = np.average([point_1[0], point_2[0], point_3[0]])
    center_y = np.average([point_1[1], point_2[1], point_3[1]])
    center_z = np.average([point_1[2], point_2[2], point_3[2]])
    data = [center_x, center_y, center_z]
    return data

"Move whole structure to origin. if else statements allow us to decide which segment we are placing"
def moveToOrigin(points, label):
    x = label[0]
    y = label[1]
    z = label[2]
    if len(points) == 4:
        centroid1 = [points[0][0] - x, points[0][1] -y, points[0][2] -z]
        centroid2 = [points[1][0] - x, points[1][1] -y, points[1][2] -z]
        centroid3 = [points[2][0] - x, points[2][1] -y, points[2][2] -z]
        centroid4 = [points[3][0] - x, points[3][1] -y, points[3][2] -z]
        out = [centroid1, centroid2, centroid3, centroid4]
    elif len(points) == 3:
        centroid1 = [points[0][0] - x, points[0][1] -y, points[0][2] -z]
        centroid2 = [points[1][0] - x, points[1][1] -y, points[1][2] -z]
        centroid3 = [points[2][0] - x, points[2][1] -y, points[2][2] -z]
        out = [centroid1, centroid2, centroid3]
    elif len(points) == 2:
        centroid1 = [points[0][0] - x, points[0][1] -y, points[0][2] -z]
        centroid2 = [points[1][0] - x, points[1][1] -y, points[1][2] -z]
        out = [centroid1, centroid2]
    else:
        print('Improper number of points input')
    return out

"calculate angle to move points to z axis. Could probably combine with rotate_struct_1"
def getAngle_X(centroid2):
    # get angle to rotate whole skull from the naseon now
    angle = math.atan(centroid2[2]/centroid2[1]) #is in radians
    angle = (-1*angle)
    return angle

"rotate any given points to x axis. x,y,z are lists of points, angle is the angle to rotate by "
def rotate_struct_1(points, angle):
    for value in points:
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
    for value in points:
        x = value[0] * math.cos(angle) - value[1] * math.sin(angle) 
        y = value[1] * math.cos(angle) + value[0] * math.sin(angle)
        value[0] = x
        value[1] = y
    return points

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

"find angle between 2 input planes. Need to understand which angles we really \
care about and which ones come first "
def calcPlaneAngle(a1, b1, c1, a2, b2, c2):
    d = ( a1 * a2 + b1 * b2 + c1 * c2 )
    e1 = math.sqrt( a1 * a1 + b1 * b1 + c1 * c1)
    e2 = math.sqrt( a2 * a2 + b2 * b2 + c2 * c2)
    d = d / (e1 * e2)
    A = math.degrees(math.acos(d))
    print("Angle is " + str(A) + " degree")
    return A

density = 1000000
preop = "/Users/georgienahass/Downloads/planned-fibula_test.stl"
postop = "/Users/georgienahass/Downloads/post-op_STL_for_analysis.stl"

# segments = input('select number of segments')
segments = 3
print(f'Number of segments is  {segments}')


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

    index_one = 0
    index_two = 1

    print('Beginning analysis of first line segment...')
    ### go to origin
    print('Moving to origin')
    anteriorLinePreOp = moveToOrigin([preOpProxA, preOpDistA], preOpPoints[index_one])
    angle1_preop = getAngle_X(anteriorLinePreOp[index_two])

    anteriorLinePostOp = moveToOrigin([postOpProxA, postOpDistA], postOpPoints[index_one])
    angle1_postop = getAngle_X(anteriorLinePostOp[index_two])

    ### lay flat
    print('Laying flat')
    rotate_1_preop = rotate_struct_1(anteriorLinePreOp, angle1_preop)
    rotate_1_postop = rotate_struct_1(anteriorLinePostOp, angle1_postop)

    ### align on axis
    angle2_preop = getAngle_Z(rotate_1_preop[index_two])
    angle2_postop = getAngle_Z(rotate_1_postop[index_two])

    print('Fixing on other axis ')
    rotate_2_preop = rotate_struct_2(rotate_1_preop, angle2_preop)
    rotate_2_postop = rotate_struct_2(rotate_1_postop, angle2_postop)

    "use the originally selected points to define a plane. We will use the same math as for the centroid \
    alignment, but these are the numbers which we will use to get the solution. Only for segment 1, however the same code \
        essentially is used for segment 2 and 3 as well. Only real change is input points"

    proxPlanePreop_1 = moveToOrigin([plane_1_pre[0], plane_1_pre[1], plane_1_pre[2]], preOpPoints[index_one])
    distPlanePreop_1 = moveToOrigin([plane_2_pre[0], plane_2_pre[1], plane_2_pre[2]], preOpPoints[index_one])

    proxPlanePostop_1 = moveToOrigin([plane_1_post[0], plane_1_post[1], plane_1_post[2]], postOpPoints[index_one])
    distPlanePostop_1 = moveToOrigin([plane_2_post[0], plane_2_post[1], plane_2_post[2]], postOpPoints[index_one])

    proxPlanePreop_2 = rotate_struct_1(proxPlanePreop_1, angle1_preop)
    distPlanePreop_2 = rotate_struct_1(distPlanePreop_1, angle1_preop)

    proxPlanePostop_2 = rotate_struct_1(proxPlanePostop_1, angle1_postop)
    distPlanePostop_2 = rotate_struct_1(distPlanePostop_1, angle1_postop)

    proxPlanePreop_3 = rotate_struct_2(proxPlanePreop_2, angle2_preop)
    distPlanePreop_3 = rotate_struct_2(distPlanePreop_2, angle2_preop)

    proxPlanePostop_3 = rotate_struct_2(proxPlanePostop_2, angle2_postop)
    distPlanePostop_3 = rotate_struct_2(distPlanePostop_2, angle2_postop)

    #equation of plane of points around 1st centroid with sengment 1 on origin 
    preProxA1,preProxB1,preProxC1,preProxD1,preProxP1,preProxP2,preProxP3 = definePlaneEquation(proxPlanePreop_3,'one')
    preDistA1,preDistB1,preDistC1,preDistD1,preDistP1,preDistP2,preDistP3 = definePlaneEquation(distPlanePreop_3,'one')

    postProxA1,postProxB1,postProxC1,postProxD1,postProxP1,postProxP2,postProxP3 = definePlaneEquation(proxPlanePostop_3,'one')
    postDistA1,postDistB1,postDistC1,postDistD1,postDistP1,postDistP2,postDistP3 = definePlaneEquation(distPlanePostop_3,'one')

    ######## normal plane should be xz plane- used three random points from xz plane. Need d value 
    # (distance from xz plane.) In this case, d = 1/2 the y component of the segment not on the origin

    y_pre = rotate_2_preop[1][1] /2
    y_post = rotate_2_postop[1][1] /2

    "need to fix this plane plotting thing so it makes sense  Currently just showing it in browser"
    # plotPlane(a1,b1,c1,d1,p1,p2,p3)
    # plotPlane(a2,b2,c2,d2,p12,p22,p23 )
    # plotPlane(a3,b3,c3,d3,p4,p5,p6)

    ######## subtract normal plane - plane 1 and normal plane - plane 2 to get angles of interest
    normMinusPlaneProxPre = calcPlaneAngle(preProxA1,preProxB1,preProxC1,0,y_pre,0)
    normMinusPlaneDistPre = calcPlaneAngle(preDistA1,preProxB1,preDistC1,0,y_pre,0)

    normMinusPlaneProxPost = calcPlaneAngle(postProxA1,postProxB1,postProxC1,0,y_post,0)
    normMinusPlaneDistPost = calcPlaneAngle(postDistA1,postProxB1,postDistC1,0,y_post,0)

    deltaProx = normMinusPlaneProxPost-normMinusPlaneProxPre
    deltaDist = normMinusPlaneDistPost-normMinusPlaneDistPre
    

# The new equation of the plane is given by a_new*x + b_new*y + c_new*z + d = 0



