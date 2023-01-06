import pyk4a
import cv2
import numpy as np
import open3d as o3d
import open3d.visualization as vis
import copy
import msvcrt as m
import matplotlib.pyplot as plt
from pyk4a import Config, PyK4A, calibration
#from customized_visualization import custom_draw_geometry_with_key_callback
#from matplotlib import pyplot as plt
#from k4aSession import GetK4aPointCloud, SetView
from register_fragments import  draw_registration_result
from PointCloudPairs import sparse_registration,pairwise_registration
import timeit
import threading
import copy as cp
import sys
#
# scratch3dbs.py - clustering for overlap data in KnnPair 
#
#
#
def  KnnPair(pcS=o3d.geometry.PointCloud(), pcT=o3d.geometry.PointCloud() ,error=24,R=0):
    # KnnPair - Returns the corresponding points between input point clouds, or 0 otherwise
    #           
    #
    # ~0.15 seconds for 25k pnt point clouds ~10x faster than (original) scratch2 version
    #
    #
    #
    # Do clustering over entire pcS cloud
    #
    # cluster_dbscan( eps=160, min_points=100) -> 9 clusters for room3
    #
    #

    labs = np.array(pcS.cluster_dbscan(eps=160, min_points=100, print_progress=False))
    mlab= labs.max()
    colors = plt.get_cmap("tab20")(labs / (mlab if mlab > 0 else 1))
    colors[labs < 0] = 0
    pcS.colors = o3d.utility.Vector3dVector(colors[:, :3])
    #
    # optionally pass in a rotation matrix, for debug
    #
    if np.size(R) != 1: 
        #o3d.visualization.draw_geometries([pcS,pcT], zoom=1/10, front=[0.0, 0.0, 1.0], lookat=[0.0, 1.0, 0.0], up=[0.0, 1, 0])
        pcS.transform(R)
    #
    #
    #    
    # for each point in pcS , find nearest point
    # in pcT and calculate its distance.  Which is 
    # the same as pcS.compute_point_cloud_distance( pcT)
    #
    pcT_tree = o3d.geometry.KDTreeFlann(pcT)


    d0=np.asarray( pcS.compute_point_cloud_distance( pcT) ) 
    # ind has the indexes of the points (we need for sort)
    ind =  np.linspace(0,len(d0)-1,len(d0) ).astype(int) 
    d0=np.column_stack( (d0,ind,labs) )
    #
    # sort d0 by nearest neighbor distance, col 0 
    # keep everything smaller than error
    d0 =np.asarray( sorted( d0,key=lambda x:x[0] ) )
    j=np.searchsorted(d0[:,0],error)
    d0=np.resize(d0,(j,3))

    d0=np.asarray( sorted(d0, key=lambda x:x[2]<0) )

    # filter out anything not in a cluster
    i=1
    for i in range(1,len(d0)-1) :
        if d0[i-1,2] <0: break;
    d0=np.resize(d0,(i-1,3))
    #print(f"pcS {mlab + 1} clusters")

    #o3d.visualization.draw_geometries([pcS,pcT], zoom=1/2, front=[0.0, 0.0, 1.0], lookat=[2.0, -10.0, 0.0], up=[0.0, 1, 0])
    #plt.plot(ind, labs)
    #plt.show()


    for i in range(0, len(d0) ):
        itmp=d0[i][1].astype(int)
        pcS.colors[itmp] = [1,0,0]
    return d0

def GetPair( index = [1,2] ):
    FlipX = np.eye(4)
    FlipZ = np.eye(4)
    FlipY = np.eye(4)
    FlipX[:3,:3] = o3d.geometry.PointCloud().get_rotation_matrix_from_xyz( (np.pi,0, 0) )
    FlipZ[:3,:3] = o3d.geometry.PointCloud().get_rotation_matrix_from_xyz( (0,0, np.pi) )
    FlipY[:3,:3] = o3d.geometry.PointCloud().get_rotation_matrix_from_xyz( (0, np.pi,0) )
    pcl = []
    pcld = []
    pcr=[o3d.geometry.PointCloud(),o3d.geometry.PointCloud(),o3d.geometry.PointCloud()]

    dataDir='C://repo//apricus//phaser_test_data//test_clouds//os0//'
    #dataDir='C://repo//bart//demo//room3//' 
    #dataDir='C://repo//bart//demo//room4//'
    for i in  index:
        pcs=o3d.io.read_point_cloud(dataDir + 'source_' + str(i).zfill(1)+'.ply',format='ply')
        pcs.estimate_normals(o3d.geometry.KDTreeSearchParamKNN( knn=10))
        pct=o3d.io.read_point_cloud(dataDir + 'target_' + str(i).zfill(1)+'.ply',format='ply')
        pct.estimate_normals(o3d.geometry.KDTreeSearchParamKNN( knn=10))
        #pcd.transform(FlipZ)
        #pcd.transform(FlipY)
        pcld.append(pcs)
        pcld.append(pct)
    return pcld
def regpair(source=o3d.geometry.PointCloud(),target=o3d.geometry.PointCloud(),init=np.identity(4), coarse_max=60, fine_max=30, max_iteration=3,RegType=o3d.pipelines.registration.TransformationEstimationPointToPlane()):
    cc=o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-18,
                                                          relative_rmse=1e-18,
                                                          max_iteration=max_iteration)
    #icp_fine = o3d.pipelines.registration.registration_phaser( source, target, coarse_max, init, RegType)  
    #icp_fine = o3d.pipelines.registration.registration_phaser( source, target,  RegType)
    a=o3d.pipelines.registration.TransformationEstimationPhaser()
    #tp= o3d.pipelines.registration.TapPoint()
    tp=1
    icp_fine = o3d.pipelines.registration.registration_phaser( source, target,tp)
    #aaa=o3d.pipelines.registration.evaluate_registration(source,target,fine_max,icp_fine.transformation)
    corr=np.asarray(aaa.correspondence_set);
    #print("icp fine: " ,icp_fine)
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds( source, target, fine_max,icp_fine.transformation )
    #draw_registration_result(source, target, icp_fine.transformation)
    #return transformation_icp, information_icp
    return icp_fine,corr

def foo( in0= int() ):
    a= in0
    if (in0==1):
        a=[2.0 ,3.0]
    elif(in0==2):
        a=True;
    else:
        a = "boop";
    return a
def main():
    aaa=foo(1)
    tp= o3d.cpu.pybind.pipelines.registration.TransformationEstimationPhaser()
    a=o3d.pipelines.registration.TransformationEstimationPhaser()
    RyCw75 = np.eye(4)

    RyCw75[:3,:3] = o3d.geometry.PointCloud().get_rotation_matrix_from_xyz( (0,75.0*np.pi/180.0, 0) )

    #index = [22,8]
    index = [4,4]

    #regType=o3d.pipelines.registration.TransformationEstimationPointToPlane()
    regType=o3d.pipelines.registration.TransformationEstimationPhaser()
    pcld = GetPair(index)
    [icpT,corr]=regpair(pcld[0], pcld[1],init=np.eye(4),coarse_max=60.0,fine_max=30.0,max_iteration=1,RegType=regType)
    pcld[0].paint_uniform_color([1, 0.706, 0])
    pcld[1].paint_uniform_color([0.2, 0.2, 0.2])

    #result = timeit.timeit('KnnPair(pcld[0], pcld[1],13.1)', globals=globals(), number=10)/10
    #print(result )

    #d=KnnPair(pcld[0], pcld[1],13.1)
    pcld[0] = pcld[0].transform(icpT.transformation)
  
    o3d.visualization.draw_geometries(pcld[:2], zoom=1/5, front=[0.0, 0.0, -1.0], lookat=[0.0, 1.0, 0.0], up=[0.0, -1, 0])
    #o3d.visualization.draw_geometries(pcr, zoom=1/10, front=[0.0, 0.0, 1.0], lookat=[0.0, 1.0, 0.0], up=[0.0, 1, 0])

    exit()
if __name__ == "__main__":
    main()