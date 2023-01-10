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
from TestAprikus1 import KnnPair
import timeit
import threading
import copy as cp
import sys
#
# clustering for overlap data in KnnPair 
#
#
#
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

    #dataDir='C://repo//apricus//phaser_test_data//test_clouds//os0//'
    dataDir='C://repo//bart//demo//room3//' 
    #dataDir='C://repo//bart//demo//room4//'
    #for i in  index:
    pcs=o3d.io.read_point_cloud(dataDir + 'pcd_' + str(index[0]).zfill(4)+'.pcd')
    pcs.estimate_normals(o3d.geometry.KDTreeSearchParamKNN( knn=10))
    pct=o3d.io.read_point_cloud(dataDir + 'pcd_' + str(index[1]).zfill(4)+'.pcd')
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

    a=o3d.pipelines.registration.TransformationEstimationPhaser()
    #tp= o3d.pipelines.registration.TapPoint()
    tp=1
    icp_fine = o3d.pipelines.registration.registration_phaser( source, target,tp)
    a=icp_fine.getRegisteredCloud()
    T=icp_fine.getTransform()
    rota = icp_fine.getRotation()
    trana = icp_fine.getTranslation()
    rotc = icp_fine.getRotationCorrelation()
    o3d.visualization.draw_geometries([a], zoom=1/5, front=[0.0, 0.0, -1.0], lookat=[0.0, 1.0, 0.0], up=[0.0, -1, 0])
    #aaa=o3d.pipelines.registration.evaluate_registration(source,target,fine_max,icp_fine.transformation)
    corr=np.asarray(aaa.correspondence_set);
    #print("icp fine: " ,icp_fine)
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds( source, target, fine_max,icp_fine.transformation )
    #draw_registration_result(source, target, icp_fine.transformation)
    #return transformation_icp, information_icp
    return icp_fine,corr


def main():
    #x, y, u, v = np.random.random((4,10))
    #plt.quiver(x, y, u, v)
   

    tp= o3d.cpu.pybind.pipelines.registration.TransformationEstimationPhaser()
    a=o3d.pipelines.registration.TransformationEstimationPhaser()
    RyCw75 = np.eye(4)

    RyCw75[:3,:3] = o3d.geometry.PointCloud().get_rotation_matrix_from_xyz( (0,75.0*np.pi/180.0, 0) )

    index = [22,17]
    #index = [3,3]

    #regType=o3d.pipelines.registration.TransformationEstimationPointToPlane()
    regType=o3d.pipelines.registration.TransformationEstimationPhaser()
    pcld = GetPair(index)
    pcld[0].paint_uniform_color([1, 0.706, 0])
    pcld[1].paint_uniform_color([0.4, 0.3, 0.2])
    pglob = o3d.pipelines.registration.registration_phaser( pcld[0], pcld[1],1)
    a=pglob.getRegisteredCloud()
    T=pglob.getTransform()
    rota = pglob.getRotation()
    trana = pglob.getTranslation()
    rotc = pglob.getRotationCorrelation()
    rotsig= pglob.getRotUncertaintyEstimate()
    o3d.visualization.draw_geometries(pcld, zoom=1/5, front=[0.0, 0.0, -1.0], lookat=[0.0, 1.0, 0.0], up=[0.0, -1, 0])



  
    pcld[0] = pcld[0].transform(T)
  
    
    #plt.plot( rotc[1500000:4000000])
    plt.plot( rotc)
    plt.show()
    o3d.visualization.draw_geometries(pcld[:2], zoom=1/5, front=[0.0, 0.0, -1.0], lookat=[0.0, 1.0, 0.0], up=[0.0, -1, 0])
    #o3d.visualization.draw_geometries(pcr, zoom=1/10, front=[0.0, 0.0, 1.0], lookat=[0.0, 1.0, 0.0], up=[0.0, 1, 0])

    exit()
if __name__ == "__main__":
    main()