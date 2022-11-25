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
   

def pairwise_registration(source=o3d.geometry.PointCloud(),target=o3d.geometry.PointCloud(),init=np.identity(4), coarse_max=60, fine_max=30, max_iteration=3):
    #print("Apply point-to-plane ICP")
    #icp_coarse = o3d.pipelines.registration.registration_icp( source, target, coarse_max, init, o3d.pipelines.registration.TransformationEstimationPointToPlane() )
    #icp_fine = o3d.pipelines.registration.registration_icp( source, target, fine_max,icp_coarse.transformation,o3d.pipelines.registration.TransformationEstimationPointToPlane() )
    cc=o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-18,
                                                          relative_rmse=1e-18,
                                                          max_iteration=max_iteration)
    icp_coarse = o3d.pipelines.registration.registration_icp( source, target, coarse_max, init, o3d.pipelines.registration.TransformationEstimationPointToPlane(),cc )
    icp_fine = o3d.pipelines.registration.registration_icp( source, target, fine_max,icp_coarse.transformation,o3d.pipelines.registration.TransformationEstimationPointToPlane(),cc )
    
    aaa=o3d.pipelines.registration.evaluate_registration(source,target,fine_max,icp_fine.transformation)
    corr=np.asarray(aaa.correspondence_set);
    #print("icp fine: " ,icp_fine)
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds( source, target, fine_max,icp_fine.transformation )
    #draw_registration_result(source, target, icp_fine.transformation)
    #return transformation_icp, information_icp
    return icp_fine,corr
def sparse_registration(pcds,init,coarse_max, fine_max, max_iteration):
    icptList=[]
    corrList=[]
    for sid in range(0,len(pcds)-1):
        dist=np.asarray( pcds[sid].compute_point_cloud_distance(pcds[sid+1]) )
        [icpT,corr] = pairwise_registration( pcds[sid], pcds[sid+1],init,coarse_max,fine_max,max_iteration )
        #print("mean,std,max dist between clouds:", np.mean(dist),np.sqrt(np.var(dist)), np.max(dist) ) 
        icptList.append(icpT)
        corrList.append(corr)
    return icptList,corrList

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

    dataDir='C://repo//bart//demo//room3//' 
    #dataDir='C://repo//bart//demo//room4//'
    for i in  index:
        pc=o3d.io.read_point_cloud(dataDir + 'pc_' + str(i).zfill(4)+'.pcd')
        pcd=o3d.io.read_point_cloud(dataDir + 'pcd_' + str(i).zfill(4)+'.pcd')
        pcd.transform(FlipZ)
        pcd.transform(FlipY)
        pcl.append(pc)
        pcld.append(pcd)
    return pcld
def GetTranslation( tr):
    T=RyCw75 = np.eye(4)
    T[0][3] = tr[0]
    T[1][3] =tr[1]
    T[2][3]=tr[2]
    return T

def  GetTheta( rangeTheta,pcld):
    coursemax=60.0
    coursemin= 30.0
    _Rtheta = np.eye(4)
    fitList = []
    #if 1:
  
    for theta in rangeTheta:
        _Rtheta[:3,:3]=o3d.geometry.PointCloud().get_rotation_matrix_from_xyz( (0,theta, 0) )
        actualTheta=np.arcsin(_Rtheta[2][0])*180/np.pi
        #print( 'actual theta:',actualTheta, 'theta:',theta*180/np.pi)
        tmpPc= cp.deepcopy(pcld[0] )
      
        #[icptList,corrList] = sparse_registration(pcld,Rtheta,coarse_max=coursemax,fine_max=coursemin,max_iteration=3)
        corrList = [1,2]
        d0=KnnPair(tmpPc, pcld[1],13.1,_Rtheta)
        #print ('theta',theta*180.0/np.pi,'len d0:',len(d0))
        #if ( len(d0) ): continue
        
        fitList.append( [len(d0),theta,corrList,cp.deepcopy(_Rtheta)] )
    maxInd = max(range(len(fitList)), key=fitList.__getitem__)
    bestTheta=fitList[maxInd]

    # max_iteration works typically at 3.  However for significan rotation about z (in addition to y), need 40
    #
    [icptList,corrList] = sparse_registration(pcld,bestTheta[3],coarse_max=coursemax,fine_max=coursemin,max_iteration=40)


    actualTheta =np.arcsin(bestTheta[3][2][0])

    bestTheta=[icptList[0].fitness,actualTheta,corrList,icptList[0].transformation]
    return bestTheta
def  GetZ( rangeZ,pcld): 
    coursemax=300.0
    coursemin= 50.0
    fitList = []
    for z in rangeZ:
        Translate=GetTranslation( [0,0, z] )
        [icptList,corrList] = sparse_registration(pcld,Translate,coarse_max=coursemax,fine_max=coursemin,max_iteration=3)
        fitList.append( [icptList[0].fitness,z,corrList,icptList[0].transformation] )
    maxInd = max(range(len(fitList)), key=fitList.__getitem__)
    bestTheta=fitList[maxInd] 
    
    [icptList,corrList] = sparse_registration(pcld,bestTheta[3],coarse_max=coursemax,fine_max=coursemin,max_iteration=20)

    actualZ =bestTheta[3][2][3]
    bestZ=[icptList[0].fitness,actualZ,corrList,icptList[0].transformation]   
    return bestZ

    
def main():
    RyCw75 = np.eye(4)

    #Translate = np.eye(4)
    RyCw75[:3,:3] = o3d.geometry.PointCloud().get_rotation_matrix_from_xyz( (0,75.0*np.pi/180.0, 0) )
    #index = [8,22]
    #index = [22,8]
    index = [8,6]
    #index=[42,35]
    #index=[39,1]
    #index=[0,3]
    pcld = GetPair(index)

    theta=np.linspace(-np.pi*100/180,np.pi*100/180,20)

    bestTheta = GetTheta(theta, pcld)
    pcld[0].transform(bestTheta[3])

    #bestZ=GetZ( np.linspace(0,1200,100),pcld)
    #pcld[0].transform(bestZ[3])
    bestZ=bestTheta

    #result = timeit.timeit('KnnPair(pcld[0], pcld[1],13.1)', globals=globals(), number=10)/10
    #print(result )

    d=KnnPair(pcld[0], pcld[1],13.1)

    print("fitness: ",bestTheta[0],"theta:", bestTheta[1]*180/np.pi,"num corresp: " ,len(bestTheta[2][0]) )




    print("fitness: ",bestZ[0],"z:", bestZ[1],"num corresp: " ,len(bestZ[2][0]), "actual theta:", bestTheta[1]*180/np.pi     )
    o3d.visualization.draw_geometries(pcld, zoom=1/10, front=[0.0, 0.0, 1.0], lookat=[0.0, 1.0, 0.0], up=[0.0, 1, 0])
    #o3d.visualization.draw_geometries(pcr, zoom=1/10, front=[0.0, 0.0, 1.0], lookat=[0.0, 1.0, 0.0], up=[0.0, 1, 0])

    exit()
if __name__ == "__main__":
    main()