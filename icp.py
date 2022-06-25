
import open3d as o3d
import numpy as np
import copy
from utils import *
    

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def point_to_point_icp(source, target, threshold, trans_init):
    print("Apply point-to-point ICP")
    reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(with_scaling=False),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = 2000))
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    draw_registration_result(source, target, reg_p2p.transformation)


def point_to_plane_icp(source, target, threshold, trans_init):
    print("Apply point-to-plane ICP")
    target.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=2000))
    target.orient_normals_consistent_tangent_plane(k=50)
    source.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=2000))
    source.orient_normals_consistent_tangent_plane(k=50)
    sigma = 0.1
    loss = o3d.pipelines.registration.TukeyLoss(k=sigma)
    reg_p2l = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(loss))
    print(reg_p2l)
    print("Transformation is:")
    print(reg_p2l.transformation, "\n")

    result = o3d.pipelines.registration.evaluate_registration(source, target, 0.05, reg_p2l.transformation)
    print(result)
    draw_registration_result(source, target, reg_p2l.transformation) 


if __name__ == "__main__":
    source = o3d.io.read_point_cloud('ExtractedPlanes/Cornera.pcd')
    target = o3d.io.read_point_cloud('ExtractedPlanes/Cornerb.pcd')
    
    # Input paramters for initial guess (Calculated by rough calibration script)

    ##  Rotation (Euler angles (radians))
    
    # Scene 1
    # yaw, pitch, roll = [ 0.00191094, -0.00133544, -0.00203349]
    # Scene 2
    # yaw, pitch, roll = [ 0.00192064, -0.00165003, -0.00114435]
    # Scene 3
    # yaw, pitch, roll = [-0.00045205, -0.00159071 ,-0.00208067]
    # Scene 4
    yaw, pitch, roll = [ 0.00096031, -0.00165986, -0.00191435]
    # Scene 5
    # yaw, pitch, roll = [ 0.00125083, -0.00130257, -0.00187364]
    
    # Translation (m)

    # Scene1
    # tx = 1.6805
    # ty = -2.355
    # tz = 1.1707

    #  # Scene2
    # tx = 1.7362
    # ty = -2.3764
    # tz = 1.1711

    #  # Scene3
    tx = 1.6582
    ty = -2.4083
    tz = 1.168

    # # Scene4
    # tx = 1.6783
    # ty = -2.4337
    # tz = 1.1682

    # # Scene5
    # tx = 1.6805
    # ty = -2.3429
    # tz = 1.169


    # Can be replaced by  Scipy module R (But I did it manually in the begining)


    r11 = math.cos(pitch)*math.cos(yaw)
    r12 = math.cos(pitch)*math.sin(yaw)
    r13 = -1 * math.sin(pitch)

    r21 = -1 * math.cos(roll)*math.sin(yaw) - math.sin(roll)*math.sin(pitch)*math.cos(yaw)
    r22 = math.cos(roll)*math.cos(yaw) - math.sin(roll)*math.sin(pitch)*math.sin(yaw)
    r23 = -1 * math.sin(roll)*math.cos(pitch)

    r31 = -1 * math.sin(roll)*math.sin(yaw) + math.cos(roll)*math.sin(pitch)*math.cos(yaw)
    r32 = math.sin(roll)*math.cos(yaw) + math.cos(roll)*math.sin(pitch)*math.sin(yaw)
    r33 = math.cos(roll)*math.cos(pitch)

    threshold = 0.05

    trans_init = np.asarray([[r11, r12, r13, tx],
                             [r21, r22, r23, ty],
                             [r31, r32, r33, tz],[0.0, 0.0, 0.0, 1.0]])
    print(trans_init)

    draw_registration_result(source, target, trans_init)

    print("Initial alignment")
    evaluation = o3d.pipelines.registration.evaluate_registration(
        source, target, threshold, trans_init)
    print(evaluation, "\n")

    point_to_point_icp(source, target, threshold, trans_init)
    # point_to_plane_icp(source, target, threshold, trans_init)