import numpy as np
import open3d as o3d
import math
from utils import *
from scipy.spatial.transform import Rotation
import random
import matplotlib.pyplot as plt
import copy
   
def DetectOrthoPlane(points, min_ratio=0.05, threshold=0.01, iterations=1000):

    plane_list = []
    temp_list = []
    N = len(points)

    target = points.copy()
    count = 0

    counter = 0

    while count < (1 - min_ratio) * N:
        w, index = PlaneRegression(
            target, threshold=threshold, init_n=3, iter=iterations)
        
        count += len(index)
        print("Counter Ortho ", counter)
        
        if counter==0:
            first_plane = w
            unit_vector_first_plane = first_plane[:3] / np.linalg.norm(first_plane[:3])
            plane_list.append([w, target[index]])
            # temp_list.append(np.array([w, target[index]],dtype='object'))
            # target = np.delete(target, index, axis=0)\
            print("UNIT VECTOR 1st plane : ", unit_vector_first_plane)
            
        
        else:
            unit_vector_new = w[:3] / np.linalg.norm(w[:3])
            print("UNIT VECTOR planes : ", unit_vector_new)
            dot_prod = np.dot(unit_vector_first_plane, unit_vector_new)
            print("DEBUG DOT Products ORTHO ", dot_prod)
            if dot_prod > 0.8:
                pass
            else:
                plane_list.append([w, target[index]])
                break
            # temp_list.append(np.array([w, target[index]],dtype='object'))
            # target = np.delete(target, index, axis=0)

        counter+=1 
        target = np.delete(target, index, axis=0)
        
        # print(counter)
        

    print(f"Numnber of points {count}")
    # print(np.array(temp_list[0][0]))

    return plane_list


def PreprocessCloud(cloud, gp_removal):
    pcd = range_filter(cloud, r_min = 3.5, r_max= 20)

    # pcd.colors = o3d.utility.Vector3dVector([1, 0, 0])
    # voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,
    #                                                         voxel_size=0.3)
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.3)
    cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20,
                                                    std_ratio=2.5)
    # cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=15, radius=1.)

    # pcd.estimate_normals(
    # search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    # pcd.orient_normals_consistent_tangent_plane(k=20)

    inlier_cloud = voxel_down_pcd.select_by_index(ind)
    outlier_cloud = voxel_down_pcd.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.3, 0.4, 0.2])
    inlier_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.8, max_nn=20))
    inlier_cloud.orient_normals_consistent_tangent_plane(k=20)
    # pcd.paint_uniform_color([0.3, 0.4, 0.2])

    plane_model, inliers = inlier_cloud.segment_plane(distance_threshold=0.09,
                                         ransac_n=3,
                                         num_iterations=2000)
    if not gp_removal:
        inlier_cloud = inlier_cloud.select_by_index(inliers)
        return inlier_cloud

    else:
        inlier_cloud = inlier_cloud.select_by_index(inliers, invert = True)
        new_points = np.asarray(inlier_cloud.points)
        other_planes = DetectMultiPlanes(new_points, min_ratio=0.1, threshold=0.01, iterations=2000)
        
        planes_eqs = []
        planes = []
        colors = []
        for _ , plane in other_planes:

            planes_eqs.append(_)

            r = random.random()
            g = random.random()
            b = random.random()

            color = np.zeros((plane.shape[0], plane.shape[1]))
            color[:, 0] = r
            color[:, 1] = g
            color[:, 2] = b

            planes.append(plane)
            colors.append(color)
            
        print(len(planes))
        
        planes = np.concatenate(planes, axis=0)
        colors = np.concatenate(colors, axis=0)

        inlier_cloud = getColoredPlanes(planes, colors)
        return inlier_cloud


def GPAlignment(plane1, plane2):
    plane1_points = np.asarray(plane1.points)
    plane2_points = np.asarray(plane2.points)
    model1, index1 = PlaneRegression(plane1_points, threshold=0.09, init_n=3, iter=2000)
    model2, index2 = PlaneRegression(plane2_points, threshold=0.09, init_n=3, iter=2000)

    unit_vector_1 = model1[:3] / np.linalg.norm(model1[:3])
    unit_vector_2 = model2[:3]/ np.linalg.norm(model2[:3])


    print("Model1 ", model1)
    print("Model2 ", model2)

    dot_product = np.dot(unit_vector_1, unit_vector_2)
    cross_product = np.cross(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)
    
    K = np.array([[0, -cross_product[0], cross_product[1]],
    [cross_product[2], 0, -cross_product[2]],
    [-cross_product[1], cross_product[0], 0]])

    R = np.identity(3) + math.sin(angle) * K + (1 - math.cos(angle)) * np.matmul(K,K)
    print(R)

    quat = [math.sin(angle/2)*cross_product[0], math.sin(angle/2)*cross_product[1], math.sin(angle/2)*cross_product[2], math.cos(angle/2)]

    angle_axis = [cross_product[0], cross_product[1], cross_product[2], angle]
    r = Rotation.from_quat(quat)
    print("Euler angle zyx")
    print(r.as_euler('zyx', degrees=False))

    print("Angle Axis Form : ", angle_axis)

    rotated_plane2 = plane2.rotate(R, center=(0, 0, 0))

    # rotated_points = np.matmul(plane2_points, R)

    rotated_points = np.asarray(rotated_plane2.points)

    model3, index3 = PlaneRegression(rotated_points, threshold=0.09, init_n=3, iter=2000)

    


    unit_vector_3 = model3[:3]/ np.linalg.norm(model2[:3])

    print("Model3 ", model3)

    z_offset = (model1[-1]-model3[-1])/math.sqrt(model1[0]**2+model1[1]**2+model1[2]**2)

    print("Guess for Z-Axis translation ", z_offset)

    aligned_plane2 = rotated_plane2.translate((0, 0, -z_offset))

    # aligned_points = rotated_points[:,-1] - z_offset
    # aligned_points =aligned_points.reshape(-1,1)



    return aligned_plane2 , R , z_offset

def getBestCluster(cloud):
    with o3d.utility.VerbosityContextManager(
    o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
        cloud.cluster_dbscan(eps=2, min_points=3, print_progress=True)
        )

    print("LABELS")
    print(np.unique(labels))
    candidates=[len(np.where(labels==j)[0]) for j in np.unique(labels)]

    print("Candidates")
    print(np.unique(candidates))

    best_candidate=int(np.unique(labels)[np.where(candidates== np.max(candidates))[0]])

    print("BEST Candidate")
    print(np.unique(best_candidate))
    
    max_label = labels.max()

    # o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)])

    
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

    cloud = cloud.select_by_index(list(np.where(labels== best_candidate)[0]))

    return cloud

def getCornerPlanes(cloud):
    points = np.asarray(cloud.points)

    other_planes = DetectOrthoPlane(points, min_ratio=0.05, threshold=0.1, iterations=2000)
        
    planes_eqs = []
    planes = []
    colors = []
    for _ , plane in other_planes:

        planes_eqs.append(_)

        r = random.random()
        g = random.random()
        b = random.random()

        color = np.zeros((plane.shape[0], plane.shape[1]))
        color[:, 0] = r
        color[:, 1] = g
        color[:, 2] = b

        planes.append(plane)
        colors.append(color)
        
    print(len(planes))
    
    planes = np.concatenate(planes, axis=0)
    colors = np.concatenate(colors, axis=0)

    inlier_cloud = getColoredPlanes(planes, colors)
    return inlier_cloud, planes, planes_eqs


def getYaw(corner1, models1, corner2, models2, z_offset):
    corner1_points = np.asarray(corner1.points)
    corner2_points = np.asarray(corner2.points)
    model1, index1 = PlaneRegression(corner1_points, threshold=0.09, init_n=3, iter=2000)
    model2, index2 = PlaneRegression(corner2_points, threshold=0.09, init_n=3, iter=2000)


    plane1b = corner1.select_by_index(index1, invert=True)
    plane1b_points = np.asarray(plane1b.points)

    model1b, index1b = PlaneRegression(plane1b_points, threshold=0.09, init_n=3, iter=200)
    unit_vector_1 = model1[:3] / np.linalg.norm(model1[:3])
    unit_vector_1b = model1b[:3] / np.linalg.norm(model1b[:3])
   
    if np.dot(np.cross(unit_vector_1, unit_vector_1b), np.array([0,0,1])) < 0:
        model1, index1 = modelb, index1b


    plane2b = corner1.select_by_index(index2, invert=True)
    plane2b_points = np.asarray(plane2b.points)

    model2b, index2b = PlaneRegression(plane2b_points, threshold=0.09, init_n=3, iter=200)
    unit_vector_2 = model2[:3]/ np.linalg.norm(model2[:3])
    unit_vector_2b = model2b[:3] / np.linalg.norm(model2b[:3])


    if np.dot(np.cross(unit_vector_2, unit_vector_2b), np.array([0,0,1])) < 0:
        model2, index2 = model2b, index2b

    

    dot_product = np.dot(unit_vector_1, unit_vector_2)
    cross_product = np.cross(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)

    # print(cross_product)
    
    K = np.array([[0, -cross_product[0], cross_product[1]],
    [cross_product[2], 0, -cross_product[2]],
    [-cross_product[1], cross_product[0], 0]])

    R = np.identity(3) + math.sin(angle) * K + (1 - math.cos(angle)) * np.matmul(K,K)
    # R = np.matmul(rotation1, R)
    print(R)

    # quat = [math.sin(angle/2)*cross_product[0], math.sin(angle/2)*cross_product[1], math.sin(angle/2)*cross_product[2], math.cos(angle/2)]
    # angle_axis = [cross_product[0], cross_product[1], cross_product[2], angle]
    
    r = Rotation.from_matrix(R)
    quat = r.as_quat()
    print("Euler angle zyx")
    print(r.as_euler('zyx', degrees=False))

    angle_temp = np.arccos(quat[-1])*2
    print("calculated angle", angle_temp)

    angle_axis = [quat[-2]/math.sin(angle_temp/2), quat[-3]/math.sin(angle_temp/2), quat[-4]/math.sin(angle_temp/2), angle_temp]

    print("Angle Axis Form : ", angle_axis)

    print("test_offset x :", (model1[-1]-model2[-1])/math.sqrt(model1[0]**2+model1[1]**2+model1[2]**2))

    rotated_corner2 = corner2.rotate(R, center=(0, 0, 0))
    

    # rotated_points = np.matmul(plane2_points, R)

    rotated_points = np.asarray(rotated_corner2.points)

    model3, index3 = PlaneRegression(rotated_points, threshold=0.09, init_n=3, iter=2000)

    unit_vector_3 = model3[:3] / np.linalg.norm(model3[:3])


    # unit_vector_3 = model3[:3]/ np.linalg.norm(model2[:3])

    offset = (model1[-1]-model3[-1])/math.sqrt(model1[0]**2+model1[1]**2+model1[2]**2)


    dot_product = np.dot(unit_vector_3, np.array([1, 0, 0]))
    # cross_product = np.cross(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)

    print("SECOND LAST ANGLE IS :  ", rad_2_degree(angle))


    if angle > 0 and angle <= np.pi/4:

        print("TEST  --------- XXXX")
        
        x_offset = offset * np.cos(angle)
        y_offset = offset * np.sin(angle)

        print("Guess for x-Axis translation ", x_offset, " with addition y offset ", y_offset)
        
        aligned_plane2 = rotated_corner2.translate((-x_offset, -y_offset, -z_offset))
    
    elif angle >= np.pi/4 and angle <= np.pi/2:

        print("TEST  --------- YYYYY")
        x_offset = offset * np.cos(angle)
        y_offset = offset * np.sin(angle)
        print("Guess for y-Axis translation ", y_offset, " with addition x offset ", x_offset)
        

        aligned_plane2 = rotated_corner2.translate((-x_offset, -y_offset, -z_offset))
    

    # aligned_points = rotated_points[:,-1] - z_offset
    # aligned_points =aligned_points.reshape(-1,1)

    DrawResultWithNormals(Corner1 + aligned_plane2)

    aligned_plane2 = rotated_corner2.select_by_index(index3, invert=True)

    last_plane1 = corner1.select_by_index(index1, invert=True)

    return last_plane1, aligned_plane2, R, x_offset, y_offset

def getLastRough(plane1, plane2, tx, ty):
    plane1_points = np.asarray(plane1.points)
    plane2_points = np.asarray(plane2.points)
    model1, index1 = PlaneRegression(plane1_points, threshold=0.09, init_n=3, iter=2000)
    model2, index2 = PlaneRegression(plane2_points, threshold=0.09, init_n=3, iter=2000)

    unit_vector_1 = model1[:3] / np.linalg.norm(model1[:3])
    unit_vector_2 = model2[:3]/ np.linalg.norm(model2[:3])

    dot_product = np.dot(unit_vector_1, unit_vector_2)
    cross_product = np.cross(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)

    # print(cross_product)
    
    K = np.array([[0, -cross_product[0], cross_product[1]],
    [cross_product[2], 0, -cross_product[2]],
    [-cross_product[1], cross_product[0], 0]])

    R = np.identity(3) + math.sin(angle) * K + (1 - math.cos(angle)) * np.matmul(K,K)
    print(R)

    quat = [math.sin(angle/2)*cross_product[0], math.sin(angle/2)*cross_product[1], math.sin(angle/2)*cross_product[2], math.cos(angle/2)]
    angle_axis = [cross_product[0], cross_product[1], cross_product[2], angle]
    r = Rotation.from_quat(quat)
    print("Euler angle zyx")
    print(r.as_euler('zyx', degrees=False))

    print("Angle Axis Form : ", angle_axis)


    rotated_plane2 = plane2.rotate(R, center=(0, 0, 0))
    

    # rotated_points = np.matmul(plane2_points, R)

    rotated_points = np.asarray(rotated_plane2.points)

    model3, index3 = PlaneRegression(rotated_points, threshold=0.09, init_n=3, iter=2000)

    unit_vector_3 = model3[:3] / np.linalg.norm(model3[:3])


    # unit_vector_3 = model3[:3]/ np.linalg.norm(model2[:3])

    offset = (model1[-1]-model3[-1])/math.sqrt(model1[0]**2+model1[1]**2+model1[2]**2)

    dot_product = np.dot(unit_vector_3, np.array([1, 0, 0]))
    # cross_product = np.cross(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)
    print("LAST ANGLE IS :  ", angle)

    print("LAST ANGLE IS :  ", rad_2_degree(angle))


    if angle > 0 and angle <= np.pi/4:

        print("LAST  --------- XXXX")
        
        x_offset = offset * np.cos(angle)
        y_offset = offset * np.sin(angle)
        print("Guess for x-Axis translation ", x_offset, " with addition x offset ", y_offset)
    
        aligned_plane2 = rotated_plane2.translate((-x_offset, 0, 0))
    
    # elif angle >= np.pi/4 and angle <= np.pi/2:
    else:
        print(angle)
        print(rad_2_degree(angle))

        print("LAST  --------- YYYYY")
        x_offset = offset * np.cos(angle)
        y_offset = offset * np.sin(angle)
        print("Guess for y-Axis translation ", y_offset, " with addition x offset ", x_offset)
        

        aligned_plane2 = rotated_plane2.translate((0, -y_offset, 0))


    return last_plane1, aligned_plane2, R, tx+x_offset, ty + y_offset




if __name__ == "__main__":

    ## Scene 1
    # source = o3d.io.read_point_cloud('Data/scene1a_ouster2.pcd')
    # target = o3d.io.read_point_cloud('Data/scene1a_ouster3.pcd')
    
    ## Scene 2
    # source = o3d.io.read_point_cloud('Data/scene2b_ouster2.pcd')
    # target = o3d.io.read_point_cloud('Data/scene2b_ouster3.pcd')
    
    ## Scene 3
    source = o3d.io.read_point_cloud('Data/corner_back_ouster2.pcd')
    target = o3d.io.read_point_cloud('Data/corner_back_ouster3.pcd')

    ## Scene 4
    # source = o3d.io.read_point_cloud('Data/scene4bbb_ouster2.pcd')
    # target = o3d.io.read_point_cloud('Data/scene4bbb_ouster3.pcd')

    ## Scene 5
    # source = o3d.io.read_point_cloud('Data/corner_front_n_back_ouster2.pcd')
    # target = o3d.io.read_point_cloud('Data/corner_front_n_back_ouster3.pcd')  
    

    ## Draw the point Cloud Data
    Draw2Clouds(source, target)


    ## Extract the ground planes
    GP1 = PreprocessCloud(source, False)
    GP2 = PreprocessCloud(target, False)


    # Draw ground both extracted Planes
    Draw2Clouds(GP1, GP2)
    
    # DrawResultWithNormals(GP1)

    # Align the ground plane
    alignedGP2, R1, tz = GPAlignment(GP1, GP2)

    # Draw Results after ground plane alignment
    Draw2Clouds(GP1, alignedGP2)

    # Extract the vertical planes
    OP1 = PreprocessCloud(source, gp_removal=True)
    OP2 = PreprocessCloud(target, gp_removal=True)

    Draw2Clouds(OP1, OP2)
    
    # Extract Dominent Container Clusters
    Cluster1 = getBestCluster(OP1)
    Cluster2 = getBestCluster(OP2)

    # Draw2Clouds(Cluster1, Cluster2)

    # Extract the dominent orthogonal planes in the clusters
    Corner1, planes1, models1 = getCornerPlanes(Cluster1)
    Corner2, planes2, models2 = getCornerPlanes(Cluster2)


    # To save the all three orthogonal planes for refinement

    GP2 = PreprocessCloud(target, False)
    Draw2Clouds(GP1+Corner1, GP2 + Corner2)

    # DrawResultWithNormals(Cluster1+Cluster2)
    # DrawResultWithNormals(Corner1 + Corner2)

    o3d.io.write_point_cloud("ExtractedPlanes/Cornera.pcd", Corner1+GP1)
    o3d.io.write_point_cloud("ExtractedPlanes/Cornerb.pcd", Corner2+GP2)


    # Yaw and x/y aligment 
    last_plane1, alighnedYaw2, rotation2, tx, ty = getYaw(Corner1, models1, Corner2, models2, tz)

    # Last x/y aligment
    only_plane1, alighned2, rotation3, tx, ty = getLastRough(last_plane1, alighnedYaw2, tx, ty)

    rotation_rough = np.matmul(np.matmul(R1, rotation2), rotation3)
    print(rotation_rough)

    print(tx, ty, tz)
    Draw2Clouds(last_plane1, alighnedYaw2)