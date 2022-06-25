import numpy as np
import open3d as o3d


def rad_2_degree(angle):
    return angle * 180 / np.pi

def degree2rad(angle):
    return angle *  np.pi /180

def ReadPlyPoint(fname):
    """ read point from ply

    Args:
        fname (str): path to ply file

    Returns:
        [ndarray]: N x 3 point clouds
    """

    pcd = o3d.io.read_point_cloud(fname)

    return PCDToNumpy(pcd)


def NumpyToPCD(xyz):
    """ convert numpy ndarray to open3D point cloud 

    Args:
        xyz (ndarray): 

    Returns:
        [open3d.geometry.PointCloud]: 
    """

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    return pcd


def PCDToNumpy(pcd):
    """  convert open3D point cloud to numpy ndarray

    Args:
        pcd (open3d.geometry.PointCloud): 

    Returns:
        [ndarray]: 
    """

    return np.asarray(pcd.points)


def RemoveNan(points):
    """ remove nan value of point clouds

    Args:
        points (ndarray): N x 3 point clouds

    Returns:
        [ndarray]: N x 3 point clouds
    """

    return points[~np.isnan(points[:, 0])]


def RemoveNoiseStatistical(pc, nb_neighbors=20, std_ratio=2.0):
    """ remove point clouds noise using statitical noise removal method

    Args:
        pc (ndarray): N x 3 point clouds
        nb_neighbors (int, optional): Defaults to 20.
        std_ratio (float, optional): Defaults to 2.0.

    Returns:
        [ndarray]: N x 3 point clouds
    """

    pcd = NumpyToPCD(pc)
    cl, ind = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio)

    return PCDToNumpy(cl)


def DownSample(pts, voxel_size=0.003):
    """ down sample the point clouds

    Args:
        pts (ndarray): N x 3 input point clouds
        voxel_size (float, optional): voxel size. Defaults to 0.003.

    Returns:
        [ndarray]: 
    """

    p = NumpyToPCD(pts).voxel_down_sample(voxel_size=voxel_size)

    return PCDToNumpy(p)


def PlaneRegression(points, threshold=0.01, init_n=3, iter=1000):
    """ plane regression using ransac

    Args:
        points (ndarray): N x3 point clouds
        threshold (float, optional): distance threshold. Defaults to 0.003.
        init_n (int, optional): Number of initial points to be considered inliers in each iteration
        iter (int, optional): number of iteration. Defaults to 1000.

    Returns:
        [ndarray, List]: 4 x 1 plane equation weights, List of plane point index
    """

    pcd = NumpyToPCD(points)

    w, index = pcd.segment_plane(
        threshold, init_n, iter)

    return w, np.array(index)


def DrawResult(points, colors):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd])


def DetectMultiPlanes(points, min_ratio=0.05, threshold=0.01, iterations=1000):
    """ Detect multiple planes from given point clouds

    Args:
        points (np.ndarray): 
        min_ratio (float, optional): The minimum left points ratio to end the Detection. Defaults to 0.05.
        threshold (float, optional): RANSAC threshold in (m). Defaults to 0.01.

    Returns:
        [List[tuple(np.ndarray, List)]]: Plane equation and plane point index
    """

    plane_list = []
    temp_list = []
    N = len(points)

    target = points.copy()
    count = 0

    while count < (1 - min_ratio) * N:
        w, index = PlaneRegression(
            target, threshold=threshold, init_n=3, iter=iterations)
    
        count += len(index)
        plane_list.append([w, target[index]])
        # temp_list.append(np.array([w, target[index]],dtype='object'))
        target = np.delete(target, index, axis=0)

    print(f"Numnber of points {count}")
    # print(np.array(temp_list[0][0]))

    return plane_list



def range_filter(cloud, r_min, r_max):
    points = np.asarray(cloud.points)

    # Removing Truck points
    center = np.array([0, 0, 0])
    radius = 3.5
    outer_boundary = 20

    distances = np.linalg.norm(points - center, axis=1)
    points = points[distances >= r_min]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    distances = np.linalg.norm(points - center, axis=1)
    points = points[distances <= r_max]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    return pcd

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)

    # o3d.visualization.draw([source_temp, target_temp])
    o3d.visualization.draw_geometries([source_temp, target_temp], point_show_normal=False)


def Draw2Clouds(cloud1, cloud2):
    cloud1.paint_uniform_color([1, 0, 0])
    cloud2.paint_uniform_color([0, 1, 0])

    o3d.visualization.draw_geometries([cloud1, cloud2],point_show_normal = False)

def DrawResultWithNormals(cloud):    
    o3d.visualization.draw_geometries([cloud],point_show_normal = False)

def getColoredPlanes(points, colors):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    # o3d.visualization.draw_geometries([pcd],point_show_normal = False)

    return pcd