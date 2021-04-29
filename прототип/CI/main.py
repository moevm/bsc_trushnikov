import sys
import open3d as o3d
import numpy as np
from backend.second import gridding
from backend.first import simplify
from pykdtree.kdtree import KDTree
import math
import os


def rmse_estimate(xyz_source, xyz_target):
    tree = KDTree(xyz_target)
    rmse = 0.0
    intensity = []
    for index, point in enumerate(xyz_source):
        dist, neighbor = tree.query(np.array([point]), 1)
        intensity.append(dist)
        rmse += dist
    rmse = math.sqrt(rmse / xyz_source.shape[0])
    return rmse


if __name__ == '__main__':
    source = []
    target = []
    results = []
    path_save = ' '
    if len(sys.argv) == 2:
        print("Сообщение: Сжатие вторым методом")
        pcd = o3d.io.read_point_cloud(sys.argv[1])
        if np.asarray(pcd.points).shape[0] == 0:
            print("Ошибка: Неверно указанный путь")
            sys.exit(1)
        source = np.asarray(pcd.points)
        results = gridding(pcd)
        target = np.asarray(results[0])
        path_save = os.path.split(sys.argv[1])[0] + '/' + os.path.split(sys.argv[1])[1].split('.')[0] + '_simplified.' + \
                    os.path.split(sys.argv[1])[1].split('.')[1]

    elif len(sys.argv) == 3:
        print("Сообщение: Сжатие первым методом")
        pcd = o3d.io.read_point_cloud(sys.argv[1])
        if np.asarray(pcd.points).shape[0] == 0:
            print("Ошибка: Неверно указанный путь")
            sys.exit(1)
        elif int(sys.argv[2]) <= 0 or int(sys.argv[2]) > 100:
            print("Ошибка: Неверно указанный процент сжатия облака точек")
            sys.exit(1)
        source = np.asarray(pcd.points)
        results = np.asarray(simplify(pcd, int(sys.argv[2])))
        target = results[0]
        sys.argv[2] = results[2]
        path_save = os.path.split(sys.argv[1])[0] + '/' + os.path.split(sys.argv[1])[1].split('.')[0] + '_' + str(
            int(sys.argv[2])) + '%.' + os.path.split(sys.argv[1])[1].split('.')[1]
    else:
        sys.exit(1)

    print('Результаты.')
    print('Исходное кол-во точек:', source.shape[0])
    print('После сжатия:', target.shape[0])
    print('RMSE:', rmse_estimate(source, target))
    print('Затраченное время:', round(results[1], 2), 'секунд')
    print('Облако точек сохраненно по пути:', path_save)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(target)
    o3d.io.write_point_cloud(path_save, pcd)
