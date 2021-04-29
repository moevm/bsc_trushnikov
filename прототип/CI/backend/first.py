import numpy as np
import open3d as o3d
from scipy.spatial import distance
import time
from pykdtree.kdtree import KDTree
import math
from progress.bar import Bar


def detect_edge_points(all_p, k1, k2):
    neighbors_idx_1 = getNeighbors(all_p, all_p, k1, False)
    lambda_ = 1.45

    # e_points = []
    # n_e_points = []

    # for i, x in enumerate(all_p):
    #    c_i = all_p[neighbors_idx_1[i]].mean(axis=0)
    #     z = np.min(distance.cdist([x], all_p[neighbors_idx_1[i]], 'euclidean')[0])
    #     dist = distance.euclidean(x, c_i)
    #     if dist > lambda_ * z:
    #        e_points.append(i)
    #     else:
    #        n_e_points.append(i)
    bar = Bar('Searching...', fill='#', suffix='%(percent)d%%')
    progress = 1
    weights = []
    for i, x in enumerate(all_p):
        if round((i/all_p.shape[0])*100,2) >= progress:
            bar.next()
            progress+=1
        c_i = all_p[neighbors_idx_1[i]].mean(axis=0)
        z = np.min(distance.cdist([x], all_p[neighbors_idx_1[i]], 'euclidean')[0])
        dist = distance.euclidean(x, c_i)
        weights.append([z, dist])
    bar.next()
    per = 100
    while per > 15:
        bar.goto(0)
        progress = 1
        e_points = []
        n_e_points = []
        for i, x in enumerate(all_p):
            if round((i / all_p.shape[0]) * 100, 2) >= progress:
                bar.next()
                progress += 1
            if weights[i][1] > lambda_ * weights[i][0]:
                e_points.append(i)
            else:
                n_e_points.append(i)
        bar.next()
        per = (100 * len(e_points)) / len(all_p)
        lambda_ += 0.01
    bar.finish()
    return np.array(e_points), np.array(n_e_points)


def getNeighbors(all_p, points, k, print_, e_points=None):
    tree = KDTree(all_p)

    if points.ndim == 1:
        points = np.array([points])
    dist, neighbors = tree.query(points, k=k)
    if neighbors.shape[0] == 1 and neighbors.ndim == 2:
        neighbors = neighbors[0]

    masks = []
    if neighbors.ndim > 1:
        neighbors = neighbors[:, :][:, 1:k]

        if print_:
            print(neighbors, '\n')

        if e_points is not None:
            masks = np.isin(neighbors, e_points, invert=True)
            return np.array(neighbors), np.asarray(masks)
    else:
        neighbors = neighbors[1:k]
        if e_points is not None:
            masks.append(np.isin(neighbors, e_points, invert=True))
            return np.array(neighbors), np.asarray(masks)
    return np.array(neighbors)


def getNormals(all_p, k, points=None):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_p)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=k))
    normals = np.asarray(pcd.normals)
    if points is not None:
        return normals[points]
    else:
        return normals


def simplify(pcd, percent):
    t = time.time()
    k = 10
    points = np.asarray(pcd.points)
    delete_p_list = []
    e_points, n_e_points = detect_edge_points(points, 30, 5)
   # print(e_points.shape[0], n_e_points.shape[0], points.shape[0])

    all_neib, masks = getNeighbors(points, points, k, False, e_points)
    neib_idx = all_neib[n_e_points]
    masks = masks[n_e_points]

    normals = getNormals(points, 20, neib_idx)
    importance = calc_importance(points[neib_idx], points[n_e_points], normals, masks)

    target = int((points.shape[0] * percent) / 100)
    target -= e_points.shape[0]

    percent_changed = False
    while target <= 0:
        percent_changed = True
        percent+=1
        target = int((points.shape[0] * percent) / 100)
        target -= e_points.shape[0]
    if percent_changed:
        print('Процент сжатия сменен на', percent, '%')

    print('Сообщение: В сжатом облаке будет', target + e_points.shape[0], 'точек и из них', e_points.shape[0],
          'точек граничные.')
    bar = Bar('Simplifying...', fill='#', suffix='%(percent)d%%')
    source = n_e_points.shape[0] - target
    progress = 1
    count = 1
    bar.goto(0)
    while n_e_points.shape[0] != target:
        bar.goto(round((count/source)*100,2))
       # if round((count/source)*100,2) >= progress:
        #    bar.next()
        #    progress+=1

        imp_id = np.argmin(importance)
        id = n_e_points[imp_id]
        delete_p_list.append(id)
        count+=1

        affected_p_1_idx = np.where(neib_idx == id)[0]
        affected_exist = False
        affected_p_1 = []
        if affected_p_1_idx.shape[0]:
            affected_p_1_idx = np.unique(affected_p_1_idx)
            affected_p_1 = np.asarray(n_e_points[affected_p_1_idx])
            affected_exist = True
            affected_p_1_idx[affected_p_1_idx > imp_id] -= 1

        points = np.delete(points, id, axis=0)
        n_e_points = np.delete(n_e_points, imp_id)
        importance = np.delete(importance, imp_id)
        neib_idx = np.delete(neib_idx, imp_id, axis=0)
        masks = np.delete(masks, imp_id, axis=0)
        n_e_points[n_e_points > id] -= 1
        e_points[e_points > id] -= 1
        neib_idx[neib_idx > id] -= 1
        all_neib[all_neib > id] -= 1

        if not affected_exist:
            continue

        affected_p_1[affected_p_1 > id] -= 1

        neib_idx1, masks1 = getNeighbors(points, points[affected_p_1], k, False, e_points)
        all_neib[affected_p_1] = neib_idx1
        neib_idx[affected_p_1_idx] = neib_idx1
        masks[affected_p_1_idx] = masks1

        if neib_idx1.ndim == 1:
            neib_idx1 = np.array([neib_idx1])
        if masks1.ndim == 1:
            masks1 = np.array([masks1])

        affected_p_2_idx = np.unique(np.where(np.isin(neib_idx, affected_p_1) == True)[0])
        affected_p_2 = np.asarray(n_e_points[affected_p_2_idx])

        neib_idx2 = neib_idx[affected_p_2_idx]
        masks2 = masks[affected_p_2_idx]

        if neib_idx2.ndim == 1:
            neib_idx2 = np.array([neib_idx2])
        if masks2.ndim == 1:
            masks2 = np.array([masks2])

        if neib_idx2.shape[0] and neib_idx2[masks2].shape[0]:
            neibs = np.concatenate((neib_idx1, neib_idx2), axis=0)
            masks_ = np.concatenate((masks1, masks2), axis=0)
            indexes = np.concatenate((affected_p_1_idx, affected_p_2_idx), axis=0)
            points_ = np.concatenate((affected_p_1, affected_p_2), axis=0)
            points_, unique_indx = np.unique(points_, return_index=True)
            neibs = neibs[unique_indx]
            masks_ = masks_[unique_indx]
            indexes = indexes[unique_indx]
        else:
            neibs = neib_idx1
            masks_ = masks1
            points_ = affected_p_1
            indexes = affected_p_1_idx

        prep_p1 = neibs.reshape(1, neibs.shape[0] * neibs.shape[1])[0]
        prep_p2 = all_neib[prep_p1]
        prep_p2 = prep_p2.reshape(1, prep_p2.shape[0] * prep_p2.shape[1])[0]
        prep_p = np.append(prep_p1, prep_p2, axis=0)
        prep_p = np.append(prep_p, points_, axis=0)
        prep_p = np.unique(prep_p)
        prep_idx = (prep_p == neibs[..., None]).any(axis=1)
        prep_p[prep_p > id] -= 1
        normals_ = getNormals(points[prep_p], k - 1)
        normals = []

        for i in prep_idx:
            normals.append(normals_[i])

        imp = calc_importance(points[neibs], points[points_], np.asarray(normals), masks_)
        importance[indexes] = imp

    bar.finish()
   # print(time.time() - t)
    #points = points.tolist()
    #points.append(time.time() - t)
    return [points, time.time() - t, percent]


def calc_importance(neighbors, points, normals, masks):
    val1 = np.where(masks == True)[0]
    val1, val2 = np.unique(val1, return_counts=True)
    importance = np.zeros(masks.shape[0])
    counts = np.ones(masks.shape[0])
    counts[val1] = val2
    masks = np.logical_not(masks)
    points = points[:, np.newaxis, :]
    imp_val = points - neighbors
    imp_val[masks] = [0, 0, 0]
    normals[masks] = [0, 0, 0]
    imp_val = np.sum(np.abs(np.sum(np.einsum('...ij,...ij->...ij', imp_val, normals), axis=2)), axis=1)
    importance[val1] = imp_val[val1]
    importance = importance / counts

    return np.asarray(importance)