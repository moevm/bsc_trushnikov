import numpy as np
import open3d as o3d
from scipy.spatial import distance
import time
from pykdtree.kdtree import KDTree
import math
from progress.bar import IncrementalBar
from progress.bar import Bar

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


def divide(X, p_thres, range_):
    delta = 1e-10
    range_[:, 0] = range_[:, 0] - delta
    range_[:, 1] = range_[:, 1] + delta
    p = (X[:, 0] > range_[0, 0]) & (X[:, 0] < range_[0, 1]) & (X[:, 1] > range_[1, 0]) & (
            X[:, 1] < range_[1, 1]) & (X[:, 2] > range_[2, 0]) & (X[:, 2] < range_[2, 1])
    Y = X[p, :]
    n = Y.shape[0]

    GRID_NUM = round(n / p_thres)
    VOLUMN = np.prod(range_[:, 1] - range_[:, 0])
    GRID_LEN = np.cbrt(VOLUMN / GRID_NUM)
    GRID_NUM = np.ceil((range_[:, 1] - range_[:, 0]) / GRID_LEN).astype(int)
    GRID_LEN = (range_[:, 1] - range_[:, 0]) / GRID_NUM

    grid_len = 0
    grid_range = []
    grid_X = []

    for i in range(1, GRID_NUM[0] + 1):
        x_min = range_[0, 0] + GRID_LEN[0] * (i - 1)
        x_max = range_[0, 0] + GRID_LEN[0] * i
        GRID_DELTA = GRID_LEN[0] * 0.1
        tmpi = X[(X[:, 0] > (x_min - GRID_DELTA)) & (X[:, 0] < (x_max + GRID_DELTA))]
        for j in range(1, GRID_NUM[1] + 1):
            y_min = range_[1, 0] + GRID_LEN[1] * (j - 1)
            y_max = range_[1, 0] + GRID_LEN[1] * j
            GRID_DELTA = GRID_LEN[1] * 0.1
            tmpj = tmpi[(tmpi[:, 1] > (y_min - GRID_DELTA)) & (tmpi[:, 1] < (y_max + GRID_DELTA))]
            for k in range(1, GRID_NUM[2] + 1):
                z_min = range_[2, 0] + GRID_LEN[2] * (k - 1)
                z_max = range_[2, 0] + GRID_LEN[2] * k
                GRID_DELTA = GRID_LEN[2] * 0.1
                tmpk = tmpj[(tmpj[:, 2] > (z_min - GRID_DELTA)) & (tmpj[:, 2] < (z_max + GRID_DELTA))]

                if tmpk.shape[0]:
                    grid_X.append(tmpk)
                    grid_range.append([x_min, x_max, y_min, y_max, z_min, z_max])
                    grid_len += 1
    return [grid_X, grid_range, grid_len]


def getNormals(all_p, k, points=None):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_p)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=k))
    normals = np.asarray(pcd.normals)
    if points is not None:
        return normals[points]
    else:
        return normals


def simplify_cube(points, n_e_points,e_points, range_):
    if n_e_points.shape[0] <= 2:
        return points

    target = int(math.ceil((n_e_points.shape[0] * 4) / 100))
    if target == 0:
        target +=1
    k = 10
    if k > target + e_points.shape[0]:
        k = target + e_points.shape[0]
    if k <= 1:
        k = 2

    if k >= points.shape[0]:
        k = points.shape[0] - 1

    all_neib, masks = getNeighbors(points, points, k, False, e_points)

    neib_idx = all_neib[n_e_points]
    masks = masks[n_e_points]

    normals = getNormals(points, 20, neib_idx)
    importance = calc_importance(points[neib_idx], points[n_e_points], normals, masks)

    while n_e_points.shape[0] != target:
       # print(n_e_points.shape[0], ' != ', target)

        imp_id = np.argmin(importance)
        id = n_e_points[imp_id]

        affected_p_1_idx = np.where(neib_idx == id)[0]
        affected_exist = False
        affected_p_1 = []
        if affected_p_1_idx.shape[0]:
            affected_p_1_idx = np.unique(affected_p_1_idx)
            affected_p_1 = np.asarray(n_e_points[affected_p_1_idx])
            affected_exist = True
            affected_p_1_idx[affected_p_1_idx > imp_id] -= 1

        points = np.delete(points, id, axis=0)
        if points.shape[0] <= 1:
            break
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
        prep_p[prep_p == points.shape[0]] -= 1
        normals_ = getNormals(points[prep_p], k - 1)
        normals = []

        for i in prep_idx:
            normals.append(normals_[i])
        imp = calc_importance(points[neibs], points[points_], np.asarray(normals), masks_)
        importance[indexes] = imp

    delta = 1e-10
    range_[:, 0] = range_[:, 0] - delta
    range_[:, 1] = range_[:, 1] + delta

    p = (points[:, 0] > range_[0, 0]) & (points[:, 0] < range_[0, 1]) & (points[:, 1] > range_[1, 0]) & (
            points[:, 1] < range_[1, 1]) & (points[:, 2] > range_[2, 0]) & (points[:, 2] < range_[2, 1])
    points = points[p]
    return points


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


def simplify(pcd):
    t = time.time()
    p_thres_min = 3000
    p_thres_max = 8000
    X = np.asarray(pcd.points)
    e_points, n_e_points = detect_edge_points(X, 30, 5)

    n = X.shape[0]
    range_ = np.array([X.min(axis=0), X.max(axis=0)]).T
    GRID_NUM = round(n / p_thres_min)
    VOLUMN = np.prod(range_[:, 1] - range_[:, 0])
    GRID_LEN = np.cbrt(VOLUMN / GRID_NUM)
    GRID_NUM = np.ceil((range_[:, 1] - range_[:, 0]) / GRID_LEN).astype(int)
    GRID_LEN = (range_[:, 1] - range_[:, 0]) / GRID_NUM
    m = 0
    simpX = [[0, 0, 0]]

    num_cycles = GRID_NUM[0] * GRID_NUM[1] * GRID_NUM[2]
    bar = IncrementalBar('Simplifying...', max=num_cycles)
    bar.goto(0)

    for i in range(1, GRID_NUM[0] + 1):
        x_min = range_[0, 0] + GRID_LEN[0] * (i - 1)
        x_max = range_[0, 0] + GRID_LEN[0] * i
        GRID_DELTA = GRID_LEN[0] * 0.1
        tmpi = X[(X[:, 0] > (x_min - GRID_DELTA)) & (X[:, 0] < (x_max + GRID_DELTA))]
        for j in range(1, GRID_NUM[1] + 1):
            y_min = range_[1, 0] + GRID_LEN[1] * (j - 1)
            y_max = range_[1, 0] + GRID_LEN[1] * j
            GRID_DELTA = GRID_LEN[1] * 0.1
            tmpj = tmpi[(tmpi[:, 1] > (y_min - GRID_DELTA)) & (tmpi[:, 1] < (y_max + GRID_DELTA))]
            for k in range(1, GRID_NUM[2] + 1):
                z_min = range_[2, 0] + GRID_LEN[2] * (k - 1)
                z_max = range_[2, 0] + GRID_LEN[2] * k
                GRID_DELTA = GRID_LEN[2] * 0.1
                tmpk = tmpj[(tmpj[:, 2] > (z_min - GRID_DELTA)) & (tmpj[:, 2] < (z_max + GRID_DELTA))]
                tmprange = np.array([[x_min, x_max], [y_min, y_max], [z_min, z_max]])

                if len(tmpk):
                    if len(tmpk) > p_thres_max:
                        grid_X, grid_range, grid_len = divide(tmpk, p_thres_min, tmprange)
                        for p in range(grid_len):
                            tmpk = np.array(grid_X[p])
                            mask = np.isin(tmpk, X[e_points])
                            mask = np.logical_and(np.logical_and(mask[:, 0], mask[:, 1]), mask[:, 2])
                            mask_n_e = np.arange(mask.shape[0])[np.logical_not(mask)]
                            mask_e = np.arange(mask.shape[0])[mask]
                            tmp = simplify_cube(tmpk, mask_n_e, mask_e, np.array(grid_range[p]).reshape(3, 2))
                            simpX = np.append(simpX, tmp, axis=0)
                    else:
                        #print(np.asarray(X[n_e_points][np.where(np.isin(X[n_e_points],tmpk) == [True,True,True] )[0]]).shape[0])
                        mask = np.isin(tmpk,X[e_points])
                        mask = np.logical_and(np.logical_and(mask[:,0],mask[:,1]),mask[:,2])
                        mask_n_e = np.arange(mask.shape[0])[np.logical_not(mask)]
                        mask_e = np.arange(mask.shape[0])[mask]

                        tmp = simplify_cube(tmpk, mask_n_e, mask_e,tmprange)
                        simpX = np.append(simpX, tmp, axis=0)
                bar.next()

    bar.finish()
    simpX = np.delete(simpX, 0, axis=0)
    return [simpX, time.time() - t]