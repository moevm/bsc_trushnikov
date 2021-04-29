import numpy as np
import qpsolvers
import time
from progress.bar import IncrementalBar


def simplify(alpha, lambda_, k, X, range_):
    n = X.shape[0]
    simpX = np.array([])
    if n * alpha < 1:
        simpX = X
    else:
        k = min(k, n - 1)
        L = np.zeros((n, n))
        A = np.zeros((n, n))
        sigma = 0
        for i in range(0, n):
            d = np.sum(np.power(X - X[i, :], 2), axis=1)
            p = np.argsort(d)
            clear_p = p[k + 1:p.shape[0]]
            d[i] = 0.0
            if clear_p.size:
                d[clear_p] = 0.0
            # ones_p = d[np.where(d != 0)[0]]
            ones_p = d != 0
            sigma = max(max(d), sigma)
            A[i, ones_p] = 1
            L[i, :] = d
        p = (A == 1)
        L[p] = np.exp(-L[p] / sigma)

        A = (A + np.transpose(A)) * 0.5
        L = (L + np.transpose(L)) * 0.5

        d = np.sum(L, axis=1)
        L = np.eye(n, dtype=int) - (L / d[:, None])

        f = np.diag(np.dot(np.dot(L, X), np.dot(L, X).T))

        delta = 1e-10
        range_[:, 0] = range_[:, 0] - delta
        range_[:, 1] = range_[:, 1] + delta

        p = (X[:, 0] > range_[0, 0]) & (X[:, 0] < range_[0, 1]) & (X[:, 1] > range_[1, 0]) & (
                X[:, 1] < range_[1, 1]) & (X[:, 2] > range_[2, 0]) & (X[:, 2] < range_[2, 1])
        f = f[p]
        A = A[p]
        A = A[:, p]
        d = np.subtract(k, np.sum(A, axis=1))
        A = A + np.diag(d)
        X = X[p, :]
        n = len(f)
        if n:
            m = round(n * alpha)
            L = np.diag(f) + lambda_ * np.dot(A.T, A)
            f = f + np.dot(lambda_ * alpha * k * A, np.ones((n, 1))).T
            f = f[0]

            # start_time = time.time()
            # print('start solving ', n)
            d = qpsolvers.solve_qp(L, -f, None, None, np.ones((1, n))[0], np.array([m]).astype(float),
                                   np.zeros((1, n))[0], np.ones((1, n))[0])
            # print("--- %s seconds ---" % (time.time() - start_time))
            p = np.argsort(-np.abs(d))
            simpX = X[p[0:m], :]

    return simpX


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


def gridding(pcd):
    t = time.time()
    alpha = 0.1
    lambda_ = 1e-3
    K = 15
    p_thres_min = 3000
    p_thres_max = 8000
    X = np.asarray(pcd.points)
    n = X.shape[0]
    range_ = np.array([X.min(axis=0), X.max(axis=0)]).T
    GRID_NUM = round(n / p_thres_min)
    VOLUMN = np.prod(range_[:, 1] - range_[:, 0])
    GRID_LEN = np.cbrt(VOLUMN / GRID_NUM)
    GRID_NUM = np.ceil((range_[:, 1] - range_[:, 0]) / GRID_LEN).astype(int)
    GRID_LEN = (range_[:, 1] - range_[:, 0]) / GRID_NUM
    m = 0
    simpX = [[0, 0, 0]]

    num_cycles = GRID_NUM[0]*GRID_NUM[1]*GRID_NUM[2]
    bar = IncrementalBar('Simplifying...', max=num_cycles)
    #print('num_cycles',num_cycles)
    counter = 1
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
               # print('simplifying i = (', i, ';', GRID_NUM[0], ') j = (', j, ';', GRID_NUM[1], ') k = (', k,
                 #     ';', GRID_NUM[2], ')')
                z_min = range_[2, 0] + GRID_LEN[2] * (k - 1)
                z_max = range_[2, 0] + GRID_LEN[2] * k
                GRID_DELTA = GRID_LEN[2] * 0.1
                tmpk = tmpj[(tmpj[:, 2] > (z_min - GRID_DELTA)) & (tmpj[:, 2] < (z_max + GRID_DELTA))]
                tmprange = np.array([[x_min, x_max], [y_min, y_max], [z_min, z_max]])

                if len(tmpk):
                    #print(len(tmpk))
                    if len(tmpk) > p_thres_max:
                       # print('dividing...')
                        grid_X, grid_range, grid_len = divide(tmpk, p_thres_min, tmprange)
                        for p in range(grid_len):
                            # print(np.array(grid_X[p]))
                            # print(np.array(grid_range[p]).reshape((3, 2)),'\n')
                            tmp = simplify(alpha, lambda_, K, np.array(grid_X[p]),
                                           np.array(grid_range[p]).reshape((3, 2)))
                            if len(tmp):
                                simpX = np.append(simpX, tmp, axis=0)
                    else:
                        tmp = simplify(alpha, lambda_, K, tmpk, tmprange)
                        if len(tmp):
                            simpX = np.append(simpX, tmp, axis=0)
                #print(math.ceil((counter/num_cycles)*100))
                bar.next()
                #counter+=1

    bar.finish()
    simpX = np.delete(simpX, 0, axis=0)
    return [simpX,time.time() - t]
