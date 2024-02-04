from sklearn.cluster import DBSCAN

def cluster(points):
        '''
        Returns clusters from 3D data

        Input
        :param point: nx3 array where n is the number of 3D points present

        :return:  An array of labels where each point is labelled into a cluster. 
                Labelled -1 if it does not belong to any cluster
        '''

        model = DBSCAN(eps=0.05, min_samples=20)
        model.fit_predict(points)
        pred = model.fit_predict(points)

        print("number of cluster found: {}".format(len(set(model.labels_))))
        print('cluster for each point: ', model.labels_)
        return model.labels_

from numpy import array, linspace
from sklearn.neighbors import KernelDensity
from matplotlib.pyplot import plot
import time

# def kde():
#         t1 = time.time()
#         '''
#         Kernel Density Estimation
#         '''
#         a = array([1000]).reshape(-1, 1)
#         kde = KernelDensity(kernel='gaussian', bandwidth=0.01).fit(a)
#         s = linspace(0,1000)
#         e = kde.score_samples(s.reshape(-1,1))
#         t2 = time.time()
#         print(e)
#         print(t2-t1)
#         plot(s, e)

# kde()

t1 = time.time()
a = array([1000]).reshape(-1, 1)
kde = KernelDensity(kernel='gaussian', bandwidth=0.01).fit(a)
s = linspace(0,1000)
e = kde.score_samples(s.reshape(-1,1))
t2 = time.time()
print(e)
print(t2-t1)
plot(s, e)
