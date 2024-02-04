def find_cluster1D(points,eps,q):
    labels=[]
    count=0
    for i in points:
        if i>q-eps and i<q+eps:
            labels.append(1) 
        else: 
            labels.append(0)
    return labels

print(find_cluster1D([1,2,3,4,5,6,7,8,9,10],2,5))
        


