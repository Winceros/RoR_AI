__author__ = 'Ildar_Gilmutdinov'

import numpy as np
import matplotlib.pyplot as plt
import math
from scipy import misc

map = np.fromfile('C:\Users\Ildar_Gilmutdinov\Documents\Python projects\RoR_AI\map2.raw',dtype=np.uint16)
#map = misc.imread('C:\Users\Ildar_Gilmutdinov\Documents\Python projects\RoR_AI\map2.png')
#print map.dtype, map.shape
map.shape = (513,513)


from scipy.ndimage.filters import generic_filter
def roughness(d,ci):
    return np.abs((d-d[ci]).max())

def topologicalRuggednessIndex(d,ci):
    #return (d-d[ci]).mean()
    return d[ci]/d.mean()

def topologicalPositionIndex(d,ci):
    return d[ci] - d.mean()

def neighbDiffMean(d):
    #bins_num = int(d.max() - d.min())
    #if bins_num <= 4:
    #    return 0
    #x,bins = np.histogram(d,bins = bins_num)

    diff = abs(d[1:] - d[:-1])
    return int(diff.mean())
#def countOfDiff

size = 9
extra_argument = 40
#f1 = generic_filter(map, roughness, size, mode='nearest', extra_arguments=(extra_argument,))
#f2 = generic_filter(map, topologicalRuggednessIndex, size, mode='nearest', extra_arguments=(extra_argument,))
#f3 = generic_filter(map, topologicalPositionIndex, size, mode='nearest', extra_arguments=(extra_argument,))
#map2 = map.astype(int)/100
#f4 = generic_filter(map2, neighbDiffMean, size, mode='nearest')

def calcRoughMap(map):
    map_len = map.shape[0]
    roughMap = np.ndarray(shape=(map_len/5+1,map_len/5+1))
    for i in xrange(0,map_len,5):
        for i2 in xrange(0,map_len,5):
            d = map[i:i+5,i2:i2+5]
            bins_num = d.max() - d.min()
            if bins_num <= 4:
                roughMap[i/5,i2/5] = 0
                continue
            hist, bins = np.histogram(d,bins = bins_num)
            maxs = 0
            for i3 in xrange(1,len(hist)-1):
                diff1 = hist[i3-1] - hist[i3]
                diff2 = hist[i3] - hist[i3+1]
                if diff1 * diff2 < 0:
                    maxs += max(abs(diff1),abs(diff2))

            roughMap[i/5,i2/5] = maxs/len(bins)
    return roughMap

def calcRoughMap2(map):
    gx,gy = np.gradient(map)
    gt = (gx**2 + gy**2)**0.5
    map_len = map.shape[0]
    roughMap = np.ndarray(shape=(map_len/5+1,map_len/5+1))
    for i in xrange(0,map_len,5):
        for i2 in xrange(0,map_len,5):
            roughMap[i/5,i2/5] = np.sum(gt[i:i+5,i2:i2+5])
    print roughMap.min(),roughMap.max()
    return roughMap

map /= map.max()/62
plt.subplot(121)
plt.imshow(map,cmap=plt.cm.gray)
#plt.subplot(242)
#plt.imshow(f1,cmap=plt.cm.gray)
#plt.subplot(243)
#plt.imshow(f2,cmap=plt.cm.gray)
#plt.subplot(244)
#plt.imshow(f3,cmap=plt.cm.gray)
#plt.subplot(245)
#plt.imshow(f4,cmap=plt.cm.gray)
roughMap = calcRoughMap2(map/map.mean())
plt.subplot(122)
plt.imshow(roughMap,cmap=plt.cm.gray)
print map.min(),map.max()
plt.show()
'''
np.set_printoptions(threshold='nan')
#print map[0:30:4,290:320:4]/1000
#print map[320:350:4,420:450:4]/1000
def vrnc(x):
    return abs(x - x.mean()).sum()

def plot_grad(x):
    gx,gy = np.gradient(x)
    gt = (gx**2 + gy**2)**0.5
    plt.imshow((gt*10).astype(int))
    return gt

def plot_hist(x):
    bins_num = x.max() - x.min()
    hist,bins = np.histogram(x,bins = bins_num)
    #print hist
    width = 0.7 * (bins[1] - bins[0])
    center = (bins[:-1] + bins[1:]) / 2
    plt.bar(center, hist, align='center', width=width)

def neighbDiffMean(arr):
    x = arr.astype(int)
    #bins_num = x.max() - x.min()
    #x,bins = np.histogram(x,bins = bins_num)
    diff = abs(x[1:] - x[:-1])
    return int(diff.mean())

def countOfDiff(arr):
    x = arr.astype(int)

    bins_num = d.max() - d.min()
    if bins_num <= 4:
        roughMap[i/5,i2/5] = 0
    hist, bins = np.histogram(d,bins = bins_num)
    maxs = 0
    for i3 in xrange(1,len(hist)-1):
        diff1 = hist[i3-1] - hist[i3]
        diff2 = hist[i3] - hist[i3+1]
        if diff1 * diff2 < 0:
            maxs += max(abs(diff1),abs(diff2))

#smooth = map[0:10,290:300]/1000
#plt.subplot(234)
#plot_hist(smooth)
#plot_grad(smooth)
#rough = map[320:350,420:450]/1000
#rough = map[320:325,420:425]
#print rough
#plt.subplot(231)
#plot_hist(rough)
#plot_grad(rough)
#sidehill = map[110:140,300:330]/1000
#sidehill = map[110:115,300:305]
#print sidehill
#plt.subplot(232)
#plot_hist(sidehill)
#plot_grad(sidehill)
#roughsidehill = map[300:330,200:230]/1000
#roughsidehill = map[300:305,200:205]
#print roughsidehill
#plt.subplot(233)
#plot_hist(roughsidehill)
#plot_grad(roughsidehill)
#plt.show()
#print np.mean(smooth),np.mean(rough),np.mean(sidehill)
#print np.std(smooth),np.std(rough),np.std(sidehill)
#print neighbDiffMean(rough),neighbDiffMean(sidehill),neighbDiffMean(roughsidehill)
#print countOfDiff(smooth),countOfDiff(rough),countOfDiff(sidehill),countOfDiff(roughsidehill)

#plt.show()
#fim = np.fft.fft2(map)
#print fim[0:10,0:10]

#plt.subplot(131)
#plt.imshow(rough,cmap=plt.cm.gray)
#plt.subplot(132)
#plt.imshow(sidehill,cmap=plt.cm.gray)
#plt.subplot(133)
#plt.imshow(roughsidehill,cmap=plt.cm.gray)
#plt.show()
'''