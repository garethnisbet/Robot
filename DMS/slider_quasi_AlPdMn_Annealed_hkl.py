#!/usr/bin/env python
import inspect, os, sys, subprocess
from mpl_toolkits.mplot3d import Axes3D
dmspath = os.path.abspath(os.path.join(os.path.dirname( __file__ )))
sys.path.append(dmspath)
sys.path.append(dmspath+'/calcms/')

import matplotlib.pyplot as plt
# ~ plt.ion()
from numpy import linalg as LA
from calcms import ts_quasi as ts
from calcms import loader as do
import imageio.v2 as imageio
import numpy as np
import time
from scipy import ndimage
from scipy.optimize import minimize
from scipy.optimize import differential_evolution
from collections import OrderedDict
#import scisoftpy.io as do
import copy
#from pyswarm import pso
from time import strftime

############# Added by TY #############

TAU=(1.0+np.sqrt(5.0))/2.0
PI=np.pi

def fib(n):
    aa = [0,1]
    if n > 1:
        for ii in range(n-1):
            aa.append(aa[-2]+aa[-1])
    if n == 0:
        return [0]
    else:
        return aa

#######################################

datestr=strftime("%Y%m%d%H%M")
# count=ts.counter_read()
# ts.counter_set(count+1)
zoomval=1
# histrange=[0,1060]

colourlim=[0, 1000]
colmap='gray'
# colmap='spectral'
# colmap='spectral_r'
# colmap='spring'
# colmap='terrain'
# colmap='terrain_r'
# colmap='jet'
# bravais = 'tetragonalA'
# bravais = 'tetragonalB'
# bravais = 'tetragonalC'
# bravais = 'monoclinicA'
# bravais = 'monoclinicB'
# bravais = 'monoclinicC'
# bravais = 'triclinic'
# bravais = 'cubic'
# bravais = 'rhombohedral'
#bravais = 'orthorhombic'
bravais = 'icosahedral'
numsteps=1000
numsteps_interactive=300
sfthresh =[123, 128]

scan=scannum=708447 
scan=scannum=708458 # AlPdMn
scan=scannum=913232 # AlPdMn
scan=scannum=913112 # AlPdMn

scan=scannum=913232 # AlPdMn 
scan=scannum=913111 # AlPdMn Annealed
scan=scannum=913123 # AlPdMn Annealed Energy Scan
# scan=scannum=913232 # AlPdMn December 2021 Not Annealed No phason strain

#scan=scannum=708553 # threefold
# scan=scannum=708554 # fivefold AlCuRu

datapoint0=1
datapoint=3

imnum=datapoint+1

scatv=0 # leave alone for pilatus2M
save,fit = 0,0
detoptimize = 1
autoreflist=0

#===============================================================================
#                               Minimize Method
#===============================================================================
# OptMethod = ts.minimizers['Nelder-Mead']
OptMethod = ts.minimizers['Differential Evolution']
# OptMethod = ts.minimizers['CG']
# OptMethod = ts.minimizers['L_BFGS_B']
# OptMethod = ts.minimizers['Powell']
# strat=ts.DE_Strategy['best1bin']
strat=ts.DE_Strategy['best1exp']
#===============================================================================

intensity=1
threshold=0

masksigma=0.001
simsigma=4.5*zoomval
width=57*zoomval # roi width
# colourmap=[2, 150]array([[-5.096  , -3.1495 ,  0.     ],

cmap=ts.cmap()['hot']
lattice2=[6.458,6.458,6.458,90,90,90]
#lattice2=[6.5,6.5,6.5,90,90,90]

scanpath = '/home/ndf61257/MintSpace/i16extra/data/2021/mm29043-1/'

outpath=os.path.dirname(os.path.realpath(__file__))+'/Processing/'+datestr+'_'+str(imnum)+'_'+str(scan)+'/'

PV=[0,1]
d = do.load(scanpath + str(scannum) + '.dat')

met=d.metadata
lattice=[met.a,met.b,met.c,met.alpha1,met.alpha2,met.alpha3]
if hasattr(d,'psic'):
    psi=d.psic[datapoint]
else:
    psi=d.metadata.psi
psi = psi-180
psi = -180
# psi=3.143
# psi = 3
# psi=90
# hkl=[0,2.084,2.3906]

hmod,kmod,lmod = 1.,1.0,1.0
# hkl=[d.h[0]*hmod,d.k[0]*kmod,d.l[0]*lmod]
# hkl=[d.metadata.h*hmod,d.metadata.k*kmod,d.metadata.l*lmod]

# hkl=np.array([d.metadata.h*hmod,d.metadata.k*kmod,d.metadata.l*lmod])
# hkl = hkl*1.0137813564571074

hkl=np.array([d.metadata.h*hmod,d.metadata.k*kmod,d.metadata.l*lmod])
# hkl = np.array([2.31420248, 3.76644535 ,1.44405395])
# hkl = np.array([2.31381488, 3.77148411, 1.44792992])
# hkl = np.array([2.5866831,  3.58156163, 1.1378524])
# hkl = np.array([2.27931876, 3.70249186, 1.29579814])
hkl = np.array([2.27931876, 3.70249186, 1.29579814])
hkl = hkl*d.energy2[datapoint]/d.energy2[datapoint0]

# hkl = np.array([1.92885617, 3.98353231, 1.39568154])
# hkl = np.array([1.73597055, 3.58517908, 1.25611338])

# hkl = hkl*d.energy2[datapoint]/d.energy2[datapoint0]
# hkl = hkl*d.energy2[datapoint]/d.energy2[datapoint0]

# hkl = [4.7318,0.0284,-0.9638]
# hkl=np.roll(hkl,-1)#for wrongly indexed
# hkl=[4.816, 0, 0]
hklint=np.round(hkl)
# hkl = hkl*0.987

imtemplate=str(scannum)+'-pilatus2M-files/%05d.tif'
try:
    energy = d.energy2[datapoint]
except:
    try:
        energy = d.metadata.Energy
    except:
        try:
            energy = d.DCMenergy
        except:
            energy = d.metadata.en
# energy = energy/1.0137813564571074

# energy = d.Energy[datapoint]

# azir=[d.metadata['azih'],d.metadata['azik'],d.metadata['azil']]
# azir=[-1,0,0]
azir=[d.metadata['azih'],d.metadata['azik'],d.metadata['azil']]

# azir=[1.57475,2.548,0]
# azir = np.roll(azir,-1)

################################### Mask ####################################

im = imageio.imread(str(scanpath + str(imtemplate % imnum)))
#im = imageio.imread(str(scanpath + str(imtemplate % imnum)))
# im = np.fliplr(im)
# lowpass = ndimage.gaussian_filter(im,20,0)
# im=im+lowpass.min()
# imdata = im-lowpass

im=ndimage.zoom(im, zoomval, order=3)

imdata=np.copy(im)

py=(713-75)*zoomval # as read from untransformed image
px=(994-65)*zoomval # as read from untransformed image horizontal direction
py=(660)*zoomval # as read from untransformed image
px=(928)*zoomval # as read from untransformed image horizontal direction

px=(1018+179)*zoomval# datapoint 0
py=(796+58)*zoomval # datapoint 6

px=(1018+127)*zoomval# datapoint 0
py=(796+21)*zoomval # datapoint 6

# py=(1007)*zoomval # as read from untransformed image

# px , py = 839*zoomval,  (737-227)*zoomval

thb=ts.bragg(lattice,hkl,energy).th()[0]

thrange=[thb-27,thb+10]

psirange=[psi-180,psi+180]
detvects=np.matrix([[1,0,0],[0,0,1]])
hkllist=ts.pilkhlrange(lattice,hkl,energy,thrange[0],thrange[1]).hklscan(numsteps)

if autoreflist:
    mslist=[[np.NAN,np.NAN,np.NAN,np.NAN,np.NAN,np.NAN,np.NAN]]
    hkllistcorse=ts.pilkhlrange(lattice,hkl,energy,thrange[0],thrange[1]).hklscan(30)
    SF, reflist, lattice2 , structure, sfc = ts.loadcif(cif_file,energy)
    reflist = np.squeeze(reflist[np.where(SF > sfthresh),:],0)
    SF = SF[np.where(SF > sfthresh)]
    for hklval in range(len(hkllistcorse[:,0])):
        ms=ts.calcms(lattice,hkllistcorse[hklval,:],hklint,reflist,energy,azir)
        mslist=np.concatenate((mslist,ms.full()),0)
    mslist=ts.reducebypsirange(mslist,psirange)
    reflist=np.matrix(ts.uniquearray(mslist[:,0:3]))
else:


######################### Reflection List #########################
    """
    reflist = np.matrix([
            [-2.548000, 0.000000, 1.574750], # -1 -1 -2 -1  1  1 (5f)
            [0.000000, -1.574750, 2.548000], # -1  1 -1 -2 -1  1 (5f)
            [-1.574750, 2.548000, 0.000000], #  1 -1 -1  1  2  1 (5f)
            [2.548000, 0.000000, 1.574750],  #  1  2  1 -1 -1  1 (5f)
            [1.574750, 2.548000, 0.000000],  #  2  1  1  1  1  1 (5f)
            [-1.574750, -0.973250, 2.548000],# -1  0 -2 -2  0  1 (2f)
            [-2.548000, 1.574750, 0.973250], # -2 -1 -2 -1  0  0 (2f)
            [1.574750, -0.973250, 2.548000], #  0  2  0 -2 -1  1 (2f)
            [0.000000, 3.149500, 0.000000],  #  2  0  0  1  2  1 (2f)
            [2.548000, 1.574750, 0.973250],  #  2  2  1  0  0  1 (2f)
            
            #[-1.57475,  0.97325,  5.6975 ],  # 0  1 -3 -3  1  4 (2f)
            #[ 1.57475,  0.97325,  5.6975 ],  # 1  3 -1 -3  0  4 (2f)
            #[-2.548  ,  3.52125,  4.12275],  # 1  0 -3 -1  3  4 (2f)
            #[ 0.     ,  5.096  ,  3.1495 ], # ???
            #[ 2.548  ,  3.52125,  4.12275],  # 3  3  0 -1  1  4 (2F)
            
            [ 0.     ,  3.1495 ,  5.096  ],  # 2  2 -2 -2  2  4 (5f)
            
            ])
    reflist2=0
    """
    #########################
    #   Else's 6D indices   #
    #########################
    
    ref_6d=np.array([
        [-1,-1,-2,-1, 1, 1],# (5f)
        [-1, 1,-1,-2,-1, 1],# (5f)
        [ 1,-1,-1, 1, 2, 1],# (5f)
        [ 1, 2, 1,-1,-1, 1],# (5f)
        [ 2, 1, 1, 1, 1, 1],# (5f)
          
        [-1, 0,-2,-2, 0, 1],# (2f)
        [-2,-1,-2,-1, 0, 0],# (2f)
        [ 0, 2, 0,-2,-1, 1],# (2f)
  
        [ 2, 0, 0, 1, 2, 1],# (2f)
        [ 2, 2, 1, 0, 0, 1],# (2f)
        [0, -1,-2, 0, 2, 1],
         
        [ 0, 1,-3,-3, 1, 4],# (2f) [-1.57475,  0.97325,  5.6975 ],
        [ 1, 3,-1,-3, 0, 4],# (2f) [ 1.57475,  0.97325,  5.6975 ],
        [ 3, 3, 0,-1, 1, 4],# (2F) [ 2.548  ,  3.52125,  4.12275],
        [3, 1, -1, 0, 3, 4], 
        [ 1, 0,-3,-1, 3, 4],# (2f) [-2.548  ,  3.52125,  4.12275],
        ])
    ref_6d=np.array([[ 1, -1, -1,  1,  2,  1],
                       [ 1,  1,  2,  1, -1, -1],
                       [ 1, -1,  1,  2,  1, -1],
                       [ 1, -1,  0,  2,  2,  0],
                       [ 1,  0, -1,  0,  2,  2],
                       [ 1,  0,  2,  2,  0, -1],
                       [ 1,  2,  0, -1,  0,  2],
                       [ 1,  2,  2,  0, -1,  0],
                       [ 1,  2,  1, -1, -1,  1],
                       [ 1,  1, -1, -1,  1,  2]])

    # ref_6d=np.array([
        # [-1,-1,-2,-1, 1, 1],# (5f)
        # [-1, 1,-1,-2,-1, 1],# (5f)
        # [ 1,-1,-1, 1, 2, 1],# (5f)
        # [ 1, 2, 1,-1,-1, 1],# (5f)
        # [ 2, 1, 1, 1, 1, 1],# (5f)
        # [-1, 0,-2,-2, 0, 1],# (2f)
        # [-2,-1,-2,-1, 0, 0],# (2f)
        # [ 0, 2, 0,-2,-1, 1],# (2f)
        # [ 2, 0, 0, 1, 2, 1],# (2f)
        # [ 2, 2, 1, 0, 0, 1],# (2f)
        # [0, -1,-2, 0, 2, 1],
        # [ 0, 1,-3,-3, 1, 4],# (2f) [-1.57475,  0.97325,  5.6975 ],
        # [ 1, 3,-1,-3, 0, 4],# (2f) [ 1.57475,  0.97325,  5.6975 ],
        # [ 3, 3, 0,-1, 1, 4],# (2F) [ 2.548  ,  3.52125,  4.12275],
        # [3, 1, -1, 0, 3, 4], 
        # [ 1, 0,-3,-1, 3, 4],# (2f) [-2.548  ,  3.52125,  4.12275],
        # ])
       

    ref_6d=np.array([
        [-1,-1,-2,-1, 1, 1],# (5f)
        [-1, 1,-1,-2,-1, 1],# (5f)
        [ 1,-1,-1, 1, 2, 1],# (5f)
        [ 1, 2, 1,-1,-1, 1],# (5f)
        [ 2, 1, 1, 1, 1, 1],# (5f)
        [-1, 0,-2,-2, 0, 1],# (2f)
        # [-2,-1,-2,-1, 0, 0],# (2f)
        [ 0, 2, 0,-2,-1, 1],# (2f)
        [ 2, 0, 0, 1, 2, 1],# (2f)
        [ 2, 2, 1, 0, 0, 1],# (2f)
        [0, -1,-2, 0, 2, 1],
        [ 0, 1,-3,-3, 1, 4],# (2f) [-1.57475,  0.97325,  5.6975 ],
        [ 1, 3,-1,-3, 0, 4],# (2f) [ 1.57475,  0.97325,  5.6975 ],
        [ 3, 3, 0,-1, 1, 4],# (2F) [ 2.548  ,  3.52125,  4.12275],
        [3, 1, -1, 0, 3, 4], 
        # [ 1, 0,-3,-1, 3, 4],# (2f) [-2.548  ,  3.52125,  4.12275],

        ])
        
    # ref_6d=np.array([

        # [-1, 0,-2,-2, 0, 1],# (2f)    # T1
        # [ 3, 3, 0,-1, 1, 4],# (2F)    # T1 T5   [ 2.548  ,  3.52125,  4.12275],
        # [3, 1, -1, 0, 3, 4],          # T1 T2
        
        # [ 0, 2, 0,-2,-1, 1],# (2f)    # T2
        # [ 1, 0,-3,-1, 3, 4],# (2f)    # T2 T3   [-2.548  ,  3.52125,  4.12275],
        # [ 2, 0, 0, 1, 2, 1],# (2f)    # T4  
        # [ 2, 2, 1, 0, 0, 1],# (2f)    # T3
        # [0,1,-3,-3,1,4],
        # [0, -1,-2, 0, 2, 1],          # T5
        # [ 1, 3,-1,-3, 0, 4],# (2f)    # T4 T5   [ 1.57475,  0.97325,  5.6975 ],

        # ])
        
    mmm=np.matrix([
            [ 1., 0., 0., 0., 0., 0.],
            [ 0., 1., 0., 0., 0., 0.],
            [ 0., 0., 0., 0., 0., 1.],
            [ 0., 0., 0., 0., 1., 0.],
            [ 0., 0., 1., 0., 0., 0.],
            [ 0., 0., 0.,-1., 0., 0.],
            ])
    # print((mmm*ref_6d.T).T)
    #ref_6d=ts.hklgen_ico(3).v()

    # MEMO  2019.08.29 TY
    # Input reflection lists contains 6d indices in Elser's scheme, and they are transformed to Cahn and Gratius 6d indices.
    
    # tau = 34/21.
    # fib_list = fib(6)
    # tau = fib_list[-1]/fib_list[-2]
    # print(str(fib_list[-1])+' '+str(fib_list[-2])+' Tau = '+str(tau))
    # tau = 21/13.
    tau = 55/34.
    
    #tau = 0.5+0.5*5**0.5
    p6d=ts.Projection6dArrayApproximant(ref_6d,tau)
    #p6d=ts.Projection6dArrayApproximantTwofoldDist(ref_6d,tau,[0,5],1.01)
    # p6d=ts.Projection6dArray(ref_6d)
    reflist0=p6d.reflection_6d()
    reflist=reflist0[0]   # parallel component   HKL = h+h'*tau, k+k'*tau, l+l'*tau,
    reflist2=reflist0[1]  # perpendicular component 


# AlCuRuMtx = np.matrix([[-0.002452,0.007980,0.006628],[0.006366,-0.003911,-0.004627],[-0.002529,-0.004388,0.009355]])+np.identity(3)

# reflist = (AlCuRuMtx*reflist.T).T


###################################################################        

########### Added by TY ###########
# this contains a 3x3 phason matrix, mtrx2 at the bottom line : a11,a12,a13,a21,a22,a23,a31,a32,a33
# mtrx2 = np.array([[a11,a12,a13],[a21,a22,a23],[a31,a32,a33]])

                          
# initial_guess = np.array([d.metadata.a,d.metadata.a,d.metadata.a,
#                           90.000000,90.000000,90.000000,#
#                           -0.078233,0.015959,0.310379,
#                           2848.974010*zoomval,-1.545367,0.032768,-1.595531,
#                           energy,
#                           0,0,0,
#                           0,0,0,
#                           0,0,0,
#                           ])
# ##################################
# initial_guess = np.array([  6.45700000e+00,   6.45700000e+00,   6.45700000e+00,
#                          9.00000000e+01,   9.00000000e+01,   9.00000000e+01,
#                         -3.59056061e-01,   1.18756590e+00,  -2.18367557e-01,
#                          5383*zoomval,  -1.54536700e+00,   3.27680000e-02,
#                         -1.58685044e+00,   d.energy2[datapoint],
#                          0,0,0,
#                          0,0,0,
#                          0,0,0,
#                         ])
# initial_guess = np.array([  6.45700000e+00,  6.45700000e+00,  6.45700000e+00,  9.00000000e+01,
#                             9.00000000e+01,  9.00000000e+01, -3.59056061e-01,  3.17907780e-01,
#                            -2.18367557e-01,  5.38300000e+03*zoomval, -1.54536700e+00,  3.27680000e-02,
#                            -1.58685044e+00,  6.30000000e+00,  0.00000000e+00,  0.00000000e+00,
#                             0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
#                             0.00000000e+00,  0.00000000e+00,  0.00000000e+00])
#                             
# initial_guess = np.array([ 6.45700000e+00,  6.45700000e+00,  6.45700000e+00,  9.00000000e+01,
#                             9.00000000e+01,  9.00000000e+01, -1.18170563e+00,  7.40984703e-01,
#                             3.69239281e-01,  5.36633333e+03*zoomval, -1.54536700e+00,  3.27680000e-02,
#                            -1.58685044e+00,  d.energy2[datapoint],  0.00000000e+00,  0.00000000e+00,
#                             0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
#                             0.00000000e+00,  0.00000000e+00,  0.00000000e+00])
# 
# 
# initial_guess = np.array([  6.45700000e+00,  6.45700000e+00,  6.45700000e+00,  9.00000000e+01,
#                             9.00000000e+01,  9.00000000e+01, -1.25809452e+00,  6.64595814e-01,
#                             5.79308725e-01,  5.26008333e+03*zoomval, -1.54536700e+00,  3.27680000e-02,
#                            -1.58685044e+00,  d.energy2[datapoint],  0.00000000e+00,  0.00000000e+00,
#                             0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
#                             0.00000000e+00,  0.00000000e+00,  0.00000000e+00])
# initial_guess = np.array([6.459098,6.459098,6.459098,90.000000,90.000000,90.000000,
#                         0,0,0,5235.860222*zoomval,0.430257,0.022243,-4.142404,
#                         d.energy2[datapoint],-0.005163,-0.001391,0.000051,-0.008542,0.000987,0.000203,-0.006125,-0.001771,-0.000463,])
#                         
# 
# initial_guess = np.array([ 6.45909800e+00,  6.45909800e+00,  6.45909800e+00,  9.00000000e+01,
#                         9.00000000e+01,  9.00000000e+01, -1.90972222e-01,  0.0892,
#                        -7.63888889e-02,  5.23169356e+03*zoomval,  6.56736667e-02,  2.22430000e-02,
#                         5.45096000e-01,   d.energy2[datapoint], -5.16300000e-03, -1.39100000e-03,
#                         5.10000000e-05, -8.54200000e-03,  9.87000000e-04,  2.03000000e-04,
#                        -6.12500000e-03, -1.77100000e-03, -4.63000000e-04])
#                        
# initial_guess = np.array([6.457794,6.457794,6.457794,90.000000,90.000000,90.000000,-0.224245,0.072694,-0.079990,5232.108293*zoomval,-2.327585,0.006876,-1.542715,d.energy2[datapoint],-0.004994,-0.002060,-0.000144,-0.008499,0.004544,0.000186,-0.005938,-0.004015,-0.000614,])
# 
# initial_guess = np.array([ 6.45779400e+00,  6.45779400e+00,  6.45779400e+00,  9.00000000e+01,
#                         9.00000000e+01,  9.00000000e+01,  3.43020085e-02,  2.25471778e-01,
#                        -3.66448333e-01,  5.23210829e+03*zoomval, -2.32758500e+00,  6.87600000e-03,
#                        -1.54271500e+00,  d.energy2[datapoint], -4.99400000e-03, -2.06000000e-03,
#                        -1.44000000e-04, -8.49900000e-03,  4.54400000e-03,  1.86000000e-04,
#                        -5.93800000e-03, -4.01500000e-03, -6.14000000e-04]) # With wrong matrix
# 
# initial_guess = np.array([ 6.45779400e+00,  6.45779400e+00,  6.45779400e+00,  9.00000000e+01,
#                         9.00000000e+01,  9.00000000e+01,  3.43020085e-02,  2.25471778e-01,
#                        -3.66448333e-01,  5.23210829e+03*zoomval, -2.32758500e+00,  6.87600000e-03,
#                        -1.54271500e+00,  d.energy2[datapoint],
#                         -4.05972212e-03, -3.36498645e-03,  1.34879883e-04,
#                         -9.42775053e-03,  3.42111613e-03,  5.44494608e-05,
#                         -4.64600081e-03, -1.85548504e-03, -1.77444436e-04])
# 
# initial_guess = np.array([6.459452,6.459452,6.459452,90.000000,
#                         90.000000,90.000000,0.021401,0.230127,-0.381077,
#                         5230.649830*zoomval,-1.566182,0.004252,-1.599469,d.energy2[datapoint],
#                         -0.003372,0.001110,-0.001425,
#                         -0.008498,0.001937,0.002098,
#                         -0.004712,-0.001707,0.001820,]) # from Fit
#                         
# initial_guess = np.array([6.459452,6.459452,6.459452,90.000000,
#                         90.000000,90.000000,0.021401,0.230127,-0.381077,
#                         4651*zoomval,-1.566182,0.004252,-1.599469,d.energy2[datapoint],
#                         -0.003372,0.001110,-0.001425,
#                         -0.008498,0.001937,0.002098,
#                         -0.004712,-0.001707,0.001820,]) # from Fit
#                         
#                         
# initial_guess = np.array([ 6.46666701e+00,  6.46666701e+00,  6.46666701e+00,  9.00000000e+01,
#         9.00000000e+01,  9.00000000e+01, -4.42304344e+00,  5.15555714e-02,
#        -3.47631510e+00,  4.65100000e+03*zoomval, -1.56618200e+00,  4.25200000e-03,
#        -1.59946900e+00,  6.29900000e+00, 0,0,0,0,0,0,0,0,0])
# 
# 
# initial_guess = np.array([ 6.45803000e+00,  6.45803000e+00,  6.45803000e+00,  9.00000000e+01,
#                             9.00000000e+01,  9.00000000e+01,0,0,0,  4.83498269e+03*zoomval, -1.80375180e-02, -2.88600289e-01,
#                            -8.76623376e+00,  energy,  0.00000000e+00,  0.00000000e+00,
#                             0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
#                             0.00000000e+00,  0.00000000e+00,  0.00000000e+00])
#                             
# initial_guess = np.array([ 6.45803000e+00,  6.45803000e+00,  6.45803000e+00,  9.00000000e+01,
#                             9.00000000e+01,  9.00000000e+01,  0.00000000e+00,  0.00000000e+00,
#                             0,  4.81230828e+03*zoomval,  2.05411081e-06,  6.87987532e-01,
#                             -1.10432795e-05,  energy,  0.00000000e+00,  0.00000000e+00,
#                             0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
#                             0.00000000e+00,  0.00000000e+00,  0.00000000e+00])
#                             
# #December Experiment
# 
# 
# initial_guess = np.array([ 6.45803000e+00,  6.45803000e+00,  6.45803000e+00,  9.00000000e+01,
#                         9.00000000e+01,  9.00000000e+01, -8.52713178e-01,  4.26356589e-01,
#                         2.13178295e-01,  4.81230828e+03*zoomval,  2.05411081e-06,  6.87987532e-01,
#                        -1.10432795e-05,  energy,  0.00000000e+00,  0.00000000e+00,
#                         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
#                         0.00000000e+00,  0.00000000e+00,  0.00000000e+00])
# 
# initial_guess = np.array([ 6.45803000e+00,  6.45803000e+00,  6.45803000e+00,  9.00000000e+01,
#                         9.00000000e+01,  9.00000000e+01, -8.52713178e-01,  4.26356589e-01,
#                         2.13178295e-01, 4.81239056e+03*zoomval,  1.96224542e-04,  6.91358987e-01,
#                        -1.41314340e-02,  energy+0.00004667e+00, 0.00000000e+00,  0.00000000e+00,
#                         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
#                         0.00000000e+00,  0.00000000e+00,  0.00000000e+00])
# 
# initial_guess = np.array([ 6.45803000e+00,  6.45803000e+00,  6.45803000e+00,  9.00000000e+01,
#                         9.00000000e+01,  9.00000000e+01, 0,0,0, 4.81239056e+03*zoomval,  1.96224542e-04,  6.91358987e-01,
#                        -1.41314340e-02,  energy+0.00004667e+00, 0.00000000e+00,  0.00000000e+00,
#                         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
#                         0.00000000e+00,  0.00000000e+00,  0.00000000e+00])
# 
# initial_guess = np.array([ 6.45803000e+00,  6.45803000e+00,  6.45803000e+00,  9.00000000e+01,
#                         9.00000000e+01,  9.00000000e+01,  0.00000000e+00,  0.00000000e+00,
#                        -3.09108527e-01,  4.81239056e+03*zoomval,  1.96224542e-04,  6.91358987e-01,
#                        -1.41314340e-02,  energy+0.00004667e+00,  0.00000000e+00,  0.00000000e+00,
#                         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
#                         0.00000000e+00,  0.00000000e+00,  0.00000000e+00])
#                         
# initial_guess = np.array([ 6.45803000e+00,  6.45803000e+00,  6.45803000e+00,  9.00000000e+01,
#                             9.00000000e+01,  9.00000000e+01, -5.95120034e-03, -4.04845686e-04,
#                            -3.08960049e-01,  4.81239056e+03*zoomval,  1.96224542e-04,  6.91358987e-01,
#                            -1.41314340e-02,  energy+0.00004667e+00,  0.00000000e+00,  0.00000000e+00,
#                             0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
#                             0.00000000e+00,  0.00000000e+00,  0.00000000e+00])
                            
initial_guess = np.array([ d.metadata.a,d.metadata.a,d.metadata.a,  9.00000000e+01,
                        9.00000000e+01,  9.00000000e+01,  2.42688472e-02, -3.06155951e-02,
                       -2.95816671e-01,  9.62478112e+03/2*zoomval,  1.96000000e-04,  6.91359000e-01,
                       -1.41310000e-02,  energy+0.00004667e+00, -2.10236187e-05,  1.89949298e-05,
                        6.52693154e-05, -1.31361116e-05,  8.20651352e-05, -4.29715307e-05,
                       -3.34503046e-04,  1.33821580e-05,  1.65281198e-04]) # fixed a datapoint 5 Starting from zero matrix

initial_guess = np.array([ d.metadata.a,d.metadata.a,d.metadata.a,  9.00000000e+01,
                        9.00000000e+01,  9.00000000e+01,  2.42688472e-02, -3.06155951e-02,
                       -2.95816671e-01,  9.62478112e+03/2*zoomval,  1.96000000e-04,  6.91359000e-01,
                       -1.41310000e-02-3.22514,  energy+0.00004667e+00, -2.10236187e-05,  1.89949298e-05,
                        6.52693154e-05, -1.31361116e-05,  8.20651352e-05, -4.29715307e-05,
                       -3.34503046e-04,  1.33821580e-05,  1.65281198e-04]) # fixed a datapoint 5 Starting from zero matrix

initial_guess = np.array([ 6.46136300e+00,  6.46136300e+00,  6.46136300e+00,  9.00000000e+01,
                        9.00000000e+01,  9.00000000e+01, -2.30548741e+00,  9.43761240e-01,
                       -2.14691025e+00,  4.81239056e+03*zoomval,  1.96000000e-04,  6.91359000e-01,
                       -7.55289069e+00,  energy+0.00004667,  0.00000000e+00,  0.00000000e+00,
                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00])
                        
initial_guess = np.array([ 6.46136300e+00,  6.46136300e+00,  6.46136300e+00,  9.00000000e+01,
                        9.00000000e+01,  9.00000000e+01, 0,0,0,  4.81239056e+03*zoomval,  1.96000000e-04,  6.91359000e-01,
                       -7.55289069e+00,  energy+0.00004667,  0.00000000e+00,  0.00000000e+00,
                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00])
                        
initial_guess = np.array([ 6.46136300e+00,  6.46136300e+00,  6.46136300e+00,  9.00000000e+01,
        9.00000000e+01,  9.00000000e+01, -1.27906977e-01,  2.13178295e-01,
       -4.19961240e+00,  4.81297196e+03*zoomval, -8.81586946e-01,  1.91228923e+00,
       -7.74668914e+00,  energy+0.00004667,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00])

initial_guess = np.array([6.461188,6.461188,6.461188,90.000000,90.000000,90.000000,0.024767,-0.030694,-0.293951,9624.781120/2*zoomval,0.000196,0.691359,-0.014131,energy+0.00004667,
                        0,0,0,
                        0,0,0,
                        0,0,0])


initial_guess = np.array([ 6.46118800e+00,  6.46118800e+00,  6.46118800e+00,  9.00000000e+01,
                        9.00000000e+01,  9.00000000e+01, -2.25624075e+00, -3.06940000e-02,
                       -2.03135410e+00,  4.81239056e+03*zoomval,  1.96000000e-04,  6.91359000e-01,
                       -1.04126278e+00,  energy+0.00004667,  0.00000000e+00,  0.00000000e+00,
                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00])
initial_guess = np.array([6.461363,6.461363,6.461363,90.000000,90.000000,90.000000,0.018156,-0.026200,-0.302918,4812.390560*zoomval,0.000196,0.691359,-0.014131,energy+0.00004667e+00,0,0,0,0,0,0,0,0,0])


initial_guess = np.array([ 6.46136300e+00,  6.46136300e+00,  6.46136300e+00,  9.00000000e+01,
                        9.00000000e+01,  9.00000000e+01, 0,0,0,
                        4.81239056e+03*zoomval,  1.96000000e-04,  6.91359000e-01,
                       -1.41310000e-02,  energy+0.00004667e+00,  0.00000000e+00,  0.00000000e+00,
                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00])

initial_guess = np.array([ 6.46136300e+00,  6.46136300e+00,  6.46136300e+00,  9.00000000e+01,
        9.00000000e+01,  9.00000000e+01, -2.24903101e+00,  0.00000000e+00,
       -3.73062016e-01,  4.81239056e+03*zoomval,  1.96000000e-04,  6.91359000e-01,
       -1.41310000e-02,  energy+0.00004667,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00])

initial_guess = np.array([6.460662,6.460662,6.460662,
                            90.000000,90.000000,90.000000,
                            -2.226417,0.005990,-0.447239,9653.725020/2*zoomval,
                            0.228572,0.667038,-2.097034,energy+0.00004667,
                            0.001483,0.001069,0.000011,
                            0.000083,-0.000347,-0.003123,
                            0.000138,-0.001041,0.001335])
                            
initial_guess = np.array([6.460662,6.460662,6.460662,
                            90.000000,90.000000,90.000000,
                            -2.226417,0.005990,-0.447239,9653.725020/2*zoomval,
                            0.228572,0.667038,-2.097034,energy+0.00004667,
                            -2.56574605e-03,  7.13075251e-04,  1.30870050e-03,
                             2.26169068e-03,  1.13153879e-03, -1.81378093e-03,
                             2.49373021e-03, -4.98114603e-05,  1.29613796e-03]) #from tripfit
                             
                             
initial_guess = np.array([6.460662,6.460662,6.460662,
                            90.000000,90.000000,90.000000,
                            -2.226417,0.005990,-0.447239,9653.725020/2*zoomval,
                            0.228572,0.667038,-2.097034,energy+0.00004667,
                            0,0,0,
                             0,0,0,
                             0,0,0]) #from tripfit
# initial_guess = np.array([ 6.46136300e+00,  6.46136300e+00,  6.46136300e+00,  9.00000000e+01,
                        # 9.00000000e+01,  9.00000000e+01, -2.32680524e+00,  1.64356589e-02,
                       # -2.49623426e-01,  4.81239056e+03*zoomval,  1.96000000e-04,  6.91359000e-01,
                       # -1.41310000e-02,  energy+0.00004667e+00,  
                       # 0.002294,0.000411,-0.000805,
                       # 0.000631,-0.000502,-0.002490,
                       # -0.000260,-0.002496,0.001995])
# initial_guess = np.array([6.460662,6.460662,6.460662,90.000000,90.000000,90.000000,-2.17441860e+00,0,0,9653.725020/2*zoomval,0.228572,0.667038,-2.097034,energy+0.00004667,0.001088,0.000914,0.000234,0.000447,-0.000290,-0.003021,-0.000152,-0.000719,0.001488,])
# initial_guess = np.array([6.460662,6.460662,6.460662,90.000000,90.000000,90.000000,-2.17441860e+00,0,0,9653.725020/2*zoomval,0.228572,0.667038,-2.097034,energy+0.00004667,0,0,0,0,0,0,0,0,0])


# initial_guess = np.array([6.461363,6.461363,6.461363,90.000000,90.000000,90.000000,0.018156,-0.026200,-0.302918,4812.390560*zoomval,0.000196,0.691359,-0.014131,energy+0.00004667e+00,-0.001264,0.001902,0.000831,0.001480,0.000033,0.000041,0.000041,-0.000782,-0.000297,])
# initial_guess = np.array([6.461363,6.461363,6.461363,90.000000,90.000000,90.000000,0.018156,-0.026200,-0.302918,4812.390560*zoomval,0.000196,0.691359,-0.014131,energy+0.00004667e+00,0,0,0,0,0,0,0,0,0])
initial_guess = np.array([6.461030,6.461030,6.461030,90.000000,90.000000,90.000000,-2.171235,-0.006615,-0.013657,14480.587530/3.,0.228572,0.667038,-2.097034,energy+0.00004667e+00,0.001283,0.000737,0.000503,0.000526,-0.000982,-0.002763,-0.000311,-0.001417,0.002340])
initial_guess = np.array([6.461030,6.461030,6.461030,90.000000,90.000000,90.000000,-2.171235,-0.006615,-0.013657,14437/3.,0.228572,0.667038,-2.097034,energy+0.00004667e+00,0.001283,0.000737,0.000503,0.000526,-0.000982,-0.002763,-0.000311,-0.001417,0.002340])





initial_guess = np.array([6.460662,6.460662,6.460662,
                            90.000000,90.000000,90.000000,
                            -2.226417,0.005990,-0.447239,9653.725020/2*zoomval,
                            0.228572,0.667038,-2.097034,energy+0.00004667,
                            -2.56574605e-03,  7.13075251e-04,  1.30870050e-03,
                             2.26169068e-03,  1.13153879e-03, -1.81378093e-03,
                             2.49373021e-03, -4.98114603e-05,  1.29613796e-03]) #from tripfit

initial_guess = np.array([6.461053,6.461053,6.461053,90.000000,90.000000,90.000000,-2.171374,-0.006984,-0.013387,14480.587530/3*zoomval,0.228572,0.667038,-2.097034,energy+0.00004667,0.001228,0.000730,0.000491,0.000507,-0.000951,-0.002741,-0.000441,-0.001405,0.002354,])
# initial_guess = np.array([6.461030,6.461030,6.461030,90.000000,90.000000,90.000000,-2.171235,-0.006615,-0.013657,14480.587530/3.,0.228572,0.667038,-2.097034,energy+0.00004667e+00,0,0,0,0,0,0,0,0,0])

#------------------------------------------------------------------------------ 
# fromtriple = [6.41833263, 6.41833263, 6.41833263,  90.0260035,  90.0260035,  90.0260035]
#fromtriple = [-2.56574605e-03,  7.13075251e-04,  1.30870050e-03,  2.26169068e-03, 1.13153879e-03, -1.81378093e-03,  2.49373021e-03, -4.98114603e-05, 1.29613796e-03])

# initial_guess = np.concatenate((fromtriple,initial_guess[6:]),0)
# fromtriple = [  6.44598292,   6.40373916,   6.40445628,  90.0260035 , 90.0561134 ,  90.0551165 ]
ig = initial_guess

# reflist = np.concatenate((ts.circlehkl(hkl,azir,lattice,-psi,energy,0.01),
                          # ts.circlehkl(hkl,azir,lattice,-psi,energy,4.9),
                          # ts.circlehkl(hkl,azir,lattice,-psi,energy,10)),0)

# reflist = ts.circlehkl(hkl,azir,ig[:6],-psi,energy,1)
detdistancepx,rotx,roty,rotz,energy = ig[9], ig[10],ig[11],ig[12],ig[13]

a11,a12,a13,a21,a22,a23,a31,a32,a33 = ig[14], ig[15],ig[16],ig[17],ig[18],ig[19],ig[20],ig[21],ig[22]

##### GUI #############
fig = plt.figure(figsize=(12, 5),dpi=130)
#spacing = np.linspace(0.32,0.05,14)
spacing = np.linspace(0.9,0.1,23)
thickness = 0.02
ax_a = plt.axes([0.6,spacing[0], 0.3, thickness], facecolor='white')
ax_h = plt.axes([0.6, spacing[1], 0.3, thickness], facecolor='white')
ax_k = plt.axes([0.6,spacing[2], 0.3, thickness], facecolor='white')
ax_l = plt.axes([0.6, spacing[3], 0.3, thickness], facecolor='white')
ax_beta = plt.axes([0.6, spacing[4], 0.3, thickness], facecolor='white')
ax_gamma = plt.axes([0.6, spacing[5], 0.3, thickness], facecolor='white')
ax_psi = plt.axes([0.6, spacing[6], 0.3, thickness], facecolor='white')
ax_th = plt.axes([0.6, spacing[7], 0.3, thickness], facecolor='white')
ax_chi = plt.axes([0.6, spacing[8], 0.3, thickness], facecolor='white')
ax_detdistancepx = plt.axes([0.6, spacing[9], 0.3, thickness], facecolor='white')
ax_rotx = plt.axes([0.6,spacing[10], 0.3, thickness], facecolor='white')
ax_roty = plt.axes([0.6, spacing[11], 0.3, thickness], facecolor='white')
ax_rotz = plt.axes([0.6, spacing[12], 0.3, thickness], facecolor='white')
ax_en = plt.axes([0.6, spacing[13], 0.3, thickness], facecolor='white')

########### Added by TY ###########
# 3x3 phason matrix
ax_a11 = plt.axes([0.6, spacing[14], 0.3, thickness], facecolor='white')
ax_a12 = plt.axes([0.6, spacing[15], 0.3, thickness], facecolor='white')
ax_a13 = plt.axes([0.6, spacing[16], 0.3, thickness], facecolor='white')
ax_a21 = plt.axes([0.6, spacing[17], 0.3, thickness], facecolor='white')
ax_a22 = plt.axes([0.6, spacing[18], 0.3, thickness], facecolor='white')
ax_a23 = plt.axes([0.6, spacing[19], 0.3, thickness], facecolor='white')
ax_a31 = plt.axes([0.6, spacing[20], 0.3, thickness], facecolor='white')
ax_a32 = plt.axes([0.6, spacing[21], 0.3, thickness], facecolor='white')
ax_a33 = plt.axes([0.6, spacing[22], 0.3, thickness], facecolor='white')
##################################

#imbox = plt.axes([0.15, 0.38,0.6, 0.6], facecolor='gray')
imbox = plt.axes([-0.1,0.1,0.85, 0.85], facecolor='gray')

#dms = ts.dmscalc(reflist,hkllist,hklint,1,psirange,100,hkl,detvects,imdata,simsigma,azir,psi,px,py,scatv,detdistancepx,rotx,roty,rotz,energy) #init arguments

########### Added by TY ###########
# reflist2 ... this contains perpendicular components
# mtrx2    ... this is 3x3 phason matrix
mtrx2 = [ig[14],ig[15],ig[16],ig[17],ig[18],ig[19],ig[20],ig[21],ig[22]]

dms = ts.dmscalc_ico(reflist,hkllist,hklint,1,psirange,100,hkl,detvects,imdata,simsigma,azir,psi,px,py,scatv,detdistancepx,rotx,roty,rotz,energy,reflist2,mtrx2)


##################################

opt,simim,dmsindex,dataim2=dms.full(ig)
imoverlay = np.copy(imdata)
imdata_max = imdata.max()
imoverlay[dmsindex]=imdata_max
detdistancepx,rotx,roty,rotz,en=ig[8],ig[9],ig[10],ig[11],ig[12]
p = plt.imshow(imoverlay,cmap=colmap,clim=(colourlim[0], colourlim[1]))

sldr_a = plt.Slider(ax_a, 'a', ig[0]-0.2, ig[0]+0.2,valinit=ig[0],valfmt = '%0.5f',color='gray')
sldr_h = plt.Slider(ax_h, 'h', hkl[0]-1.2, hkl[0]+1.2,valinit=hkl[0],valfmt = '%0.5f',color='gray')
sldr_k = plt.Slider(ax_k, 'k', hkl[1]-1.2, hkl[1]+1.2,valinit=hkl[1],valfmt = '%0.5f',color='gray')
sldr_l = plt.Slider(ax_l, 'l', hkl[2]-1.5, hkl[2]+1.5,valinit=hkl[2],valfmt = '%0.5f',color='gray')
sldr_beta = plt.Slider(ax_beta, 'beta', ig[4]-0.1, ig[4]+0.1,valinit=ig[4],valfmt = '%0.5f',color='gray')
sldr_gamma = plt.Slider(ax_gamma, 'gamma', ig[5]-0.1, ig[5]+0.1,valinit=ig[5],valfmt = '%0.5f',color='gray')
sldr_psi = plt.Slider(ax_psi, 'psi', ig[6]-5.5, ig[6]+5.5,valinit=ig[6],valfmt = '%0.5f',color='gray')
sldr_th = plt.Slider(ax_th, 'th', ig[7]-5.5, ig[7]+5.5,valinit=ig[7],valfmt = '%0.5f',color='gray')
sldr_chi = plt.Slider(ax_chi, 'chi', ig[8]-5.5, ig[8]+5.5,valinit=ig[8],valfmt = '%0.5f',color='gray')
sldr_detdistancepx = plt.Slider(ax_detdistancepx, 'dist', ig[9]-300, ig[9]+300,valinit=ig[9],valfmt = '%0.5f',color='gray')
sldr_rotx = plt.Slider(ax_rotx, 'rotx', ig[10]-5, ig[10]+5,valinit=ig[10],valfmt = '%0.5f',color='gray')
sldr_roty = plt.Slider(ax_roty, 'roty', ig[11]-5, ig[11]+5,valinit=ig[11],valfmt = '%0.5f',color='gray')
sldr_rotz = plt.Slider(ax_rotz, 'rotz', ig[12]-10, ig[12]+10,valinit=ig[12],valfmt = '%0.5f',color='gray')
sldr_en = plt.Slider(ax_en, 'en', ig[13]-0.5, ig[13]+0.5,valinit=ig[13],valfmt = '%0.5f',color='gray')

########### Added by TY ###########
sldr_a11 = plt.Slider(ax_a11, 'a11', ig[14]-0.05, ig[14]+0.05,valinit=ig[14],valfmt = '%0.05f',color='gray')
sldr_a12 = plt.Slider(ax_a12, 'a12', ig[15]-0.05, ig[15]+0.05,valinit=ig[15],valfmt = '%0.05f',color='gray')
sldr_a13 = plt.Slider(ax_a13, 'a13', ig[16]-0.05, ig[16]+0.05,valinit=ig[16],valfmt = '%0.05f',color='gray')
sldr_a21 = plt.Slider(ax_a21, 'a21', ig[17]-0.05, ig[17]+0.05,valinit=ig[17],valfmt = '%0.05f',color='gray')
sldr_a22 = plt.Slider(ax_a22, 'a22', ig[18]-0.05, ig[18]+0.05,valinit=ig[18],valfmt = '%0.05f',color='gray')
sldr_a23 = plt.Slider(ax_a23, 'a23', ig[19]-0.05, ig[19]+0.05,valinit=ig[19],valfmt = '%0.05f',color='gray')
sldr_a31 = plt.Slider(ax_a31, 'a31', ig[20]-0.05, ig[20]+0.05,valinit=ig[20],valfmt = '%0.05f',color='gray')
sldr_a32 = plt.Slider(ax_a32, 'a32', ig[21]-0.05, ig[21]+0.05,valinit=ig[21],valfmt = '%0.05f',color='gray')
sldr_a33 = plt.Slider(ax_a33, 'a33', ig[22]-0.05, ig[22]+0.05,valinit=ig[22],valfmt = '%0.05f',color='gray')
##################################

iii=1
_last_hkl = hkl.copy()
_last_update_time = 0.0

def update(val):
        global _last_hkl, _last_update_time
        now = time.time()
        if now - _last_update_time < 0.1:
            return
        _last_update_time = now
        a = sldr_a.val
        h = sldr_h.val
        k = sldr_k.val
        l = sldr_l.val
        beta = sldr_beta.val
        gamma = sldr_gamma.val
        th = sldr_th.val
        psi = sldr_psi.val
        chi = sldr_chi.val
        detdistancepx = sldr_detdistancepx.val
        rotx = sldr_rotx.val
        roty = sldr_roty.val
        rotz = sldr_rotz.val
        en = sldr_en.val
########### Added by TY ###########
        # 3x3 phason matrix
        a11 = sldr_a11.val
        a12 = sldr_a12.val
        a13 = sldr_a13.val
        a21 = sldr_a21.val
        a22 = sldr_a22.val
        a23 = sldr_a23.val
        a31 = sldr_a31.val
        a32 = sldr_a32.val
        a33 = sldr_a33.val
##################################

        ig[0],ig[1],ig[2],ig[3],ig[4],ig[5],ig[6],ig[7],ig[8],ig[9],ig[10],ig[11],ig[12],ig[13],ig[14],ig[15],ig[16],ig[17],ig[18],ig[19],ig[20],ig[21],ig[22] = a,a,a,90,90,90,psi,th,chi,detdistancepx,rotx,roty,rotz,en,a11,a12,a13,a21,a22,a23,a31,a32,a33
            #print 'updated, phason matrix = ',a11,a12,a13,a21,a22,a23,a31,a32,a33
        hklnew = np.array([h,k,l])
        if not np.allclose(hklnew, _last_hkl):
            dms.sethkl(hklnew)
            hkllistnew=ts.pilkhlrange(lattice,hklnew,energy,thrange[0],thrange[1]).hklscan(numsteps_interactive)
            dms.sethkllist(hkllistnew)
            _last_hkl[:] = hklnew
        opt,simim,dmsindex,dataim2=dms.full(ig)

########### Added by TY ###########
        
##################################

        imoverlay = np.copy(imdata)
        imoverlay[dmsindex]=imdata_max
#         imscale=int(im*a*b*c*alpha*beta*gamma*th*psi)
#         imscale=im*a*b*c*alpha*beta*gamma*th*psi
        p.set_data(imoverlay)
#         misc.imsave('/home/ndf61257/TRMS/PMN/Processing/SliderB/%i.tif' % count,imoverlay)
#         imgno = round(sldr_b.val)
#         p2.set_data(vol[:,:,imgno-1])
sldr_a.on_changed(update)
sldr_h.on_changed(update)
sldr_k.on_changed(update)
sldr_l.on_changed(update)
sldr_beta.on_changed(update)
sldr_gamma.on_changed(update)
sldr_th.on_changed(update)
sldr_psi.on_changed(update)
sldr_chi.on_changed(update)
sldr_detdistancepx.on_changed(update)
sldr_rotx.on_changed(update)
sldr_roty.on_changed(update)
sldr_rotz.on_changed(update)
sldr_en.on_changed(update)
########### Added by TY ###########
sldr_a11.on_changed(update)
sldr_a12.on_changed(update)
sldr_a13.on_changed(update)
sldr_a21.on_changed(update)
sldr_a22.on_changed(update)
sldr_a23.on_changed(update)
sldr_a31.on_changed(update)
sldr_a32.on_changed(update)
sldr_a33.on_changed(update)
##################################

def overlay2rgb(sigma,ig,im, colourlim):
    im_8bit = im*1
    im_8bit[np.where(im_8bit > colourlim[1])] = colourlim[1]
    im_8bit = (im_8bit*255/im_8bit.max()).astype(int)
    opt,simim,dms_index,dataim2=dms.full(ig)
    empty = np.zeros(im.shape)
    dmspoints = np.copy(empty)
    dmspoints[dms_index]=255
    dmspointsconv=ndimage.gaussian_filter(dmspoints, sigma,order=0)
    dmspointsconv=dmspointsconv*(255/dmspointsconv.max())
    print("save using misc.imsave('Processing/Images/imrgb.png',imrgb)")
    return ts.im2rgb(im_8bit,dmspointsconv,dmspoints)
        
# im10deg = overlay2rgb(0.1,ig,im,colourlim)
# imageio.imsave('Processing/Images/im_'+datestr+'.png',im10deg)

# dnp.plot.image(im,name='im')
# imfilter=im
# 
# imfilter[np.where(imfilter<1600)]=0
# # imfilter[np.where(im>=1600)]=1
# dnp.plot.image(imfilter,name='im')
# ims,k1=ts.multiscalefilter2(imfilter,5,20,1,100,1,'line')
# imfilter=np.sum(ims[:,:,range(0,ims.shape[2],1)],2)
# dnp.plot.image(imfilter,name='im2')
# n=1
# energy = 5
# darwin_w=2.12*ts.constants['re']*((ts.constants['keV2A']/energy)/n+1)**2*((20.02*3543.8)/(np.pi*sin(ts.bragg(lattice,[1,1,1],energy).th()*np.pi/180)))

#f = open("Processing/reflist.txt", "w")
#f.write('reflist = np.matrix([')
#[f.write("[%f, %f, %f],\n" % (reflist[ii,0],reflist[ii,1],reflist[ii,2])) for ii in range(reflist.shape[0])]
#f.write('])')
#f.close()
plt.show()
