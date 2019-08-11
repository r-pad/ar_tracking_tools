#!/usr/bin/env python

import numpy as np

def makeMarkerGrid(rows, cols, marker_size, margin = 0, 
                   corner = (0, 0, 0),
                   width = 0, height = 0, depth = 0):
    if(width <= 0 and height <= 0 and depth <= 0):
        raise ValueError('Atleast one dimension must be non-zero')
    
    x0, y0, z0 = corner
    center_margin = margin + marker_size/2.
    
    centers = []
    if(depth == 0):
        for y in np.linspace(y0 - center_margin, y0 - height + center_margin, rows):
            for x in np.linspace(x0 + center_margin, x0 + width - center_margin, cols):
                centers.append((x,y,z0))
    elif(height == 0):
        for z in np.linspace(z0 - center_margin, z0 - depth + center_margin, rows):
            for x in np.linspace(x0 + center_margin, x0 + width - center_margin, cols):
                centers.append((x,y0,z))
    elif(width == 0):
        for z in np.linspace(z0 - center_margin, z0 - depth + center_margin, rows):
            for y in np.linspace(y0 - center_margin, y0 - height + center_margin, cols):
                centers.append((x0,y,z))
    else:
        raise ValueError('Atleast on dimension (width, height, depth) must equal zero')
    
    return np.array(centers)

def multiResolutionMaskUR(n):
    return np.concatenate([np.tile(np.arange(n),n) + 2*n*np.repeat(np.arange(n),n) + n, 
                           np.tile(np.arange(n),n) + 2*n*np.repeat(np.arange(n),n) + 2*n*n])

def multiResolutionMaskUL(n):
    return np.concatenate([np.tile(np.arange(n),n) + 2*n*np.repeat(np.arange(n),n), 
                           np.tile(np.arange(n),n) + 2*n*np.repeat(np.arange(n),n) + 2*n*n + n])


def makeMultiResolutionGrid(width, spacing = 1, margin = 0, cols_ul = 1, cols_ur = 2):

    marker_size_ul = (width - (2*cols_ul-1)*spacing - 2*margin)/(2.*cols_ul)
    pts_ul = makeMarkerGrid(2*cols_ul, 2*cols_ul, 
                            marker_size_ul,
                            width = width, height = width)
    pts_ul = pts_ul[multiResolutionMaskUL(cols_ul)]

    marker_size_ur = (width - (2*cols_ur-1)*spacing- 2*margin)/(2.*cols_ur)
    pts_ur = makeMarkerGrid(2*cols_ur, 2*cols_ur, 
                               marker_size_ur,
                               width = width, height = width)
    pts_ur = pts_ur[multiResolutionMaskUR(cols_ur)]
    
    
    marker_sizes = np.concatenate([np.repeat(marker_size_ul, 2*(cols_ul**2)), 
                                   np.repeat(marker_size_ur, 2*(cols_ur**2))])
    return np.concatenate([pts_ul, pts_ur]), marker_sizes