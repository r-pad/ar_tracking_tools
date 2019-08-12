#!/usr/bin/env python

import numpy as np
import xml.etree.ElementTree as ET
from ar_tracking_tools.geometry_utils import foldPoints, getEdges

alvar_marker_config_flat = \
'''
    <marker index="{0:d}" status="1">
        <corner x="{1:}" y="{3:}" z="0.0" />
        <corner x="{2:}" y="{3:}" z="0.0" />
        <corner x="{2:}" y="{4:}" z="0.0" />
        <corner x="{1:}" y="{4:}" z="0.0" />
    </marker>
'''

alvar_marker_config = \
'''
    <marker index="{0:d}" status="1">
        <corner x="{1:}" y="{2:}" z="{3:}" />
        <corner x="{4:}" y="{5:}" z="{6:}" />
        <corner x="{7:}" y="{8:}" z="{9:}" />
        <corner x="{10:}" y="{11:}" z="{12:}" />
    </marker>
'''

def makeMarkerConfigFlat(index, position, marker_size):
    marker_config_str = alvar_marker_config_flat.format(index, 
        position[0], position[0]+marker_size,
        position[1], position[1]+marker_size)
    return marker_config_str                                    

def makeMarkerConfig(index, trans_mat, marker_size):
    corners = np.array([[-marker_size/2.0,-marker_size/2.0, 0.0, 1.0], 
                        [ marker_size/2.0,-marker_size/2.0, 0.0, 1.0],
                        [ marker_size/2.0, marker_size/2.0, 0.0, 1.0],
                        [-marker_size/2.0, marker_size/2.0, 0.0, 1.0]]).dot(trans_mat.T)[:,:3]
      
    marker_config_str = alvar_marker_config.format(index, *corners.flatten())
    return marker_config_str

def parseBundleXml(filename):
    tree = ET.parse(filename)
    root = tree.getroot()
    centers = {}
    corners = {}
    min_pts = []
    max_pts = []
    for marker in root: 
        idx = int(marker.get('index')) 
        corner_pts = [] 
        for corner in marker: 
            corner_pts.append([float(corner.get('x'))/100.0, 
                               float(corner.get('y'))/100.0, 
                               float(corner.get('z'))/100.0]) 
        corners[idx] = corner_pts
        centers[idx] = np.mean(corner_pts, axis=0) 
        min_pts.append(np.min(corner_pts, axis=0)) 
        max_pts.append(np.max(corner_pts, axis=0)) 
        interior = (np.max(min_pts, axis=0), np.min(max_pts, axis=0))
    return corners, centers, interior


marker_corners = np.array([[-1/2.,-1/2., 0],
                           [1/2.,-1/2., 0],
                           [1/2.,1/2., 0],
                           [-1/2.,1/2., 0]])

def makeBundleXml(point_sets, marker_size, edges = None, start_index = 0):
    # Calculate edges, if not given
    if(edges is None):
        edges = getEdges(point_sets)
    
    config_str = ''
    num_points = 0            
    centers = []
    for pts, edge in zip(point_sets, edges):
        if(edge is None):
            unfolded_pts = pts
        else:
            unfolded_pts = foldPoints(pts, edge)
    
        centers.extend(unfolded_pts)

    if(type(marker_size) not in [np.ndarray, list]):
        marker_size = np.repeat(marker_size, len(centers))
    
    for index, (pt, sz) in enumerate(zip(centers, marker_size)):
        corners = sz*marker_corners + pt
        if(edge is not None):
            corners = foldPoints(corners, edge, reverse=True)
        config_str += alvar_marker_config.format(index + start_index, *corners.flatten())
        num_points += 1

    full_config_str = '<?xml version="1.0" encoding="UTF-8" standalone="no" ?>\n' + \
        '<multimarker markers="{:d}">'.format(num_points) + config_str + '</multimarker>'
    
    return full_config_str