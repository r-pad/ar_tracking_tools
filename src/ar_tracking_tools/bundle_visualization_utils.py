#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ar_tracking_tools.bundle_xml_utils import parseBundleXml

def visualizeBundleXML(xml_filename, ax = None, show = True):

    if(ax is None):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
    
    corners, centers, interior = parseBundleXml(xml_filename)

    corner_colors = [(1,0,0), (0,0,1), (0,1,0), (1,1,0)] 
    for tag_id, corner_pts in corners.items():
        corner_pts = np.array(corner_pts)
        ax.plot(corner_pts[[0,1,2,3,0],0], corner_pts[[0,1,2,3,0],1], corner_pts[[0,1,2,3,0],2], 
                label='Tag {}'.format(tag_id))
        center = np.mean(corner_pts, axis=0)
        ax.text(center[0],center[1],center[2], '{}'.format(tag_id), size=10, zorder=1, color='k')
        for pt, c in zip(corner_pts, corner_colors):
            ax.scatter(pt[0], pt[1], pt[2], c=c)

    ax.set_aspect('equal')
    ax.view_init(90, -90)
    
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    if(show == True):
        plt.show()
    
    return ax
