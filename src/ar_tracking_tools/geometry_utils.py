#!/usr/bin/env python

import numpy as np
from tf.transformations import rotation_matrix
  
def foldPoints(pts, edge, reverse = False):
    num_pts = pts.shape[0]
    
    center_mat = np.eye(4)
    center_mat[:3,3] = -np.array(edge[0])
    if(reverse):
        angle = -edge[2]
    else:
        angle = edge[2]
    rot_mat = rotation_matrix(angle, np.array(edge[1]) - np.array(edge[0]))
    
    edge_mat = np.linalg.inv(center_mat).dot(rot_mat).dot(center_mat)
    folded_pts = edge_mat.dot(np.concatenate([pts.T, np.ones((1,num_pts))])).T[:,:3]
    
    return folded_pts

def getNormal(pts):
    demean = pts-np.mean(pts,axis=0)
    vn = np.linalg.svd(demean.T.dot(demean))[0][:,-1]
    vn /= np.linalg.norm(vn)
    return vn

def project2Plane(pt, vn, pt0):
    v = pt-pt0
    return pt - np.dot(v,vn)*vn

def planeIntersection(nv1, pt1, nv2, pt2):
    edge_axis = np.cross(nv1, nv2)
    edge_axis /= np.linalg.norm(edge_axis)
    cos_angle = np.dot(nv1, nv2)
    sin_angle = np.dot(np.cross(edge_axis, nv1), nv2)
    edge_angle = np.arctan2(sin_angle, cos_angle)

    proj_pt1 = project2Plane(pt1, nv2, pt2)
    proj_nv1 = project2Plane(pt1 + nv1, nv2, pt2)

    proj_v = proj_nv1 - proj_pt1
    proj_v /= np.linalg.norm(proj_v)

    t = (np.dot(nv1, pt1) - np.dot(nv1, proj_pt1)) / np.dot(nv1, proj_v)
    edge_center = proj_pt1 + t*proj_v
    
    return edge_axis, edge_center, edge_angle

def getEdges(point_sets, normals = None):
    box_center = np.mean(np.concatenate(point_sets), axis=0)
    
    centers = []
    for pts in point_sets:
        centers.append(np.mean(pts, axis = 0))

    if(normals is None):
        normals = []
        for pts, pts_center in zip(point_sets, centers):
            vn = getNormal(pts)
            if(np.dot(vn, pts_center - box_center) < 0):
                vn *= -1
            normals.append(vn)
    
    base_vn = normals[0]
    base_center = centers[0]
    edges = [None]
    for pts, vn, pts_center in zip(point_sets[1:], normals[1:], centers[1:]):
        edge_axis, edge_center, edge_angle = planeIntersection(base_vn, base_center, vn, pts_center)
        
        ts = np.dot(pts - edge_center, edge_axis)
        
        edge_start = edge_center + edge_axis*np.min(ts)
        edge_end = edge_center + edge_axis*np.max(ts)
        edges.append((edge_start, edge_end, -edge_angle))
    return edges