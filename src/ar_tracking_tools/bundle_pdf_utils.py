#!/usr/bin/env python

import numpy as np
from fpdf import FPDF
import os
from PIL import Image

from ar_tracking_tools.geometry_utils import getEdges, foldPoints

IN2CM = 2.54
CM2PT = 72./IN2CM

def addMarker(pdf, index, position, size = 5.0, text_size = None, coords = None):
    x, y = position
    y *= -1
    os.system('rosrun ar_track_alvar createMarker {0:d} -s {1:}'.format(index, size))
    filename = 'MarkerData_{0:d}.png'.format(index)
    pdf.image(filename, x-size/2., y-size/2., size, size)
    os.remove(filename)
    
    if(text_size is not None):
        pdf.set_font('Courier', '', text_size*CM2PT)
        if(coords is not None):
            pdf.text(x-size/2., y+size/2. + text_size,
                'Alvar {0:d} {1:}cm ({2:})'.format(index, size, ', '.join(np.round(coords, 2).astype(str))))
        else:
            pdf.cell(x-size/2., y+size/2. + text_size,
                'Alvar {0:d} {1:}cm ({2:},{3:})'.format(index, size, x, y))
        
def addOrientedKey(pdf, position, size = 2.0, text_size = None, coords = None):
    x, y = position
    y *= -1
    pdf.line(x-size/2., y, x+size/2., y)
    pdf.line(x, y-size/2., x, y+size/2.)
    pdf.ellipse(x-size/4., y-size/4., size/2., size/2.)
    pdf.line(x, y-size/2.,
             x+size/16., y-.375*size)
    pdf.line(x, y-size/2.,
             x-size/16., y-.375*size)
    
    if(text_size is not None):
        pdf.set_font('Courier', '', text_size*CM2PT)
        pdf.set_xy(x, y + size/2. - text_size)
        if(coords is not None):
            pdf.text(x, y + size/2. - text_size, '({0:})'.format(', '.join(np.round(coords, 2).astype(str))))
        else:
            pdf.text(x, y + size/2. - text_size, '({0:},{1:})'.format(x, y))

def isOnSheet(pt, marker_size, page_idxs, printable_size, overlap):
    xm,ym = pt-marker_size/2.
    xp,yp = pt+marker_size/2.
    j,k = page_idxs
    w,h = printable_size - overlap
    left = xm >= j*w and xm <= (j+1)*w + overlap
    rigth = xp >= j*w and xp <= (j+1)*w + overlap
    upper = yp <= -k*h and yp >= -(k+1)*h - overlap
    lower = yp <= -k*h and yp >= -(k+1)*h - overlap
    ul = upper and left
    ur = upper and rigth
    ll = lower and left
    lr = lower and rigth
    
    return ul or ur or ll or lr

def makeBundlePdf(point_sets, marker_size, 
                  edges = None, 
                  start_index = 0,
                  paper_size = None, 
                  paper_orientation = 'P',
                  text_size = None, 
                  margin = 1., 
                  overlap = 2.,
                  rosette_positions = None, 
                  rosette_size = 1.):
    """
    Create a bundle pdf from set of points, seperated by face. 
    First set should be the central face
    """
    # Calculate edges, if not given
    if(edges is None):
        edges = getEdges(point_sets)
    
    # Flattens all marker centers
    flat_pts = []
    pts = []
    for p, e in zip(point_sets, edges):
        if(e is None):
            unfolded_pts = p
        else:
            unfolded_pts = foldPoints(p, e)
        assert np.all(np.abs(unfolded_pts[:,2]) < 1e-9), 'Unfolded z value nonzero for edge {}'.format(e)

        flat_pts.extend(unfolded_pts)
        pts.extend(p)
    flat_pts = np.stack(flat_pts)[:,:2]
    

    # Flattens all edge end points
    flat_edges = []
    for e in edges:
        if(e is not None):
            flat_e = foldPoints(np.stack([e[0], e[1]]), e)
            flat_edges.append(flat_e)

   
    if(rosette_positions is None):
        rosette_pts = []
        flat_rosette_pts = []
    elif(np.array(rosette_positions).size <= 3):
        rosette_pts = [np.array(rosette_positions)]
        flat_rosette_pts = np.expand_dims(np.array(rosette_positions)[:2], 0).astype(float)
    else:
        rosette_pts = []
        flat_rosette_pts = []
        for p, e in zip(rosette_positions, edges):
            if(e is None):
                unfolded_pts = p
            else:
                unfolded_pts = foldPoints(p, e)
            assert np.all(np.abs(unfolded_pts[:,2]) < 1e-9), 'Unfolded z value nonzero rosette on edge {}'.format(e)

            flat_rosette_pts.extend(unfolded_pts)
            rosette_pts.extend(p)

            flat_rosette_pts = np.stack(flat_rosette_pts)[:,:2]


    if(type(rosette_size) not in [np.ndarray, list]):
        rosette_size = np.repeat(rosette_size, len(flat_rosette_pts))

    if(type(marker_size) not in [np.ndarray, list]):
        marker_size = np.repeat(marker_size, len(flat_pts))


    # Find upper left corner of all markers
    min_x = np.min(flat_pts[:,0])
    max_y = np.max(flat_pts[:,1])
    max_marker_size = np.max(marker_size)
    tl_corner = np.array([min_x-max_marker_size/2., 
                          max_y+max_marker_size/2.])
    flat_pts = flat_pts.copy()
    flat_pts[:,0] -= tl_corner[0]
    flat_pts[:,1] -= tl_corner[1]
  
    if(len(flat_rosette_pts) > 0):
        flat_rosette_pts = flat_rosette_pts.copy()
        flat_rosette_pts[:,0] -= tl_corner[0]
        flat_rosette_pts[:,1] -= tl_corner[1]

    # If paper size is not defined, set to required size to fit all markers
    if(paper_size is None):
        overlap = 0
        paper_size = ( np.max(flat_pts[:,0]) + 2*margin + max_marker_size/2., 
                      -np.min(flat_pts[:,1]) + 2*margin + max_marker_size/2.)

    # Determine the number of sheets required
    printable_size = (np.array(paper_size) - 2*margin)
    sheet_coords = np.abs(flat_pts) / (printable_size - overlap)
    overlap_norm = overlap / (printable_size - overlap)
    num_sheets = np.ceil(np.max(sheet_coords, axis=0) - overlap_norm).astype(int)

    pdf = FPDF(orientation = paper_orientation, unit = 'cm', format=paper_size)
    if(text_size is not None):
        pdf.set_font('Courier', '', text_size*CM2PT)
        
    for j in range(num_sheets[0]):
        for k in range(num_sheets[1]):
            
            pdf.add_page()
            
            # Add alignment markings if necessary 
            if(j > 0):
                mark_loc = margin
                pdf.line(mark_loc, 0, mark_loc, margin)
                pdf.line(mark_loc, paper_size[1], mark_loc, paper_size[1]-margin)
            if(k > 0):
                mark_loc = margin
                pdf.line(0, mark_loc, margin, mark_loc)
                pdf.line(paper_size[0], mark_loc, paper_size[0]-margin, mark_loc)
            
            if(j < num_sheets[0]-1):
                mark_loc = paper_size[0]-margin-overlap
                pdf.line(mark_loc, 0, mark_loc, margin)
                pdf.line(mark_loc, paper_size[1], mark_loc, paper_size[1]-margin)
            if(k < num_sheets[1]-1):
                mark_loc = paper_size[1]-margin-overlap
                pdf.line(0, mark_loc, margin, mark_loc)
                pdf.line(paper_size[0], mark_loc, paper_size[0]-margin, mark_loc)
            
            if(text_size is not None and np.any(num_sheets>1)):
                pdf.text(margin/2, margin/2, '{0:}'.format((j,k)))
            
            paper_corner = np.array((j,-k))*(printable_size - overlap) - np.array([margin,-margin])

            for e in flat_edges:
                e_srt = e[0][:2] - tl_corner - paper_corner
                e_end = e[1][:2] - tl_corner - paper_corner
                pdf.dashed_line(e_srt[0], -e_srt[1], e_end[0], -e_end[1])
 
            for idx, (pt2, pt3, sz) in enumerate(zip(flat_rosette_pts, rosette_pts, rosette_size)):
                if(isOnSheet(pt2, sz, (j,k), printable_size, overlap)):
                    addOrientedKey(pdf, pt2 - paper_corner,
                              sz, text_size = text_size, coords = pt3)
           
            for idx, (pt2, pt3, sz) in enumerate(zip(flat_pts, pts, marker_size)):
                if(isOnSheet(pt2, sz, (j,k), printable_size, overlap)):
                    addMarker(pdf, start_index + idx, pt2 - paper_corner, 
                              sz, text_size = text_size, coords = pt3)
    return pdf
