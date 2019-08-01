#!/usr/bin/env python

alvar_marker_config = \
'''
    <marker index="{0:d}" status="1">
        <corner x="{1:}" y="{3:}" z="0.0" />
        <corner x="{2:}" y="{3:}" z="0.0" />
        <corner x="{2:}" y="{4:}" z="0.0" />
        <corner x="{1:}" y="{4:}" z="0.0" />
    </marker>
'''

def makeMarkerConfig(index, position, marker_size):
    marker_config_str = alvar_marker_config.format(index, 
        position[0], position[0]+marker_size,
        position[1], position[1]+marker_size)
    return marker_config_str                                    

def main():
    import os
    from argparse import ArgumentParser
    
    parser = ArgumentParser()
    parser.add_argument('--output_path', type=str, default=None,
        help='Path to save configuration')
    parser.add_argument('--grid_width', type=int, default=3,
        help='Number of marker columns')
    parser.add_argument('--grid_height', type=int, default=6,
        help='Number of marker rows')
    parser.add_argument('--grid_square_size', type=float, default=7.0,
        help='Space between markers in cm')
    parser.add_argument('--marker_size', type=float, default=5.0,
        help='Size of markers in cm')
    args = parser.parse_args()

    if(args.output_path is None):
        args.output_path = os.path.realpath(os.path.curdir)
    if(os.path.isdir(args.output_path)):
        args.output_path = os.path.join(args.output_path, 
            '{:}x{:}_grid_{:}cm_marker_{:}cm_spacing.xml'.format(
                args.grid_width, args.grid_height, args.marker_size, args.grid_square_size))

    config_str = '<?xml version="1.0" encoding="UTF-8" standalone="no" ?>\n' + \
        '<multimarker markers="{:d}">'.format(args.grid_width*args.grid_height)

    index = 0
    for j in range(args.grid_height):
        for k in range(args.grid_width):
            config_str += makeMarkerConfig(index, 
                (k*args.grid_square_size, j*args.grid_square_size),
                args.marker_size)
            index += 1
    config_str += '</multimarker>'
    with open(args.output_path, 'w') as f:
        f.write(config_str)

if __name__=='__main__':
    main()

