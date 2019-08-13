# AR Tracking Tools
A set of tools for tracking artags in ros.

## Tracking a multiresolution AR Bundle with defualt bundle tracking:
```
rosrun ar_tracking_tools findMarkerBundlesNoKinect 5.0 0.2 0.2 /image/topic /info/topic /camera/frame /path/to/bundle.xml
```

## Tracking a AR Bundle with RANSAC:
```
roslaunch ar_tracking_tools track_bundle_ransac.launch image_topic:=/image/topic info_topic:=/info/topic marker_size:=5.0 bundle_file:=/path/to/bundle.xml bundle_frame:=/marker_bundle
```
