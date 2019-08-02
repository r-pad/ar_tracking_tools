# AR Tracking Tools
A set of tools for tracking artags in ros.

## Tracking a AR Bundle with defualt bundle tracking:
```
rosrun ar_track_alvar findMarkerBundlesNoKinect 5.0 0.2 0.2 /image/topic info_topic:=/info/topic marker_bundle /path/to/bundle.xml
```

## Tracking a AR Bundle with RANSAC:
```
roslaunch object_pose_utils image_folder_track_bundle.launch image_topic:=/image/topic info_topic:=/info/topic marker_size:=5.0 bundle_file:=/path/to/bundle.xml bundle_frame:=/marker_bundle
```
