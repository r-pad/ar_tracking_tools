# AR Tracking Tools
A set of tools for tracking artags in ros.

## Tracking a AR Bundle:
```
roslaunch object_pose_utils image_folder_track_bundle.launch image_topic:=/image/topic info_topic:=/info/topic marker_size:=5.0 bundle_file:=/path/to/bundle.xml bundle_frame:=/marker_bundle
```
