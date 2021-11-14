# Camera Projections using Eigen
A C++ class containing a number of camera projections and operations using the efficiency of Eigen.

For those starting with computer vision, understanding how these projections work and how points in the world map back onto a 2D plane can be quite tricky. This is something that there seems to be very few headers/libraries which include in their documentation or as exposed functions to the user.


I hope this comes in handy.

Note: there'll be many more additions to comments and general documentation when I get around to it. But essentially:

```cpp
  CameraCal cc(<insert camera calibration params>);
  CameraOps ops(cc);
  
  Eigen::Vector3d camera_point = ops.world_to_camera(point, extrinsics);
```


## Dependencies

- Eigen (>=v3.4.0)
