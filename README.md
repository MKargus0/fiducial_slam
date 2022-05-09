# fiducial_slam

Simultaneous localization and mapping using fidusial markers as features.


## Example 1: All fiducials are located in the same plane
https://user-images.githubusercontent.com/48154519/167446009-db369941-99e0-4af0-b77b-b602e1cfabc1.mp4


## Example 2: Fiducial markers are located with different position and orientation in space.
https://user-images.githubusercontent.com/48154519/167446310-779020d2-61e8-4586-8611-16c1474d6982.mp4


## Usage of multiple cameras
### Multiple camera source can be used for pode estimation

![Screenshot from 2021-05-19 21-50-59](https://user-images.githubusercontent.com/48154519/167449729-ecb1cff0-330f-4097-a97b-d9f1f3c73dc2.png)
### Multi-camera navigation can only be used if the field of view of the cameras does not overlap!!!

## Try samples


### Before start edit configs:
#### 1) config/cameraConfigs/<your_camera>.yaml (add intrinsic params, pose in body frame)
#### 2) config/fiducialBoards/<your_map>.yaml (add basis fidusial marker add size)
#### 3) config/fiducialDetectorConfigs/<your_dict>.yaml (use of marker dictionaris setup)
#### 4) config/navigation_system_config.yaml (add path to other files common system setup)


### Build

1) ```mkdir build```

2) ```cd build```

3) ```cmake ..```

4) ```make -j4```
