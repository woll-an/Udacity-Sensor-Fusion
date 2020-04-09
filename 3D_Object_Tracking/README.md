# 3D Object Tracking

## FP.1 Match 3D Objects

I implemented the following algorithm:

1. loop through the matched keypoint pairs
2. select boxes in frames which contain the respective keypoint from the match
3. count occurences for each potential match of boxes
4. update best matches map if a box match appears more often than current stored box combination

## FP.2 Compute Lidar-based TTC

I compute the medians of the Lidar Points in X direction for the current (``d1``) and previous frame (``d0``). The time to collision with the constant velocity model can then be computed with ``TTC = delta_t * d1 / (d0 - d1)`` where delta_t is ``1/frame_rate``.

## FP.3 Associate Keypoint Correspondences with Bounding Boxes

I loop through all matches and add those to the boxes keypoint matches, where the current keypoint is inside the bounding box.

## FP.4 Compute Camera-based TTC

I used implementation from the previous lecture about TTC. Compute for all keypoint combinations the distance ratio, take the median and compute TTC with ``TTC = -delta_t / (1 - medianDistRatio)``

## FP.5 Performance Evaluation 1

I try to avoid the influence of outliers by using the median for the X value of the lidarPoints in the current and previous frame. Nevertheless, sometimes the Lidar-based TTC is a little bit off. As the points of the lidar do not measure the distance to the exact same points on the vehicle in two concurrent frames, the computation might be influenced by variations in the depth in x direction of the car in front.

## FP.6 Performance Evaluation 2

The following tables show the Lidar and camera based TTCs for the three detector / descriptor combinations, which were chosen as a result of the performance evaluation of the midterm project.

### FAST ORB
|TTC Lidar [s] | TTC Camera [s] |
|--------|---------|
| 12.5156 | 11.7971 |
| 15.7465 | 19.7123 |
| 11.9844 | 13.5472 |
| 13.1241 | 12.2073 |
| 11.1746 | 13.3607 |
| 12.8086 | 12.7284 |
| 8.95978 | 11.7264 |
| 9.59863 | 10.835 |
| 8.52157 | 10.4391 |
| 9.51552 | 11.4283 |
| 9.61241 | 9.11715 |

### FAST BRIEF
| TTC Lidar [s] | TTC Camera [s] |
|--------|---------|
| 12.5156 | 11.6107 |
| 15.7465 | 15.2011 |
| 11.9844 | 13.6193 |
| 13.1241 | 12.6099 |
| 11.1746 | 12.1008 |
| 12.8086 | 12.2522 |
| 8.95978 | 11.4625 |
| 9.59863 | 10.9244 |
| 8.52157 | 11.179 |
| 9.51552 | 11.9352 |
| 9.61241 | 8.2058 |

### FAST BRISK
|TTC Lidar [s] | TTC Camera [s] |
|--------|---------|
| 12.5156 | 12.5 |
| 15.7465 | 22.7441 |
| 11.9844 | 12.5256 |
| 13.1241 | 12.0122 |
| 11.1746 | 14.0994 |
| 12.8086 | 12.0299 |
| 8.95978 | 12.5605 |
| 9.59863 | 11.4982 |
| 8.52157 | 11.8 |
| 9.51552 | 12.4833 |
| 9.61241 | 9.79683 |

Overall the computations of the three detector / descriptor combinations are quite reasonable. Other detectors result in much worse results, as can be seen in the next table for HARRIS ORB, where the camera based TTC is up to two times as high as the lidar based one and gets even negative and infinite for two frames.

### HARRIS ORB
|TTC Lidar [s] | TTC Camera [s] |
|--------|---------|
| 12.5156 | 10.9082 |
| 15.7465 | 34.7543 |
| 13.1241 | 17.6204 |
| 11.1746 | 20.5862 |
| 12.8086 | 11.4377 |
| 8.95978 | 11.6948 |
| 9.59863 | 5.85828 |
| 8.52157 | -13.6263 |
| 9.51552 | 6.71705 |
| 9.61241 | 12.5848 |
| 8.3988 | -inf |