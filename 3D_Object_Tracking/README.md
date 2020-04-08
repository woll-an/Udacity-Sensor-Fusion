# 3D Object Tracking

## FP.1 Match 3D Objects

1. loop through the matched keypoint pairs
2. select boxes in frames which contain the respective keypoint from the match
3. count occurences for each potential match of boxes
4. update best matches map if a box match appears more often than current stored box combination

## FP.2 Compute Lidar-based TTC

I compute the medians of the Lidar Points in X direction for the current (``d1``) and previous frame (``d0``). The time to collision with the constant velocity model can then be computed with ``TTC = d1 * delta_t / (d0 - d1)`` where delta_t is ``1/frame_rate``.

## FP.3 Associate Keypoint Correspondences with Bounding Boxes

## FP.4 Compute Camera-based TTC

## FP.5 Performance Evaluation 1

## FP.6 Performance Evaluation 2