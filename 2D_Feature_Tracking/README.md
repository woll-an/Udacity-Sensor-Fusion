# 2D Feature Tracking

## Usage
after building the binary with ``mkdir build && cd build``, ``cmake ..`` and ``make``, use ``./2D_feature_tracking <detectorType> <decriptorType>``

## MP.1
Before inserting a new DataFrame, I check if the size of the vector containing the Dataframes already has the maximum size, i.e. ``dataBufferSize``. If this is the case, I remove the first element before I insert a new one.

## MP.2
Analogue to the implementation of the Shi Tomasi detector, I created functions for ``detKeypointsHarris`` and ``detKeypointsModern``. For Harris detection I used the implementation from a previous chapter including the removal of overlapping keypoints. For modern detectors I use the default detectors from openCV in an ``if - else`` structure and call ``detector->detect`` on the image. 

## MP.3
I check for every keypoint ``kp`` with ``vehicleRect.contains(kp.pt)``, if a keypoint is inside the rectangle of the car. I store all keypoints in an intermediate vector and move them in the end to the keypoints vector.

## MP.4
I create the default descriptors from openCV in an ``if - else`` structure in ``descKeypoints`` and call ``extractor->compute`` on the image and the keypoints.

## MP.5
In ``matchDescriptors`` I set ``matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED)`` as a matcher if ``MAT_FLANN`` is selected.

## MP.6
In ``matchDescriptors`` I select the k nearest neighbors as a matches if ``SEL_KNN`` is selected.

## MP.7

Data for keypoints on vehicle in ``performance.csv`` listed by detector. FAST and BRISK find most keypoints

## MP.8

Data for matched keypoints for every detector/descriptor combination in ``performance.csv``.

## MP.9

Data for keypoint detection and matchin in ``performance.csv``.
As we want to run our feature detecton on a car with real time constraints, we aim for the detector/decriptor combination with the shortest computing time which detects a good amount of keypoints on the car in front of us. The best detector is therefore FAST, which gets the best result combined either with BRIEF, BRISK or ORB descriptor.