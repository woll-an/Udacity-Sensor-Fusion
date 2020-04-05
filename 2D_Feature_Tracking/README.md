# Usage
after building the binary with ``mkdir build && cd build``, ``cmake ..`` and ``make``, use ``./2D_feature_tracking <detectorType> <decriptorType>``

# MP.1 - MP.6
Implemented in ``MidTermProkect_Camera_student.cpp`` and ``matching2D_Student.cpp``.

# MP.7 - MP.9

Data in ``performance.csv``.

## Best detector/decriptor combination
As we want to run our feature detecton on a car with real time constraints, we aim for the detector/decriptor combination with the shortest computing time which detects a good amount of keypoints on the car in front of us. The best detector is therefore FAST, which gets the best result combined either with BRIEF, BRISK or ORB descriptor.