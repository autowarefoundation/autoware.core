# autoware_object_recognition_utils

## Overview

This package contains a library of common functions that are useful across the perception module and planning module.

## Design

### Conversion

Ensuring accurate and efficient converting between DetectedObject and TrackedObject types.

### Geometry

It provides specialized implementations for each object type (e.g., DetectedObject, TrackedObject, and PredictedObject) to extract the pose information.

### Matching

It provides utility functions for calculating geometrical metrics, such as 2D IoU (Intersection over Union), GIoU (Generalized IoU), Precision, and Recall for objects. It also provides helper functions for computing areas of intersections, unions, and convex hulls of polygon

### Object Classification

Designed for processing and classifying detected objects, it implements the following functionalities:

- Handling of vehicle category checks
- Conversion between string class names and numerical labels
- Probability-based classification selection
- String representation of object labels

### Predicted Path Utils

Providing utility functions for handling predicted paths of objects. It includes the following functionalities:

- calcInterpolatedPose: Calculates an interpolated pose from a predicted path based on a given time.
- resamplePredictedPath (version 1): Resamples a predicted path according to a specified time vector, optionally using spline interpolation for smoother results.
- resamplePredictedPath (version 2): Resamples a predicted path at regular time intervals, including the terminal point, with optional spline interpolation.

## Usage

include all-in-one header files if multiple functionalities are needed:

```cpp
#include <autoware_object_recognition_utils/object_recognition_utils.hpp>
```

include specific header files if only a subset of functionalities is needed:

```cpp
#include <autoware_object_recognition_utils/object_classifier.hpp>
```
