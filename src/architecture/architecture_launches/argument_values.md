# Arguments for different package nodes and the values they can legally take on

## preprocessor
* full - gives you total contorl over the preprocessed image; cropping, white filtering,
fine-grained warp perspective, the works.
* easy_birdseye (default) - like full, but warp perspective is just two sliders to squeeze the top
and bottom of the image.
* passthrough - no preprocessor; just passes the image as captured onto the lane detector.

## lane_detector
* dbscan (default) - a density-based clustering lane detector.
* kmeans - a K-means based lane detector.
* largest-contour - a simple contour-based + offset lane detector.
* birdseye (NOT IMPLEMENTED) - an image-split LSRL-based lane detector.

## vehicle
* sim (default) - simulates the vehicle in the simulator.
* actor1 - runs the vehicle in the actor1 namespace.
* actor2 - runs the vehicle in the actor2 namespace.
