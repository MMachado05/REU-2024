# Arguments for different package nodes and the values they can legally take on

## preprocessor
* full - Gives you total contorl over the preprocessed image; cropping, white filtering,
fine-grained warp perspective, the works.
* easy_birdseye (default) - Like full, but warp perspective is just two sliders to squeeze the top
and bottom of the image.
* passthrough - No preprocessor; just passes the image as captured onto the lane detector.

## lane_detector
* dbscan (default) - A density-based clustering lane detector.
* kmeans - A K-means based lane detector.
* largest-contour - A simple contour-based + offset lane detector.
* birdseye - An image-split LSRL-based lane detector. *NOTE:* This lane detector only works
with the `passthrough` preprocessor.

## vehicle
* sim (default) - Simulates the vehicle in the simulator.
* actor1 - Runs the vehicle in the actor1 namespace.
* actor2 - Runs the vehicle in the actor2 namespace.
