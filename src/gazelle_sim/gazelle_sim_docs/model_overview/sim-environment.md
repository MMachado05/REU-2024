Playing Field Overview
------
The GazelleSim simulation environment is called the playing field.  The ground plain of the playing field is defined by an image.

Required Parameters - Playing Field Definition
------
* map_dir [string] - The base path to the playing field image.  Please note, that this parameter is typically defined in the ROS launch file.  This allows the GazelleSim simulation package to use relative path to key files.
* field_image_file [sting] - The playing field image file name
* pixels_per_meter [double] - Number pixels per meter for the playing field image
* x_center_img [int] - Location of X = 0 in the playing field image
* y_center_img [int] - Location of Y = 0 in the playing field image 
* img_view_scale [double] - Scaling of the viewed playing field image during simulation


Optional Parameters - Robot Information
------
* robot_list [list of strings] - the list of robots in the simulation environment
* Xr_init [double] - Initial robot X location
* Yr_init [double] - Initial robot Y location
* Theta_init [double] - Initial robot rotation relative to X-axis

Optional Parameter - Solver Time Step
------
* delta_t [double] - the fixed time step of the solver (default 0.05 sec).  Please note that GazelleSim will attempt to run the simulation at a fixed time step in real time using ROS rate and sleep functionality.  The targeted time step (Target) and the actual step (Act) is periodically displayed on the playing field.


| Sample Time Step  |
| ----------------- |
| ![](img/time_step.png) |



Optional Parameters - Obstructions
------
* rectlist - Two dimensional list of rectangle corners locations.  Each sublist contains a list of 4 (x,y) points for the obstruction in clockwise order.
```
rectlist: [ [ x1, y1, x2, y2, x3, y3, x4, y4 ],
            [ x1, y1, x2, y2, x3, y3, x4, y4 ],
			...
            [ x1, y1, x2, y2, x3, y3, x4, y4 ] ]
```
For example,
```
rectlist: [ [ 2.00, -0.75,  2.50, -0.75,  2.50, -1.25,  2.00, -1.25],
            [-0.25 , 2.75,  0.25,  2.75,  0.25,  2.25, -0.25,  2.25],
            [-2.00, -2.00, -1.75, -2.25, -2.00, -2.50, -2.25, -2.25] ]
```
* circlist - Two dimensional list of circular obstruction locations and radii.  Each sublist contains the center of the obstruction (xc, yc) and the obstruction radius.
```
circlist: [ [ x_1, y_1, radius_1 ],
            [ x_2, y_2, radius_2 ],
			...
            [ x_n, y_n, radius_n ] ]
```
For example,
```
circlist: [ [ -2.0, 2.0, 0.25 ],
            [  1.5, 1.0, 0.2  ] ]
```


Optional Parameters - GPS Waypoints
------
* gps_waypoints - Two dimensional list of gps waypoint locations.  Each sublist contains the latitude and longitude in decimal for format.  An identifier will be dislayed next to each waypoint.  The identifiers are assigned by the order of the entries in the GPs waypoint list.
```
gps_waypoints: [ [ lat0, lon0 ],
                 [ lat1, lon1 ],
				 ...
				 [ lat_n, lon_n ] ]
```

Next: [Robot Models](../model_overview/robot-models.md)
