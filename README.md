## Blob Algorithm (blob_follow package description)
### In Progress Files:
  * None
### Files to launch
#### blob.launch -> blob.py -> blob (node)
  Best lane following algorithm
  
  In dynamic reconfigure:
  * enable_drive AND enable_forward
  * drive_speed to change linear speed
  * blob_mult to adjust angular velocity (if needed)

#### test_image_demo.launch -> test_image_demo.py -> test_image_demo (node)
  Works worse than previous one
  
  In dynamic reconfigure:
  * enable_drive
  * drive_speed to change linear speed

### Change description
  Made Paul's blob follow algorihm work on gazelle sim
  * Added blob_follow package, that uses Paul's code from lane following
  * Copied gazelle_sim package
  
  Paul's code used python scripts from lane_follow_blob folder
  
  I copied lane_follow_blob into lane_helpers directory and added a few lines to lane_detection.py script

  Code started to work better after this

  To test default Nicolas Paul's code, change this:
        
      from lane_helpers.vec import Vec
      from lane_helpers.lane_centering import center_lane
      from lane_helpers.lane_detection import find_lanes, compute_lines
      from lane_helpers.utils import rows, cols
  
  to this:
  
      from lane_follow_blob.vec import Vec
      from lane_follow_blob.lane_centering import center_lane
      from lane_follow_blob.lane_detection import find_lanes, compute_lines
      from lane_follow_blob.utils import rows, cols
    
