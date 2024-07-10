#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# from dynamic_reconfigure.server import Server
# from lane_detectors_pkg.cfg import ...  # packageName.cfg

class TemplateLaneDetector:
    """
    This is a template file you can copy to create new lane-detecting nodes.

    You'll need to add new imports, message types, parameters, etc. as you see fit.

    Be sure to replace "template" whereever you see it!
    """

    preprocessed_img_subscriber: rospy.Subscriber
    center_offset_publisher: rospy.Publisher

    offset_message: Twist

    # ------------------------------------------------
    # ------- Internal state-related functions -------
    # ------------------------------------------------
    def __init__(self):
        """
        Initializes the node.
        """
        # Node architecture
        rospy.init_node("template_lane_detector", anonymous=True)

        self.preprocessed_img_subscriber = rospy.Subscriber(
            rospy.get_param("~img_in_topic"), Image, self._find_lane
        )
        self.center_offset_publisher = rospy.Publisher(
            rospy.get_param("~desired_twist_out_topic"), Twist, queue_size=1
        )
        # self.dyn_rcfg_srv = Server(
        #     ..., self._dynamic_reconfig_callback
        # )

        self.offset_message = Twist()

    # def _dynamic_reconfig_callback(self, config, _):
    #     """
    #     Callback function for dynamic reconfigure.
    #
    #     Parameters
    #     ----------
    #     config : DBScanLaneDetectorConfig
    #         The new dynamic reconfigure parameters.
    #     """
    #     return config

    # ---------------------------------------
    # ----------- Lane detection ------------
    # ---------------------------------------
    def _find_lane(self, ros_image: Image) -> None:
        """
        Detects a lane in the image and publishes the offset between the desired
        midpoint, and the current midpoint.

        The expected values to be published are angular x and y speeds within absolute
        values of 1. In the case of an exceptional situation, the z angular velocity
        will contain useful information:
        * 1: No lane lines were detected.
        * 2: Only one lane line was detected.
        * 3: There was some other exceptional error.

        Paramaters
        ----------
        ros_image : Image
            The preprocessed image to detect the lane in.
        """
        # TODO: Implement this!

        self.center_offset_publisher.publish(self.offset_message)


if __name__ == "__main__":
    try:
        detector = TemplateLaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
