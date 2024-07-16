#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image


class PassthroughImagePreprocessor:
    """
    A dummy preprocessing node to make it so that lane detector nodes that want
    to worry about processing themselves can do so without needing to change the architecture,
    or the launch files.
    """
    raw_img_subscriber: rospy.Subscriber
    processed_img_publisher: rospy.Publisher

    # ------------------------------------------------
    # ------- Internal state-related functions -------
    # ------------------------------------------------
    def __init__(self):
        """
        Initializes an image-preprocessing node.

        Sets up subscribers, publishers, and dynamic reconfigure.
        """
        # Node architecture
        rospy.init_node("line_detection_preprocessor", anonymous=True)
        self.raw_img_subscriber = rospy.Subscriber(
            rospy.get_param("~img_in_topic"), Image, self._preprocess_image
        )
        # TODO: Add a subcscriber for live cropping
        self.processed_img_publisher = rospy.Publisher(
            rospy.get_param("~img_out_topic"), Image, queue_size=1
        )


    # -----------------------------------------------------
    # ----------- Image Preprocessing functions -----------
    # -----------------------------------------------------
    def _preprocess_image(self, ros_image: Image) -> None:
        """
        Passthrough the image.

        Parameters
        ----------
        ros_image: Image
            ROS image to be preprocessed.
        """
        self.processed_img_publisher.publish(ros_image)


if __name__ == "__main__":
    try:
        preprocessor = PassthroughImagePreprocessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
