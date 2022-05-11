"""
This module consists of list of topics name that is subscribed by various nodes
"""

# the topic that has compressed image from the camera (of type CompressedImage)
IMAGE_TOPIC = '/usb_cam/image_raw/compressed'

# the topic to control the movement (of type Twist)
CMD_VEL = 'cmd_vel'

# the topic that has the bounding boxes (of type BoundingBoxes)
BOUNDING_BOXES = 'darknet_ros/bounding_boxes'

STATE_TOPIC = 'master/state'

# contains message that has information about the bounding box (of type Trash)
TRASH_TOPIC = 'litter_picker/vision'
OBSTACLE = "litter_picker/obstacle"
SCAN = "/scan"