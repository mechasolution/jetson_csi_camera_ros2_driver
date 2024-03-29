import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def gstreamer_pipeline(
    sensor_id=0,
    width=400,
    height=300,
    framerate=10,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            width,
            height,
            framerate,
            flip_method,
            width,
            height,
        )
    )


class CameraDriverNode(Node):
    def __init__(self):
        super().__init__(
            "jetson_csi_camera_ros2_driver_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        _camera_id = (
            self.get_parameter_or(
                "camera_id", Parameter("camera_id", Parameter.Type.INTEGER, 0)
            )
            .get_parameter_value()
            .integer_value
        )
        _image_width = (
            self.get_parameter_or(
                "image_width", Parameter("image_width", Parameter.Type.INTEGER, 400)
            )
            .get_parameter_value()
            .integer_value
        )
        _image_height = (
            self.get_parameter_or(
                "image_height", Parameter("image_height", Parameter.Type.INTEGER, 300)
            )
            .get_parameter_value()
            .integer_value
        )
        self.frame_id = (
            self.get_parameter_or(
                "frame_id", Parameter("frame_id", Parameter.Type.STRING, "camera_link")
            )
            .get_parameter_value()
            .string_value
        )
        _flip_method = (
            self.get_parameter_or(
                "flip_method", Parameter("flip_method", Parameter.Type.INTEGER, 0)
            )
            .get_parameter_value()
            .integer_value
        )

        self.get_logger().info("camera_id: %s" % (_camera_id))
        self.get_logger().info("image_width: %s" % (_image_width))
        self.get_logger().info("image_height: %s" % (_image_height))
        self.get_logger().info("frame_id: %s" % (self.frame_id))
        self.get_logger().info("flip_method: %s" % (_flip_method))

        self.image_publisher = self.create_publisher(
            Image, "Image", qos_profile_sensor_data
        )
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cap = cv2.VideoCapture(
            gstreamer_pipeline(
                sensor_id=_camera_id,
                width=_image_width,
                height=_image_height,
                flip_method=_flip_method,
            ),
            cv2.CAP_GSTREAMER,
        )
        self.br = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.br.cv2_to_imgmsg(frame, "bgr8")
            msg.header.frame_id = self.frame_id
            msg.header.stamp = super().get_clock().now().to_msg()
            self.image_publisher.publish(msg)
        else:
            self.get_logger().info("Fail to get video frame")
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriverNode()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    except SystemExit:
        node.get_logger().info("System Exit")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
