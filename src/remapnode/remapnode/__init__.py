import rclpy
from remapnode.node import RemapNode

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = RemapNode()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()