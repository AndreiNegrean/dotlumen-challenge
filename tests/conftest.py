import pytest
import logging
import rclpy
from modules.ros_listener import RosListener
from modules.stream_controller import StreamController


@pytest.fixture(scope="session", autouse=True)
def setup_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[logging.StreamHandler(), logging.FileHandler('test_logs.log')])
    logging.info("Logging setup complete.")


@pytest.fixture(scope="session", autouse=True)
def ros_setup() -> None:
    """Initialize ROS before running tests and shut down after all tests.
    """
    logging.info("Starting ROS2 Client...")
    rclpy.init()
    logging.info("Started successfully ROS2 Client.")
    yield
    logging.info("Stopping ROS2 Client...")
    rclpy.shutdown()
    logging.info("Stopped successfully ROS2 Client.")


@pytest.fixture(scope="class", autouse=True)
def ros_listener(request) -> None:
    """Fixture to initialize and provide the `RosListener` instance for the test class.

    @request: The request object provides access to the test context, including the test class (`request.cls`).
    """
    request.cls.ros_listener = RosListener()


@pytest.fixture(scope="class", autouse=True)
def stream_controller(request) -> None:
    """Fixture to initialize and provide the `StreamController` instance for the test class.

    @request: The request object provides access to the test context, including the test class (`request.cls`).
    """
    request.cls.stream_controller = StreamController()
