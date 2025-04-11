import logging
import subprocess
import os
import yaml
from config.stream_configurations import common_stream_config, COLOR_STREAM, DEPTH_STREAM, INFRA_STREAM

ROS_LAUNCH_COMMAND: str = "ros2 launch"
CAMERA_MANAGER_PACKAGE_NAME: str = "camera_manager_pkg"
ROS_LAUNCH_FILE_NAME: str = "launch_file"
ROS_LAUNCH_CONFIG_FILE_PATH: str = "./config/camera_config.yaml"
ROS_LAUNCH_CONFIG_FILE_ARG: str = "config_file:=" + ROS_LAUNCH_CONFIG_FILE_PATH


class StreamController:
    """A class to manage and control the camera streams, including RGB, Depth, and Infrared.

    This class provides methods for starting, stopping, and monitoring the camera stream. It interacts with the
    underlying hardware (e.g., Intel RealSense D455) and allows testing components to retrieve stream information.
    """
    def __init__(self) -> None:
        logging.info("Initialing StreamController...")
        self._stream_process: subprocess.Popen
        logging.info("Initialized successfully StreamController.")

    @property
    def stream_process(self) -> subprocess.Popen:
        """Returns the subprocess that is running the camera stream.

        This method provides access to the 'subprocess.Popen' object that controls the camera streaming process.
        It allows you to inspect or manage the streaming process (e.g. check if it's running, send signals, etc.).
        """
        return self._stream_process

    def start_stream(self) -> None:
        """Starts the camera streaming by launching a ROS launch file via a subprocess.

        This method initializes the camera stream by executing a ROS launch command through a subprocess. It uses the
        'roslaunch' tool to start the CameraManager node, which is responsible for setting up and controlling the camera
        streams. The streaming process is launched in a separate process to ensure it does not block the main execution.
        """
        logging.info("Starting stream process...")
        self._stream_process = subprocess.Popen(
            [ROS_LAUNCH_COMMAND, CAMERA_MANAGER_PACKAGE_NAME, ROS_LAUNCH_FILE_NAME, ROS_LAUNCH_CONFIG_FILE_ARG])
        logging.info("Started successfully stream process.")

    def stop_stream(self) -> None:
        """Stops the camera streaming process by sending a SIGTERM signal.

        This method terminates the running camera stream by sending the 'SIGTERM' signal to the subprocess responsible
        for the stream. The subprocess is expected to handle the termination gracefully.
        """
        logging.info("Stopping stream process...")
        self._stream_process.terminate()
        logging.info("Stopping successfully stream process.")


def generate_yaml_config_file() -> None:
    """Generates the YAML configuration file for the CameraManager.
    """
    logging.info("Generating YAML config file...")
    config_dict = {
        "enable_color": COLOR_STREAM.enabled,
        "rgb_camera.profile": f"{COLOR_STREAM.width}x{COLOR_STREAM.height}x{COLOR_STREAM.fps}",
        "enable_depth": DEPTH_STREAM.enabled,
        "depth_module.depth_profile": f"{DEPTH_STREAM.width}x{DEPTH_STREAM.height}x{DEPTH_STREAM.fps}",
        "enable_infra": INFRA_STREAM.enabled,
        "depth_module.infra_profile": f"{INFRA_STREAM.width}x{INFRA_STREAM.height}x{INFRA_STREAM.fps}",
    }
    config_dict.update(common_stream_config)

    with open(ROS_LAUNCH_CONFIG_FILE_PATH, "w+") as yaml_config_file:
        yaml.dump(config_dict, yaml_config_file)
    logging.info("Generated successfully YAML config file.")


def ensure_yaml_config_file_exists() -> None:
    """Ensures that the YAML configuration file exists for the camera manager.
    """
    logging.info("Checking if YAML config file exists...")
    assert os.path.exists(ROS_LAUNCH_CONFIG_FILE_PATH), f"YAML config file not found: '{ROS_LAUNCH_CONFIG_FILE_PATH}'"
    logging.info("Checked that YAML config file exists.")
