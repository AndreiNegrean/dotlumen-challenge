from dataclasses import dataclass


@dataclass
class StreamType:
    """Represents a supported camera stream type.

    This class is used to define the attributes for a specific camera stream, including the stream's name, whether it
    is enabled or not, its resolution (width and height), its frame rate (fps), and the associated ROS topic for the
    stream.
    """
    name: str
    enabled: bool
    width: int
    height: int
    fps: int
    topic: str


def create_full_topic_name(topic: str, stream_config: dict) -> str:
    """Constructs the full topic name by appending the camera namespace and camera name parameters.
    """
    assert "camera_namespace" in stream_config, "'camera_namespace' key not found in camera_config.py"
    assert "camera_name" in stream_config, "'camera_name' key not found in camera_config.py"
    return "/" + stream_config['camera_namespace'] + "/" + stream_config['camera_name'] + topic
