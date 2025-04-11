"""This module contains the configurations for different camera streams (e.g., RGB, Depth, Infrared) and additional
configuration settings necessary for stream initialization and operation.
"""
from modules.stream_type import StreamType, create_full_topic_name

common_stream_config: dict = {
    "camera_namespace": "camera",
    "camera_name": "D455_camera",
    "align_depth.enable": True
}

STREAM_HEALTH_TOPIC: str = create_full_topic_name("/health_status", common_stream_config)

COLOR_STREAM: StreamType = StreamType(name="color",
                                      enabled=True,
                                      width=1280,
                                      height=800,
                                      fps=30,
                                      topic=create_full_topic_name("/color/image_raw", common_stream_config))

DEPTH_STREAM: StreamType = StreamType(name="infra",
                                      enabled=True,
                                      width=1280,
                                      height=720,
                                      fps=90,
                                      topic=create_full_topic_name("/depth/image_raw", common_stream_config))

INFRA_STREAM: StreamType = StreamType(name="infra",
                                      enabled=False,
                                      width=1280,
                                      height=720,
                                      fps=90,
                                      topic=create_full_topic_name("/infra/image_raw", common_stream_config))
