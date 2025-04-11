import pytest
import logging
from typing import List
from modules.stream_type import StreamType
from modules.stream_controller import generate_yaml_config_file, ensure_yaml_config_file_exists
from config.stream_configurations import COLOR_STREAM, DEPTH_STREAM

TEST_NAME: str = "Stream FPS Test"
STREAM_DURATION_IN_SECONDS: int = 60
STREAMS_TO_TEST: List[StreamType] = [COLOR_STREAM, DEPTH_STREAM]


@pytest.mark.usefixtures("ros_listener")
@pytest.mark.usefixtures("stream_controller")
class TestStreamFPS:
    """Test class for validating the FPS of different camera streams.

    This class contains test methods to ensure that the FPS of specified streams to test ('STREAMS_TO_TEST') matches
    the expected values. It includes setup and teardown methods for preparing the testing environment and cleaning up
    after tests.
    """

    def test_execution(self) -> None:
        """This method runs the necessary steps to execute the test, including setting up the test environment,
        running the test, and cleaning up afterward.
        """
        self.set_up_test()
        self.run_fps_stream_test()
        self.tear_down_test()

    def run_fps_stream_test(self) -> None:
        """Runs the FPS test for each stream specified in 'STREAMS_TO_TEST'.

        Subscribes to each stream's topic, waits for a specified duration to receive messages,
        stops the subscriber, and then checks if the FPS is within the expected range.
        """
        logging.info(f"Running '{TEST_NAME}' test...")
        for stream in STREAMS_TO_TEST:
            self.ros_listener.start_subscriber_for_specific_topic(topic=stream.topic)
            self.ros_listener.wait_ros_spin(STREAM_DURATION_IN_SECONDS)
            self.ros_listener.stop_subscriber_for_specific_topic(topic=stream.topic)
            self.check_stream_fps(stream)
        logging.info(f"'{TEST_NAME}' run complete.")

    def set_up_test(self) -> None:
        """Prepares the environment for the test by configuring and starting the camera stream.
        """
        logging.info("Set up test...")
        generate_yaml_config_file()
        ensure_yaml_config_file_exists()
        self.stream_controller.start_stream()
        logging.info("Set up complete.")

    def tear_down_test(self) -> None:
        """Cleans up resources and stops any active streams after the test.
        """
        logging.info("Tear down test...")
        self.ros_listener.stop()
        self.stream_controller.stop_stream()
        logging.info("Tear down complete.")

    def check_stream_fps(self, stream: StreamType) -> None:
        """This method compares the calculated FPS for a given stream with the expected FPS value. If the actual FPS is
        outside the acceptable range of the expected FPS +- tolerance, an assertion error is raised.
        """
        logging.info(f"Checking stream '{stream.name}' FPS...")
        tolerance: float = 0.5
        expected_fps: float = float(stream.fps)
        min_accepted_fps: float = expected_fps * (1 - tolerance / 100)
        max_accepted_fps: float = expected_fps * (1 + tolerance / 100)
        actual_fps: float = self._calculate_frame_rate(stream.topic)

        assert min_accepted_fps <= expected_fps <= max_accepted_fps, \
            f"FPS doesn't match the expected value. Actual: {actual_fps}. Expected: {expected_fps} (+- 5% tolerance)"
        logging.info(f"Checked that stream '{stream.name}' has expected FPS.")

    def _calculate_frame_rate(self, topic) -> float:
        """This method computes the frame rate by determining the time difference between the first and last frames
        received for the specified topic and divides the number of frames by this time difference.
        """
        number_of_frames = len(self.ros_listener.received_messages[topic])
        if number_of_frames < 2:
            return 0.0
        else:
            first_frame_ts = self.ros_listener.received_messages[topic][0]["timestamp"]
            last_frame_ts = self.ros_listener.received_messages[topic][-1]["timestamp"]
            delta = last_frame_ts - first_frame_ts
            return (number_of_frames - 1) / delta if delta > 0 else 0.0
