import pytest
import logging
from typing import List
from modules.stream_type import StreamType
from modules.stream_controller import generate_yaml_config_file, ensure_yaml_config_file_exists
from config.stream_configurations import COLOR_STREAM, DEPTH_STREAM

TEST_NAME: str = "Stream Latency Test"
STREAM_DURATION_IN_SECONDS: int = 60
STREAMS_TO_TEST: List[StreamType] = [COLOR_STREAM, DEPTH_STREAM]


@pytest.mark.usefixtures("ros_listener")
@pytest.mark.usefixtures("stream_controller")
class TestStreamLatency:
    """Test class for verifying the end-to-end latency of camera streams in ROS.

    Measures the latency between camera frame capture and the corresponding ROS topic publication.
    Ensures that the latency for specified streams to test ('STREAMS_TO_TEST') is within an acceptable threshold.
    """

    def test_execution(self) -> None:
        """This method runs the necessary steps to execute the test, including setting up the test environment,
        running the test, and cleaning up afterward.
        """
        self.set_up_test()
        self.run_latency_stream_test()
        self.tear_down_test()

    def run_latency_stream_test(self) -> None:
        """Runs the Latency test for each stream specified in 'STREAMS_TO_TEST'.

        Subscribes to each stream's topic, waits for a specified duration to receive messages,
        stops the subscriber, and then checks if the latency is within the expected range.
        """
        logging.info(f"Running '{TEST_NAME}' test...")
        for stream in STREAMS_TO_TEST:
            self.ros_listener.start_subscriber_for_specific_topic(topic=stream.topic)
            self.ros_listener.wait_ros_spin(STREAM_DURATION_IN_SECONDS)
            self.ros_listener.stop_subscriber_for_specific_topic(topic=stream.topic)
            self.check_stream_latency(stream)
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

    def check_stream_latency(self, stream: StreamType) -> None:
        """Verifies the latency for the given stream by comparing the frame capture time (from the message header)
        with the subscriber time when the message was received.
        """
        logging.info(f"Checking stream '{stream.name}' latency...")
        latency_threshold: float = 0.1
        latencies_that_exceeds_the_threshold: dict = self._calculate_latencies_that_exceeds_the_threshold(stream.topic, latency_threshold)
        # TODO: Implement a solution to report the frames that exceeds the latency threshold

        assert len(latencies_that_exceeds_the_threshold) == 0, \
            f"There are '{len(latencies_that_exceeds_the_threshold)}' frames that exceeds " \
            f"the '{latency_threshold}s' latency threshold."
        logging.info(f"Checked that stream '{stream.name}' has no latencies that exceeds the threshold.")

    def _calculate_latencies_that_exceeds_the_threshold(self, topic, latency_threshold: float) -> dict:
        """This method calculates if a frame exceeds the latency threshold by determining the time difference
        between the camera capture timestamp from msg header and ros subscriber message received timestamp.
        """
        latencies_that_exceeds_the_threshold = {}
        for msg in self.ros_listener.received_messages[topic]:
            latency: float = float(msg["timestamp"] - msg["msg"]["header"]["timestamp"] / 1e9)

            if latency > latency_threshold:
                latencies_that_exceeds_the_threshold[msg] = latency

        return latencies_that_exceeds_the_threshold
