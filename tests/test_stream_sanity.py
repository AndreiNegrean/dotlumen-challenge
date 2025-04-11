import pytest
import logging
from typing import List
from modules.stream_type import StreamType
from modules.stream_controller import generate_yaml_config_file, ensure_yaml_config_file_exists
from config.stream_configurations import COLOR_STREAM, DEPTH_STREAM, INFRA_STREAM, STREAM_HEALTH_TOPIC

TEST_NAME: str = "Stream Sanity Test"
STREAM_DURATION_IN_SECONDS: int = 20
STREAMS_TO_TEST: List[StreamType] = [COLOR_STREAM, DEPTH_STREAM, INFRA_STREAM]
ERROR_MSG: str = "error dummy"


@pytest.mark.usefixtures("ros_listener")
@pytest.mark.usefixtures("stream_controller")
class TestStreamSanity:
    """Test class for verifying the sanity of camera streams. It ensures that each stream
    is functioning correctly by checking for any error messages published on the 'STREAM_HEALTH_TOPIC' topic.
    """

    def test_execution(self) -> None:
        """This method runs the necessary steps to execute the test, including setting up the test environment,
        running the test, and cleaning up afterward.
        """
        self.set_up_test()
        self.run_sanity_stream_test()
        self.tear_down_test()

    def run_sanity_stream_test(self) -> None:
        """Runs a sanity check for all configured streams by subscribing to common 'STREAM_HEALTH_TOPIC' topic.
        """
        logging.info(f"Running '{TEST_NAME}' test...")
        self.ros_listener.start_subscriber_for_specific_topic(topic=STREAM_HEALTH_TOPIC)
        for stream in STREAMS_TO_TEST:
            self.ros_listener.start_subscriber_for_specific_topic(topic=stream.topic)
            self.ros_listener.wait_ros_spin(STREAM_DURATION_IN_SECONDS)
            self.ros_listener.stop_subscriber_for_specific_topic(topic=stream.topic)
        self.check_stream_sanity(STREAM_HEALTH_TOPIC)
        self.ros_listener.stop_subscriber_for_specific_topic(topic=STREAM_HEALTH_TOPIC)
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

    def check_stream_sanity(self, topic: str) -> None:
        """Verifies the integrity of the stream by checking if any error messages are published on the specified topic.
        The method searches for error-related messages in the received messages and asserts that none are found.
        """
        logging.info("Checking stream for error messages...")
        found_error_msg: list = []
        for msg in self.ros_listener.received_messages[topic]:
            if ERROR_MSG in msg["msg"]["data"].lower():
                found_error_msg.append(msg["msg"]["data"])

        assert len(found_error_msg) == 0, f"There are error messages published on '{STREAM_HEALTH_TOPIC}' topic."
        logging.info(f"Checked that stream is error free. Topic '{STREAM_HEALTH_TOPIC}' has no error message published.")
