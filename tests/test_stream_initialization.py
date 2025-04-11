import pytest
import logging
from typing import List
from modules.stream_type import StreamType
from modules.stream_controller import generate_yaml_config_file, ensure_yaml_config_file_exists
from config.stream_configurations import COLOR_STREAM, DEPTH_STREAM, INFRA_STREAM

TEST_NAME: str = "Stream Initialization Test"
STREAMS_TO_TEST: List[StreamType] = [COLOR_STREAM, DEPTH_STREAM, INFRA_STREAM]


@pytest.mark.usefixtures("ros_listener")
@pytest.mark.usefixtures("stream_controller")
class TestStreamInitialization:
    """Test the initialization of the camera streams.

    This class verifies that the camera manager node correctly starts streaming based on the provided
    configuration file. It checks that the expected topics are published and that the streams are
    initialized properly according to the configuration.
    """

    def test_execution(self) -> None:
        """This method runs the necessary steps to execute the test, including setting up the test environment,
        running the test, and cleaning up afterward.
        """
        self.set_up_test()
        self.run_stream_init_test()
        self.tear_down_test()

    def run_stream_init_test(self) -> None:
        """Runs the stream initialization test for each stream specified in 'STREAMS_TO_TEST'.
        """
        logging.info(f"Running '{TEST_NAME}' test...")
        for stream in STREAMS_TO_TEST:
            self.check_stream_init(stream)
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

    def check_stream_init(self, stream: StreamType) -> None:
        """This method checks if the topic associated with the provided stream is present in the list of published
        topics by the RosListener. It ensures that the stream has been successfully initialized and the corresponding
        topic is published.
        """
        logging.info(f"Checking if stream '{stream.name}' initialized...")
        assert stream.topic in self.ros_listener.published_topics, \
            f"Topic '{stream.topic}' not found in ROS published topics. The stream '{stream.name}' might " \
            f"not be started successfully. Check if the stream is configured as enabled."
        logging.info(f"Checked that stream '{stream.name}' initialized successfully.")
