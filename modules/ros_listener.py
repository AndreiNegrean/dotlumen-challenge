import logging
from typing import List, Dict
from functools import partial
import time
import rclpy
from rclpy.node import Node
from ros2topic.api import get_msg_class

NODE_TEST_NAME: str = "stream_test_node"
QOS_PROFILE: int = 10
SPIN_ONCE_TIMEOUT: float = 0.1


class RosListener(Node):
    """A ROS2 listener node that subscribes to topics and handles incoming messages.
    """
    def __init__(self) -> None:
        logging.info("Initialing RosListener...")
        super().__init__(NODE_TEST_NAME)
        self._published_topics: List = self._get_published_topics()
        self._received_messages: Dict = {}
        self._subscribers: Dict = {}
        logging.info("Initialized successfully RosListener.")

    @property
    def subscribers(self) -> Dict:
        """A dictionary where the keys are topic names and the values are the corresponding subscription objects.
        """
        return self._subscribers

    @property
    def published_topics(self) -> List:
        """A list of topic names (str) that the listener has published to.
        """
        return self._published_topics

    @property
    def received_messages(self) -> Dict:
        """A dictionary where the keys are topic names and the values are the corresponding topic received messages.
        """
        return self._received_messages

    def _get_published_topics(self) -> List:
        """Retrieve the list of topics that the node has published to.
        """
        logging.info("Retrieving the ROS published topics...")
        published_topics = [topic_name for topic_name, topic_type in self.get_topic_names_and_types()]
        logging.info("Retrieved successfully the ROS published topics.")
        return published_topics

    def start_subscriber_for_specific_topic(self, topic: str) -> None:
        """Starts a subscriber for a specific topic and begins receiving messages.

        This method subscribes to a specific topic and assigns a callback function to handle incoming messages.
        """
        logging.info(f"Starting ROS subscriber for topic '{topic}'...")
        assert topic in self._published_topics, f"Topic '{topic}' not found in the published ones."
        msg_type = get_msg_class(self, topic, include_hidden_topics=True)
        self._subscribers[topic] = (
            self.create_subscription(msg_type=msg_type,
                                     topic=topic,
                                     callback=partial(self.callback, topic),
                                     qos_profile=QOS_PROFILE))
        self._received_messages[topic] = []
        logging.info(f"Started successfully ROS subscriber for topic '{topic}'.")

    def stop_subscriber_for_specific_topic(self, topic: str) -> None:
        """Stops the subscriber for a specific topic.

        This method unsubscribes from a specified topic and stops processing incoming messages for that topic.
        """
        logging.info(f"Stopping ROS subscriber for topic '{topic}'...")
        assert topic in self._subscribers, f"Not found started subscriber for topic: '{topic}'."
        self.destroy_subscription(self._subscribers[topic])
        logging.info(f"Stopped successfully ROS subscriber for topic '{topic}'.")

    def start_subscribers_for_all_published_topics(self) -> None:
        """Starts subscribers for all topics the node is currently publishing to.

        This method iterates over all the topics that the node has published to and creates a subscriber for each topic.
        This is useful for subscribing to all topics without needing to specify each one manually.
        """
        logging.info(f"Starting ROS subscribers for all published topics...")
        for topic in self._published_topics:
            msg_type = get_msg_class(self, topic, include_hidden_topics=True)
            self._subscribers[topic] = (
                self.create_subscription(msg_type=msg_type,
                                         topic=topic,
                                         callback=partial(self.callback, topic),
                                         qos_profile=QOS_PROFILE))
            self._received_messages[topic] = []
        logging.info(f"Started successfully ROS subscriber for all published topics.")

    def stop_subscribers_for_all_published_topics(self) -> None:
        """Stops subscribers for all topics the node is currently subscribed to.

        This method iterates over all active subscriptions and stops each one, ensuring that the node no longer receives
        messages from any of the subscribed topics.
        """
        logging.info(f"Stopping ROS subscribers for all published topics...")
        for topic in self._subscribers:
            self.destroy_subscription(self._subscribers[topic])
        logging.info(f"Stopped successfully ROS subscriber for all published topics.")

    def callback(self, topic, msg) -> None:
        """Handles the incoming message for a specific topic and stores it with a timestamp.
        """
        self._received_messages[topic].append({"timestamp": time.time(), "msg": msg})

    def wait_ros_spin(self, wait_time: int) -> None:
        """Spins the ROS node for a specified amount of time, processing incoming events.

        This method continuously processes ROS events by calling `rclpy.spin_once()` within a loop. It will spin until
        the specified `wait_time` has elapsed or until the node is shut down.
        """
        logging.info(f"Waiting ROS spin for '{wait_time}' seconds...")
        start = time.time()
        while rclpy.ok() and (time.time() - start) < wait_time:
            rclpy.spin_once(self, timeout_sec=SPIN_ONCE_TIMEOUT)
        logging.info(f"Done.")

    def stop(self) -> None:
        """Stops the RosListener by destroying the node.
        """
        logging.info(f"Stopping ROS Node...")
        self.destroy_node()
        logging.info(f"Stopped successfully the ROS Node.")
