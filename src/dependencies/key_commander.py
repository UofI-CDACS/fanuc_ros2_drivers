from pynput import keyboard
from pynput.keyboard import KeyCode

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class KeyCommander(Node):
    """
    A class that allows keys to be registered with
    callbacks to be executed when they are pressed.
    """

    def __init__(self, key_callbacks=[]):
        super().__init__('figure_four_key_commander')
        self._key_pressed_publisher = self.create_publisher(String, 'key_pressed', 10)

        # add callbacks that are registered with keys
        self._key_callbacks = key_callbacks
        # create a listener for the keyboard that will notify us of presses and releases
        self._listener = keyboard.Listener(
                on_press=self.notify_key_pressed,
                on_release=self.notify_key_released)

        # start the threaded listener
        self._listener.start()

    # stop listening to the keyboard key events
    def stop(self):
        self._listener.stop()

    # callback if a key is pressed 
    def notify_key_pressed(self, key):
        #self.get_logger().info(f'key pressed: {key}')
        # check for callback attached to this key and execute if found
        for key_cb_pair in self._key_callbacks:
            if key_cb_pair[0] == key:
                self.get_logger().info('executing callback attached to key')
                # keys worth taking note of are published
                self.publish_key_pressed(key_cb_pair[0])
                # execute callback
                key_cb_pair[1]()

    # log the key event to the terminal
    def notify_key_released(self, key):
        #self.get_logger().info(f'key released: {key}')
        return

    # notify subscribing nodes which key was pressed
    def publish_key_pressed(self, key):
        self.get_logger().info(f'publishing {key}')
        msg = String()
        msg.data = f'{key}'
        self._key_pressed_publisher.publish(msg)

