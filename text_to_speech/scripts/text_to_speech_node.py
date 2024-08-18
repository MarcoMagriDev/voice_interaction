#!/usr/bin/env python3
#
# Copyright (c) 2024 Marco Magri
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import os
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer, ServerGoalHandle, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from voice_interaction_msgs.action import Say
from text_to_speech import BACKENDS, WrongVoiceConfig, AudioPlayer
from text_to_speech.utils import to_absolute_file_path


DEFAULT_VOICE_CONFIG = """
type: piper
params:
  model_file: text_to_speech/config/voices/piper/en_GB-northern_english_male-medium.onnx
"""


class TextToSpeechNode(Node):
    def __init__(self):
        super().__init__("text_to_speech")

        self.default_voice_config = (
            self.declare_parameter(
                "default_voice_config",
                DEFAULT_VOICE_CONFIG,
            )
            .get_parameter_value()
            .string_value
        )

        self.default_device_indexes = (
            self.declare_parameter("default_device_indexes", [-1])
            .get_parameter_value()
            .integer_array_value
        )

        self.__action_server = ActionServer(
            self,
            Say,
            "say",
            execute_callback=self.__execute_callback,
            cancel_callback=self.__cancel_callback,
        )

        self.audio_players = []

    def __cancel_callback(self, cancel_request) -> None:
        for a in self.audio_players:
            a.stop()
        return CancelResponse.ACCEPT

    def __execute_callback(self, goal_handle: ServerGoalHandle) -> Say.Result:
        request: Say.Goal = goal_handle.request
        result = Say.Result()

        # If no voice config in the request -> use the default config
        request.voice_config = (
            to_absolute_file_path(request.voice_config)
            if request.voice_config
            else to_absolute_file_path(self.default_voice_config)
        )

        try:
            with open(request.voice_config, "r") as f:
                voice_configuration_dict = yaml.safe_load(f)
        except FileNotFoundError:
            voice_configuration_dict = yaml.safe_load(request.voice_config)

        # Check backend type provided in configuration dict
        try:
            backend_type = voice_configuration_dict["type"]
        except KeyError:
            result.message = f"Backend type not specified in voice configuration dict"
            goal_handle.abort()
            return result

        # Check backend type is known
        if not backend_type in BACKENDS:
            result.message = f"Unknown backend {backend_type}. Known backends are {list(BACKENDS.keys())}"
            goal_handle.abort()
            return result

        goal_handle.publish_feedback(Say.Feedback(status=Say.Feedback.CONVERTING))
        try:
            audio_file = BACKENDS[backend_type].text_to_audio_file(
                request.text, voice_configuration_dict["params"]
            )
        except WrongVoiceConfig as e:
            result.message = f"{e}"
            goal_handle.abort()
            return result

        devices = request.devices
        if not devices:
            devices = (
                [None]
                if -1 in self.default_device_indexes
                else self.default_device_indexes
            )

        audio_players = [AudioPlayer(audio_file, device) for device in devices]

        goal_handle.publish_feedback(Say.Feedback(status=Say.Feedback.PLAYING))
        for a in audio_players:
            a.start()

        while not all([a.playback_complete for a in audio_players]):
            if goal_handle.is_cancel_requested:
                for a in audio_players:
                    a.stop()
                result.message = "Cancel request received. Say has been interrupted."
                goal_handle.canceled()
                return result
            time.sleep(0.01)

        for a in audio_players:
            if a.error_message:
                self.get_logger().error(
                    f"Unable to play audio on device {a.device}: {a.error_message}"
                )

        os.remove(audio_file)
        result.message = "Text said successfully"
        goal_handle.succeed()
        return result


def main():
    rclpy.init(args=None)
    text_to_speech_node = TextToSpeechNode()
    executor = MultiThreadedExecutor()
    executor.add_node(text_to_speech_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        text_to_speech_node.destroy_node()

    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
