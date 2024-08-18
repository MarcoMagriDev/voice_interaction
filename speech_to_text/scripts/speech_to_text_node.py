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
from threading import Thread
from typing import Callable
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import speech_recognition as sr

from std_msgs.msg import String
from std_srvs.srv import Trigger


class STTParams:
    type: str
    model: str
    language: str

    def __init__(self, node: Node) -> None:
        self.type = (
            node.declare_parameter("stt_client.type", "faster_whisper")
            .get_parameter_value()
            .string_value
        )
        self.model = (
            node.declare_parameter("stt_client.model", "large-v3")
            .get_parameter_value()
            .string_value
        )
        self.language = (
            node.declare_parameter("stt_client.language", "en")
            .get_parameter_value()
            .string_value
        )

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__("speech_to_text")
        self.mic = sr.Microphone(
            device_index=(
                self.declare_parameter("device_index", Parameter.Type.INTEGER)
                .get_parameter_value()
                .integer_value
            )
        )

        self.recognizer = sr.Recognizer()
        self.recognizer.pause_threshold = (
            self.declare_parameter("vad.pause_threshold", 0.5)
            .get_parameter_value()
            .double_value
        )
        self.recognizer.phrase_threshold = (
            self.declare_parameter("vad.phrase_threshold", 0.1)
            .get_parameter_value()
            .double_value
        )
        self.recognizer.energy_threshold = (
            self.declare_parameter("vad.energy_threshold", 200)
            .get_parameter_value()
            .integer_value
        )
        self.recognizer.dynamic_energy_threshold = (
            self.declare_parameter("vad.dynamic_energy_threshold", False)
            .get_parameter_value()
            .bool_value
        )
        self.efficient_wordnet_detector = None

        start_enabled = (
            self.declare_parameter("start_enabled", False)
            .get_parameter_value()
            .bool_value
        )

        self.text_publisher = self.create_publisher(String, "~/text", 10)

        self._enable_service = self.create_service(
            Trigger, "~/enable", self.enable_callback
        )
        self._disable_service = self.create_service(
            Trigger, "~/disable", self.disable_callback
        )

        if start_enabled:
            self.recognizer.listen_in_background(self.mic, self._process_audio)

    def enable_callback(self, request: Trigger.Request, response: Trigger.Response):
        if not self._listener_stopper is None:
            response.success = True
            response.message = "Continuous listening is already active."
            return response

        self._listener_stopper = self.recognizer.listen_in_background(
            self.mic, self._process_audio
        )
        response.success = True
        response.message = "Continuous listening enabled."
        return response

    def disable_callback(self, request: Trigger.Request, response: Trigger.Response):
        if self._listener_stopper is None:
            response.success = True
            response.message = "Continuous listening is not active."
            return response

        self._listener_stopper()
        response.success = True
        response.message = "Continuous listening disabled."
        return response

    def _process_audio(self, recognizer, audio):
        try:
            self.get_logger().info(f"Converting speech to text ... ")
            text = recognizer.recognize_faster_whisper(
                audio, language="en", model="large-v3", device="cuda"
            )
            # TODO: add multiple implementations
            # return self.recognizer.recognize_google(audio, language="en-GB")
            # return self.recognizer.recognize_whisper_api(audio, language="en")
            # return self.recognizer.recognize_google_cloud(audio, credentials_json="credentials.json")
            # return json.loads(self.recognizer.recognize_vosk(audio))["text"]
            self.get_logger().info(f"Text converted! Result: {text} ")
            self.text_publisher.publish(String(data=text))
        except Exception as e:
            self.get_logger().error(f"{e}")


def main():
    rclpy.init(args=None)
    speech_to_text_node = SpeechToTextNode()

    try:
        rclpy.spin(speech_to_text_node)
    except KeyboardInterrupt:
        speech_to_text_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
