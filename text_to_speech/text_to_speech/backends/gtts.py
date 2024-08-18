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
from typing import Any, Dict
from gtts import gTTS
from ..tts import TTS
from ..utils import get_constructor_keys, get_temp_file_name
from rclpy.logging import get_logger

LOGGER = get_logger("text_to_speech.gtts")


class GTTS(TTS):
    def __init__(self) -> None:
        super().__init__()

    def text_to_audio_file(self, text: str, voice_config_dict: Dict[str, Any]) -> str:
        available_configurations = get_constructor_keys(gTTS)
        config_dict = {}
        for key, value in voice_config_dict.items():
            if key in available_configurations:
                config_dict[key] = value
            else:
                LOGGER.warning(
                    f"Provided config `{key}` is not valid and thus will not be applied."
                )

        tmp_audio_file = get_temp_file_name()
        if len(text) > 0 and text.split(" ")[0] not in [".", "!", "?", ","]:
            try:
                tts = gTTS(text=text, **config_dict)
                tts.save(tmp_audio_file)
                return tmp_audio_file
            except AssertionError:
                print("assertion error.")
                pass
            except:
                pass
