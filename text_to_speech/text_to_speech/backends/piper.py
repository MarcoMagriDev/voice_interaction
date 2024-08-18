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
from dataclasses import dataclass
from typing import Any, Dict
import wave
from ..tts import TTS, WrongVoiceConfig
from ..utils import to_absolute_file_path, get_temp_file_name
from piper.voice import PiperVoice


@dataclass
class PiperConfig:
    model_file: str

    @staticmethod
    def from_dict(dict: Dict[str, Any]):
        try:
            model_file = to_absolute_file_path(dict["model_file"])
        except KeyError:
            raise WrongVoiceConfig(
                "`model_file` missing in provided voice configuration"
            )

        return PiperConfig(model_file)


class Piper(TTS):
    def __init__(self) -> None:
        super().__init__()

    def text_to_audio_file(self, text: str, voice_config_dict: Dict[str, Any]):
        piper_config = PiperConfig.from_dict(voice_config_dict)
        voice = PiperVoice.load(piper_config.model_file)

        temp_file_name = get_temp_file_name()
        voice.synthesize(text, wave.Wave_write(temp_file_name))

        return temp_file_name
