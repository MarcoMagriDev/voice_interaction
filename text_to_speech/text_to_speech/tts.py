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
from abc import ABC, abstractmethod


class WrongVoiceConfig(Exception):
    def __init__(self, str: str):
        super().__init__(str)


class TTS(ABC):
    @abstractmethod
    def text_to_audio_file(text: str, voice_config_dict: Dict[str, Any]) -> str:
        pass
