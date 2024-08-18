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
import inspect
import os
import uuid
from ament_index_python.packages import get_package_share_directory


def to_absolute_file_path(path: str):
    if path.startswith("/"):
        return path
    splitted = path.split("/")
    package_name = splitted.pop(0)
    return os.path.join(
        get_package_share_directory(package_name),
        *splitted,
    )


def get_constructor_keys(cls):
    signature = inspect.signature(cls.__init__)
    return [
        param.name for param in signature.parameters.values() if param.name != "self"
    ]


def get_temp_file_name(extension="wav"):
    unique_id = uuid.uuid4()
    return f"/tmp/{unique_id}.{extension}"
