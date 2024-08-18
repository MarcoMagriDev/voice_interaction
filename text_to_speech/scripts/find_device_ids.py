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
import argparse
import time
from typing import List
import numpy as np
import sounddevice as sd


class Device:
    class Types:
        IO: str = "I/O Device"
        OUTPUT: str = "Output Device"
        INPUT: str = "Input Device"

    name: str
    index: int
    card: int
    id: int
    type: Types
    sample_rate: int

    def __init__(self, dict) -> None:
        self.name = dict["name"]
        self.index = dict["index"]
        if dict["max_input_channels"] > 0 and dict["max_output_channels"] > 0:
            self.type = Device.Types.IO
        elif dict["max_input_channels"] > 0:
            self.type = Device.Types.INPUT
        elif dict["max_output_channels"] > 0:
            self.type = Device.Types.OUTPUT

        self.samplerate = dict["default_samplerate"]

    def __repr__(self) -> str:
        return f"{self.type} - index: {self.index} - name: {self.name}"


def test_output(output_devices: List[Device]):
    frequency = 440  # 440 Hz is the frequency of A4
    duration = 2.0  # Duration in seconds

    for device in output_devices:
        print(f"Checking {device}")
        t = np.linspace(0, duration, int(device.samplerate), False)
        test_tone = 0.5 * np.sin(2 * np.pi * frequency * t)
        try:
            sd.play(
                test_tone,
                device.samplerate,
                device=device.index,
            )
            sd.wait()
            time.sleep(2)
        except sd.PortAudioError:
            print(f"Unable to play on device with id {device['index']}")


def test_input(input_devices: List[Device]):
    duration = 3.0  # Duration in seconds
    sample_rate = 48000  # Sample rate in samples per second (common value)
    for device in input_devices:
        # Record audio from the specified input device
        print(f"Recording from {device}...")
        audio_data = sd.rec(
            int(sample_rate * duration),
            samplerate=device.samplerate,
            channels=1,
            device=device.index,
        )
        sd.wait()

        print(f"Replaying.. ")
        sd.play(
            audio_data,
            sample_rate,
        )
        sd.wait()

def print_devices(devices: List[Device]):
    print(50 * "-")
    for d in devices:
        print(d)
    print(50 * "-")
    print()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-i",
        "--input",
        help="Allow to test input devices",
        action="store_true",
        required=False,
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Allow to test output devices",
        action="store_true",
        required=False,
    )
    known_args, unknown_args = parser.parse_known_args()
    args = vars(known_args)

    devices = [Device(d) for d in sd.query_devices()]
    
    input_devices = [
        d for d in devices if d.type == Device.Types.INPUT or d.type == Device.Types.IO
    ]

    output_devices = [
        d for d in devices if d.type == Device.Types.OUTPUT or d.type == Device.Types.IO
    ]

    if args["output"]:
        print("Found output devices: ")
        print_devices(output_devices)
        test_output(output_devices)
    if args["input"]:
        print("Found input devices: ")
        print_devices(input_devices)
        test_input(input_devices)
