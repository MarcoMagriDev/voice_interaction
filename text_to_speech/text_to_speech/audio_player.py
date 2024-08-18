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
import numpy as np
import sounddevice as sd
import soundfile as sf
import threading
import resampy

class AudioPlayer:
    def __init__(self, file, device=None):
        self.data, self.samplerate = sf.read(file)
        self.device = device
        self.stream = None
        self.thread = None
        self.playback_complete_event = threading.Event()
        self.playback_complete = True
        self.error_message = ""
        self.stop_playback = False  # Flag to signal stop request

    def play_audio(self):
        # Check if the audio data is mono or stereo
        if self.data.ndim == 1:  # Mono
            channels = 1
        elif self.data.ndim == 2:  # Stereo or multi-channel
            channels = self.data.shape[1]
        else:
            raise ValueError("Unsupported number of dimensions in audio data")

        try:
            device_info = sd.query_devices(self.device, "output")
        except ValueError as e:
            self.playback_complete = True
            self.error_message = f"{e}"
            return

        device_samplerate = device_info["default_samplerate"]
        if self.samplerate != device_samplerate:
            self.data = resampy.resample(self.data, self.samplerate, device_samplerate)
            self.samplerate = device_samplerate

        # Open a stream
        try:
            self.stream = sd.OutputStream(
                samplerate=self.samplerate,
                channels=channels,
                callback=self.callback,
                device=self.device,
            )
        except sd.PortAudioError as e:
            self.playback_complete = True
            self.error_message = f"{e}"
            return

        self.stream.start()
        # Wait until the file is done playing
        self.playback_complete_event.wait()
        self.stream.stop()
        self.stream.close()

    def callback(self, outdata, frames, time, status):
        if status:
            print(status)
        if self.data.size == 0 or self.stop_playback:  # Check if stop is requested
            self.playback_complete_event.set()  # Signal that playback is complete
            self.playback_complete = True
            raise sd.CallbackStop

        # Get the chunk of audio data
        chunk = self.data[:frames]

        # Handle cases where chunk is mono or stereo
        if chunk.ndim == 1:  # Mono
            chunk = chunk[:, np.newaxis]  # Convert to 2D array (shape: (frames, 1))

        # Ensure chunk matches the shape of outdata
        if chunk.shape[1] != outdata.shape[1]:
            raise ValueError(
                f"Chunk channels ({chunk.shape[1]}) do not match output channels ({outdata.shape[1]})"
            )

        # Output the chunk to the audio device
        outdata[: len(chunk)] = chunk

        # Remove the chunk from the data
        self.data = self.data[len(chunk) :]

    def start(self):
        self.playback_complete_event.clear()
        self.playback_complete = False
        self.stop_playback = False  # Reset the stop flag before starting playback
        self.thread = threading.Thread(target=self.play_audio, daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_playback = True  # Signal the callback to stop playback
        self.playback_complete_event.set()  # Ensure the event is set if stopping early
        if self.thread:
            self.thread.join()

    def wait_for_completion(self):
        self.playback_complete_event.wait()
