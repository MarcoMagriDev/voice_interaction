# Voice interaction

Collection of packages for voice interaction in ROS2

* `voice_interaction_msgs`: contains interfaces messages, services and actions
* `text_to_speech`: provides text to speech functionalities with multiple backends
* `speech_to_text`: provides speech to text functionalities with multiple backends


## Echo-cancelling setup ubuntu
To ensure your Speech-to-Text (STT) and Text-to-Speech (TTS) functionalities work seamlessly without interference, it's crucial to prevent the system from transcribing its own responses. This can be achieved by setting up echo cancellation in PulseAudio as described in the guide available [here](https://www.linuxuprising.com/2020/09/how-to-enable-echo-noise-cancellation.html) or by using an headset.