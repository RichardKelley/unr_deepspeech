# unr_deepspeech - Speech Recognition for Robots

The unr_deepspeech package is a ROS node providing speech-to-text for
robots. It does this by wrapping Mozilla's TensorFlow implementation
of Baidu's DeepSpeech network for speech recognition. The system works
entirely offline, and provides high-quality transcriptions using
pretrained neural networks available from Mozilla. It also aims to be
easy to install, with minimal configuration required to produce
acceptable transcriptions.

The package has been tested in ROS Kinetic and ROS Melodic.

## Prerequisites

To install this package, you must first have installed the Python
DeepSpeech packaage. This can be done through pip using either:

```
pip install deepspeech
```

or

```
pip install deepspeech-gpu
```

depending on whether or not you have a GPU available on your
system. If you have a TensorFlow-compatible GPU, we recommend that you
use it.

## Installation

Installing this package follows the normal ROS conventions. Download
the package to a ROS workspace, download a pretrained model into the
`model/` directory, and run `catkin build`.

### Getting a model.

To get started you will probably want to use a pretrained
model. Mozilla provides a model that achieves excellent performance on
spoken English. To download that model, change into the `model/`
directory and run the following command:

```
wget -O - https://github.com/mozilla/DeepSpeech/releases/download/v0.3.0/deepspeech-0.3.0-models.tar.gz | tar xvfz -
```

After this command runs, your directory tree should look like this:

```
.
├── CMakeLists.txt
├── data
├── model
│   ├── alphabet.txt
│   ├── lm.binary
│   ├── output_graph.pb
│   ├── output_graph.pbmm
│   ├── output_graph.rounded.pb
│   ├── output_graph.rounded.pbmm
│   └── trie
├── package.xml
├── README.md
├── setup.py
├── src
│   ├── deepspeech_node.py
│   ├── __init__.py
│   ├── unr_deepspeech_client.py
│   └── unr_deepspeech_server.py
└── srv
    └── Listen.srv

4 directories, 16 files
```

Once the model is downloaded, you should be ready to use the system.

## Usage

### Server
To use the system, first start the server. Make sure a `roscore` is
running, and then in a terminal where you have already sourced
`setup.bash` for your workspace, run the command

```
rosrun unr_deepspeech unr_deepspeech_server.py
```

Now you can use the service.

In addition to raw transcriptions, the server supports correcting the
transcription using a dictionary file. To use this feature, create a
file in the package directory containing the list of words you want to
allow the system to output, one word per line. Then, before starting
the server, give the file name to the ROS parameter server by running
this command:

```
rosparam set /unr_deepspeech/dictionary dictionary.txt
```

Now start the server. The server will list the full path of the file,
or produce an error if it can't find the file.

### Client

Once the server is running, you can use the ROS service interface to
transcribe audio files. We provide a demo that shows how to use the
client code. To use the demo, run the command

```
rosrun unr_deepspeech unr_deepspeech_client.py
```

This will listen for audio through your microphone, write the audio to
a `wav` file, and then send the filename to the server for
transcription. The server will then send back a response containing
the recognized text, which the client will print before
exiting. Running the above command will attempt to use what your
system considers to be the "default" audio input device. To get a list
of available devices, run this command:

```
rosrun unr_deepspeech unr_deepspeech_client.py -1
```

This will print a list of devices and their indices and then exit. If,
for example, you had a microphone listed as device 14, then you could
ask the client to use that device by running the following command:

```
rosrun unr_deepspeech unr_deepspeech_client.py 14
```

If the device cannot generate audio in the format required by
DeepSpeech, the client will print an error and then exit.

## Caveats, Requirements, and Notes

The Mozilla DeepSpeech model requires single-channel audio and a 16
kHz sample rate. The client program gives an example of how to convert
audio into this format using Python. If you generate your own audio
through another source and run into issues, check that your audio data
is compatible with DeepSpeech's format requirements.

At the moment, the client and server have to be running on the same
machine. This requirement will be lifted once we transition from using
`wav` files on disk to streaming audio in memory.

The performance of the system (both in terms of speed and
transcription quality) depends heavily on the quality of your
microphone. During development, we tested the system with builtin
laptop microphones and a Nessie Blue USB microphone, and found that
the USB microphone substantially outperformed the laptop
microphone. If you have trouble getting decent transcriptions, we
recommend testing with a better microphone.

This package was written by Bryson Lingenfelter, Nate Thom, Sybille
Horcholle, and Richard Kelley.