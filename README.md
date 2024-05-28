# dexhandv2_cpp_client
DexHand V2 C++ Client Library and Demonstration Client for Connecting to DexHand Hardware

## Project Overview
This is a C++ library that facilitates connecting to the V2 DexHand - a Dexterous Humanoid Robot Hand. 
You can learn more about the DexHand at http://www.dexhand.com.

The C++ client library is intended to be a very thin layer on top of the actual servo motors that control
the hand positions. High level controls and integrations with platforms such as ROS 2 and other controllers
will be provided separately. If you want to interface directly to a DexHand via USB, using C++, this package
is for you. 

We also provide a [Python version of this package](https://github.com/iotdesignshop/dexhandv2_python_client) if you prefer
to interface in Python. In fact, the Python package uses the C++ package internally, providing bindings to the
underlying C++ objects. This helps improve performance, and also ensures that we have consistency in the SDK's. 
So, the choice is really yours depending on which language you prefer for your application.

## Dependencies

### Linux Only

This library currently is only scoped to build and run on linux systems. We test on Ubuntu 22.04 LTS. 
If you'd like to take a stab at getting it to compile for other platforms, we would welcome those PR's
as it would be great to have it running on Windows, and MacOS as well.

### Packages

__libudev-dev__ - Required for USB communications to the dexhand
__protobuf-compiler__ - For compiling proto files
__libprotobuf-dev__ - Google Protocol Buffer library
__pkg-config__

## Installation

### Git
Clone this package to a working folder on your machine using Git. Then initialize the submodules using:

```(bash)
git submodule update --init --recursive
```

## Building the Package

The package uses CMAKE to compile. Most of the time, you will open the project folder in an IDE such as Visual Studio
Code, and the CMakefiles.txt file will automatically be discovered and offer you the option to configure the project.

However, if you also wish to build from the command line, you can do the following from the folder you cloned the 
repository into:

```(bash)
mkdir build
cd build
cmake ..
make
```

This will create an executable in the build folder named ```sample-client``` which you can run:

```(bash)
./sample-client
```

If you have a DexHand device plugged into USB, then it should start to move, printing the position and other relevant servo 
metrics out to the screen.

## Architectural Overview

There are a couple different modes for working with the package to control a DexHand depending on your requirements:

- __Direct Message Exchange__ - This is basically a "low level" interface to the hand where you subscribe to the status
messages coming from the hand, and broadcast control messages back to the motors in the hand. This mode requires a bit
of understanding of how the messaging structure works, and managing timing and such to avoid letting buffers overflow.
- __Servo Abstraction Layer__ - This high level interface virtualizes the servo state into objects and you just interact
with these virtual servos in order to send commands to the hand and to see the state of the servo. A ServoManager object
parses all inbound messages and schedules outbound messages automatically based on any changes to the servo objects.

If you are just getting started, it's probably easiest to use the ServoManager and abstraction layer to control the 
servos in the hand. You can still control the frequency of updates and other parameters to tune the communications, 
but you don't need to manage nearly as much messaging infrastructure to achieve your intended results.


### Example Code
- The __main.cpp__ file in the root folder of the project contains a default implementation that uses the ServoManager
and servo abstraction layer to control the hand. This is the easiest way to get started. You can basically start editing
the code to make your own controls.
- In __examples/native_messages.cpp__ you can see how to directly subscribe to the message handlers and how to isse commands
directly to the DexHand. This is suitable for applications where you might be controlling the hand at a low-level high frequency
manner such as controlling it with an AI model or something like that.




