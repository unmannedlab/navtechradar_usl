# Navtech IA SDK

The Navtech IA SDK provides a basic interface to communicate with the IA sensor. The SDK provides the source code for C++ and a .NET DLL that can easily be integrated into applications running on Windows and Linux.


The IA sensor operates in two modes:

* native data - Used to receive the raw fft data from the sensor

* plot extraction - Used to receive only points where an object is present

The SDK provides support for both modes, allowing developers to receive either type of data from the sensor and also provides an interface to send control and configuration messages to the sensor.

Communication between the sensor and the software SDK is over Ethernet and utilises a proprietary binary communication protocol called _Colossus Network Protocol_. The SDK abstracts this protocol to avoid having to develop the low-level socket and messaging processing code. The [Colossus Protocol documentation can be found here](https://navtechradar.atlassian.net/wiki/display/PROD/Colossus+Network+Data+Protocol).

## Directory Overview

### .kateproject.d

The project file for the Kate editor

### .vscode

The VSCode workspace is a collection of one or more folders and settings used by VSCode

### cpp

Contains the old (depreciated) version of the IASDK, built using the c++11 standard. No further cahnges will be made to this version of the SDK

### cpp_17

Contains the new version of the IASDK, built using the c++17 standard. This folder is configured to use VS Code Build Task and CMake Extensions to enable fast development setup times.

### csharp

Contains a C# version of the IASDK

### iasdk

Contains the IASDK Visual Studio solution files

### protobuf

Contains the protobuf files used in the IASDK

### ros1

Contains the (now depreciated) ROS1 implementation of the IASDK

### ros2

Contains the new ROS2 implementation of the IASDK

## SDK Requirements

### C++ 11

#### C++11 Compiler
* GCC 4.8 and above
* Clang 3.5 and above
* Visual Studio 2019 (VC++ 2019 runtime libraries)

### C++ 17

#### C++17 Compiler
* GCC 9.x and above
* Clang 10 and above

### Microsoft .NET

.NET 4.8 and above

## Linux Specific Requirements

The SDK uses a number of shell scripts as part of its functionality. To use the shell scripts we require bash on Ubuntu. To ensure the necessary shell facilities are available we recommend executing the following command:
```
sudo dpkg-reconfigure -p critical dash
```

## License

See file `LICENSE.txt` or go to <https://opensource.org/licenses/MIT> for full license details.

## Building the SDK
The SDK is written in C++ and must be compiled prior to use.  The SDK is designed to be compiled by a standard C++ compiler installation; that is, no external libraries (beyond the standard and/or Posix libraries) are required.
The SDK may be built using either:
* Command line
* From Visual Studio Code
* From Visual Studio

To build the SDK outside Visual Studio, your platform must have the CMake tools installed.  To check if CMake is present, run:
```bash
cmake --version
```
### Building from the command line
Cmake can be invoked directly from the command line to build the SDK, as follows:
```
cd <IA SDK install path>/iasdk/build
cmake .
make
```

The build can be removed by running:
```
make clean
```

The build process will generate two executables by default:
```
/iasdk/build/testclient
/iasdk/build/navigationclient
```

### Building from VS Code
The project is configured to work with the [Microsoft CMake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools). 
If the CMake Tools are installed, opening the project in VSCode should detect the make files and configure the project accordingly.  The first time a build is selected you will need to select a compiler _kit_.  This will present a list of possible compiler configurations.


### Using the VS Code build task
The `.vscode` folder contains a build task configuration for invoking CMake.
To build:
* hit ***ctrl-shift-b***
* select ***Build***

(Note: you can also select ***Clean*** as one of the build options)

The build process will (again) generate two executables by default:
```
/iasdk/build/testclient
/iasdk/build/navigationclient
```


### Building from Visual Studio
If the Microsoft C++ compiler (msvc) is used, the SDK provides a Visual Studio solution (.sln) file.

Double-clicking on the .sln file will launch Visual Studio.
In Visual Studio select:

***Build -> Build Solution***

(Alternatively, use ***crtl-shift-b***)

## SDK API Overview

(TODO - add info on callbacks etc)

## C# Radar Client API

The .NET API is based on C#6 (.NET Framework 4.8) and was developed in Visual Studio 2019.
There are two project within the repro:

1. **IASDK** - The API DLL for use within any 3rd party projects to assist with connecting to the radar
1. **TestClient** - This is a very simple console application that runs up, connects to a radar and then displays some information before auto-disconnecting and closing. This provides a simple example of the recommended steps to connect and consume data from the radar.

### Usage of the SDK

The steps involved in connecting and getting data are as follows:

Setup your radar client and hook up the message and connection events:

```C#
_radarTcpClient = new RadarTcpClient();

_radarTcpClient.OnConfigurationData += ConfigurationDataHandler;

_radarTcpClient.OnFftData += FftDataHandler;

_radarTcpClient.OnConnectionChanged += ConnectionChangedHandler;
```

Connect to the radar:
```
_radarTcpClient.Connect("192.168.0.1");
```

On successful connection you will receive a Configuration message with details of the radar's current configuration. So you must have the handler setup before you connect.
```C#
private void ConfigurationDataHandler(object sender, GenericEventArgs<TcpConfigurationDataMessage> configurationMessage)
{
	var rotationHz = configurationMessage.Payload.RotationSpeed / 1000.0	
}
```

Once connected and you have the config data, tell the radar to start sending FFT Data:
```
_radarTcpClient.StartFftData();
```

You must handle incoming FFT Data:
```C#
private static void FftDataHandler(object sender, GenericEventArgs<FftData> fftEventArgs)
{
	var azimuth = fftEventArgs.Payload.Message.Azimuth;	
}
```

When you need to disconnect, firstly stop the FFT Data:
```
_radarTcpClient.StopFftData();
```

Then disconnect:
```
_radarTcpClient.Disconnect();
```

## C++ Radar Client API

The c++ API is based on c++17 and was developed in Visual Studio 2019.
There are two project within the repo:

* **IASDK** - The API DLL for use within any 3rd party projects to assist with connecting to the radar
* **TestClient** - This is a very simple console application that runs up, connects to a radar and then displays some information before auto-disconnecting and closing. This provides a simple example of the recommended steps to connect and consume data from the radar.

### Usage of the SDK

The steps involved in connecting and getting data are as follows:

Setup your radar client and hook up the message and connection events:
	
    radar_client = allocate_owned<Radar_client>("127.0.0.1"_ipv4);

    radar_client->set_configuration_data_callback(configuration_data_handler);
	
	radar_client->set_fft_data_callback(fft_data_handler);

Connect to the radar:

	radar_client->start();


On successful connection you will receive a Configuration message with details of the radar's current configuration. So you must have the handler setup before you connect.

	void configuration_data_handler(const Configuration_data::Pointer& data, const Configuration_data::ProtobufPointer& protobuf_configuration)
	{

	}

Once connected and you have the config data, tell the radar to start sending FFT Data:


	radar_client->start_fft_data();


You must handle incoming FFT Data:

	void fft_data_handler(const Fft_data::Pointer& data)

	{

	}

When you need to disconnect, firstly stop the FFT Data:

	radar_client->stop();

Then unbind the data handlers:

    radar_client->set_fft_data_callback();

    radar_client->set_configuration_data_callback();

Then disconnect:

	radar_client->stop();

