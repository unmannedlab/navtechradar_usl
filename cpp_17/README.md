# C++17 Implementation of the IASDK

This folder contains a C++17 standard version of the IASDK

This version of the SDK was developed on Ubuntu 20.04

```shell
sudo apt install build-essential clang g++ protobuf-compiler libprotobuf-dev cmake
```

## Notes

Preferred compiler for bulding is Clang V10 but GCC 9.3.x should work

The code uses using statments to make pointer ownership more obvious:

* Owner_of is a std::unique_ptr;
* Shared_owner is a std::shared_ptr;

Please see utility/Pointer_types.h for a full explanation.

## Peak Finding

The class Peak_finder can be used to process FFT data and search for peaks.
The algorithm will sub-resolve within radar bins and return a power at a distance in metres on the azmith being checked.

The algorithm implemented here will slide a window over the FFT data moving forwards by the size of the window, when the FFT has risen and then fallen, the peak resolving algorithm is run to sub-resolve the distance.

See Peak_finder.h for the data structure that is generated per azimuth

The navigation_main.cpp is a sample application that will peak search and report back upto ten targets per azimuth.

* threshold - Threshold in dB
* bins_to_operate_on - Radar bins window size to search for peaks in
* start_bin - Start Bin
* buffer_mode - Buffer mode should only be used with a staring radar
* buffer_length - Buffer Length
* max_peaks_per_azimuth - Maximum number of peaks to find in a single azimuth
