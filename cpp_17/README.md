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
