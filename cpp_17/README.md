# C++17 Implementation of the IASDK

This folder contains a C++17 standard version of the IASDK

## Notes

Preferred compiler for bulding is Clang V10 but GCC 9.3.x should work

The code uses using statments to make pointer ownership more obvious:

* Owner_of is a std::unique_ptr;
* Shared_owner is a std::shared_ptr;
