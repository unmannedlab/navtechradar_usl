#ifndef VECTOR_MATHS_H
#define VECTOR_MATHS_H

#include <cstring>
#include <vector>

namespace Navtech {

    namespace Vector_maths {

        template<class T>
        static inline void scalar_sum(std::vector<T>& data, std::size_t length, T& result)
        {
            for (auto index = 0u; index < length; index++)
                result += data[index];
        }

        template<class T>
        static inline void scalar_sum(T* data, std::size_t length, T& result)
        {
            for (auto index = 0u; index < length; index++)
                result += data[index];
        }

        template<class T>
        static inline void scalar_square(std::vector<T>& data, std::size_t length, T& result)
        {
            for (auto index = 0u; index < length; index++)
                result += data[index] * data[index];
        }

        template<class T>
        static inline void scalar_square(T* data, std::size_t length, T& result)
        {
            for (auto index = 0u; index < length; index++)
                result += data[index] * data[index];
        }

        template<class T>
        static inline void vector_square(std::vector<T>& data, std::size_t length, T* result)
        {
            for (auto index = 0u; index < length; index++)
                result[index] += data[index] * data[index];
        }

        template<class T>
        static inline void vector_square(T* data, std::size_t length, T* result)
        {
            for (auto index = 0u; index < length; index++)
                result[index] += data[index] * data[index];
        }

        template<class T>
        static inline void vector_cube(T* data, std::size_t length, T* result)
        {
            for (auto index = 0u; index < length; index++)
                result[index] += data[index] * data[index] * data[index];
        }

        template<class T>
        static inline void vector_cube(std::vector<T>& data, std::size_t length, T* result)
        {
            for (auto index = 0u; index < length; index++)
                result[index] += data[index] * data[index] * data[index];
        }

        template<class T>
        static inline void vector_multiply(T* data1, T* data2, std::size_t length, T* result)
        {
            for (auto index = 0u; index < length; index++)
                result[index] += data1[index] * data2[index];
        }

        template<class T>
        static inline void vector_multiply(std::vector<T>& data1, std::vector<T>& data2, std::size_t length, T* result)
        {
            for (auto index = 0u; index < length; index++)
                result[index] += data1[index] * data2[index];
        }
    } // namespace Vector_maths

} // namespace Navtech

#endif // VECTOR_MATHS_H
