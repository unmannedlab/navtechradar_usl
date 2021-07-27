#ifndef POINTER_TYPES_H
#define POINTER_TYPES_H

#include <memory>

namespace Navtech {
    // ------------------------------------------------------
    // These type aliases provide a more specific declaration
    // for the use of the pointer; particularly with regards
    // to lifetime management of objects.
    //

    // Declares that the lifetime of the object is managed
    // by the class declaring ownership.  Ownership of the
    // object may be transferred (via move()) but cannot be
    // shared.
    // Owner_of should only be used in one of two situations:
    // - The object may be lazy-instantiated -that is, its
    //   lifetime may be significantly shorter than that of
    //   its parent.
    // - The object's may be replaced at run-time; for example
    //   with a sub-type.
    //
    template<typename T>
    using Owner_of = std::unique_ptr<T>;

    template<typename T, typename... Arg_Ty>
    Owner_of<T> make_owned(Arg_Ty&&... args)
    {
        return Owner_of<T> { new T { std::forward<Arg_Ty>(args)... } };
    }


    // Declares that the lifetime of the object is shared
    // by several objects. The last owner to go out of scope
    // destroys the shared object.
    //
    template<typename T>
    using Shared_owner = std::shared_ptr<T>;

    template<typename T, typename... Arg_Ty>
    Shared_owner<T> make_shared_owner(Arg_Ty&&... args)
    {
        return std::make_shared<T>(std::forward<Arg_Ty>(args)...);
    }

    // Declares that the class does NOT manage the lifetime
    // of the object, it simply _uses_ (that is, calls the
    // methods of) the object.
    //
    template<typename T>
    using Association_to = T*;

    template<typename T>
    auto associate_with(T& from)
    {
        return &from;
    }

    template<typename T>
    auto associate_with(T* from)
    {
        return from;
    }

    template<typename T>
    auto associate_with(Shared_owner<T>& from)
    {
        return from.get();
    }

    template<typename T>
    auto associate_with(Owner_of<T>& from)
    {
        return from.get();
    }
} // namespace Navtech

#endif // POINTER_TYPES_H
