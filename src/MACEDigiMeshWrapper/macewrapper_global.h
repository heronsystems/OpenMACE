#ifndef MACEWRAPPER_GLOBAL_H
#define MACEWRAPPER_GLOBAL_H

#ifdef _MSC_VER
#if defined(MACEDIGIMESHWRAPPER_LIBRARY)
#  define MACEWRAPPERSHARED_EXPORT __declspec(dllexport)
#else
#  define MACEWRAPPERSHARED_EXPORT __declspec(dllimport)
#endif
#else
#  define MACEWRAPPERSHARED_EXPORT
#endif

#include <vector>
#include <functional>

template <typename ...T>
void Notify(std::vector<std::function<void(T...)>> lambdas, T... args)
{
    for(auto it = lambdas.cbegin() ; it != lambdas.cend() ; ++it) {
        (*it)(args...);
    }
}


template<const char* T>
void variadicExpand(std::function<void(const char* element)> lambda)
{
    lambda(T);
}

template<const char* Head, const char* Next, const char* ...Tail>
void variadicExpand(std::function<void(const char* element)> lambda)
{
    lambda(Head);
    variadicExpand<Next, Tail...>(lambda);
}

#endif // MACEWRAPPER_GLOBAL_H
