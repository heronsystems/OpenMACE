#ifndef CHAIN_INHERITANCE_H
#define CHAIN_INHERITANCE_H

template <typename ...T>
class ChainInheritance;

template<typename HEAD, typename ...T>
class ChainInheritance<HEAD, T...> : public ChainInheritance<T...>, public HEAD
{

};

template<>
class ChainInheritance<>
{

};

#endif // CHAIN_INHERITANCE_H
