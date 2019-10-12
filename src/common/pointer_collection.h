#ifndef POINTER_COLLECTION_H
#define POINTER_COLLECTION_H

#include <vector>

template <typename ...T>
class _PointerCollection;

template <typename Head, typename ...Tail>
class _PointerCollection<Head, Tail...> : public _PointerCollection<Tail...>
{
public:

    using _PointerCollection<Tail...>::Set;
    using _PointerCollection<Tail...>::Get;

private:

    Head *m_ptr;

public:
    _PointerCollection<Head, Tail...>() :
        _PointerCollection<Tail...>(),
        m_ptr(nullptr)
    {
    }

    void Set(Head* ptr) {
        m_ptr = ptr;
    }

    void Get(Head* &ptr) {
        ptr = m_ptr;
    }

    std::vector<void*> GetAll() {
        std::vector<void*> list = _PointerCollection<Tail...>::GetAll();
        list.push_back((void*)m_ptr);
        return list;
    }
};


template <>
class _PointerCollection<>
{
public:
    void Set() {
    }

    void Get() {
    }

    std::vector<void*> GetAll() {
        return {};
    }
};


template <typename ...TT>
class PointerCollection : public _PointerCollection<TT...>
{
private:

public:
    template<typename T>
    void Add(T* ptr)
    {
        this->Set(ptr);
    }


    template <typename T>
    T* Retreive()
    {
        T* ptr;
        this->Get(ptr);
        return ptr;
    }

    template<typename T>
    void ForEach(std::function<void(T* ptr)> func)
    {
        std::vector<void*> list = this->GetAll();
        for(auto it = list.cbegin() ; it != list.cend() ; ++it)
        {
            if(*it == nullptr)
            {
                continue;
            }
            func((T*)*it);
        }
    }
};

#endif // POINTER_COLLECTION_H
