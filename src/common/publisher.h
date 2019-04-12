#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <vector>
#include <functional>
#include "optional_parameter.h"

template <typename Subscriber, typename Criteria = void*>
class Publisher
{
public:

    void AddSubscriber(Subscriber* ptr, OptionalParameter<Criteria> c = OptionalParameter<Criteria>())
    {
        m_Subscribers.push_back(std::make_tuple(ptr, c));
    }


    void Emit(const std::function<void(Subscriber *)> &func) const
    {
        for(std::tuple<Subscriber*, OptionalParameter<Criteria>> sub : m_Subscribers)
        {
            func(std::get<0>(sub));
        }
    }


    void Emit(const std::function<void(Subscriber *)> &func, const Criteria &c) const
    {
        for(const std::tuple<Subscriber*, OptionalParameter<Criteria>> sub : m_Subscribers)
        {
            if(std::get<1>(sub).IsSet() == false)
            {
                throw std::runtime_error("A publisher has attempted to be emitted with criteria, yet no criteria exists for the subscriber");
            }

            if(std::get<1>(sub).Value() == c)
            {
                func(std::get<0>(sub));
            }
        }
    }

private:

    std::vector<std::tuple<Subscriber*, OptionalParameter<Criteria>>> m_Subscribers;
};

#endif // PUBLISHER_H
