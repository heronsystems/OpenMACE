#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H

#include <list>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace levelset {


template <class Element>
class PriorityQueue
{
public:
    typedef std::list<Element>               ElementList;


    PriorityQueue(unsigned _size    = 1000,
                  double   _inc_max = 2
            );

    ~PriorityQueue();



    bool empty() const;
    int push(const Element &e, double t);
    void pop();
    const Element & top();
    int increase_priority(const Element& e, int bucket, double t_new);
    void clear();
    unsigned size() const;

    void print() const;

private:

    ElementList *    m_tab;
    unsigned         m_size;
    unsigned         m_nb_elem;
    double           m_t0;
    int              m_i0;
    double           m_delta;
    double           m_inc_max;
};

} //end of namespace levelset

#endif // PRIORITY_QUEUE_H
