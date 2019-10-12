#include "queue.hpp"

namespace levelset {

template <class Element>
PriorityQueue<Element>::PriorityQueue(unsigned _size, double _inc_max)
    : m_size(_size), m_t0(0), m_i0(0), m_nb_elem(0),m_inc_max(_inc_max)
{
    m_tab   = new ElementList[_size];
    m_delta = _inc_max / static_cast<double>(_size);
}

template <class Element>
PriorityQueue<Element>::~PriorityQueue()
{
    delete[] m_tab;
}


// Access/Modif the queue


template <class Element>
bool PriorityQueue<Element>::empty() const
{
    return (m_nb_elem == 0);
}

template <class Element>
int PriorityQueue<Element>::push(const Element &e, double t)
{
    int i = 0;

    if (empty()) {
        m_t0    = t;
        m_i0    = 0;
    }
    else {

        // test to prevent errors
        if (t-m_t0 > m_inc_max) {
            std::cerr << "Error : Increment=" << t-m_t0 << " >> Inc max" << m_inc_max << std::endl;
            std::cerr << "        e=" << e << " , t=" << t << " , m_t0=" << m_t0 << std::endl;
            //print();
            //TODO laisse passer le prog kan mm en cas de bug.
            //exit(2);
        }

        // compute the indice in the circular array.
        i = static_cast<int>(floor((t-m_t0) * m_size / m_inc_max));

        // compute the true indice in the regular array.
        if (i >= 0){
            i = (i >= m_size)? m_size-1 : i;
            i = (i + m_i0) % m_size;
        }
        else{
            // if t < m_t0, compute new values for m_i0, m_t0, and verify that the end of the circular array is empty of at least |i| bucket.
            while (i++ < 0){

                m_i0 = (m_i0-1 >= 0)? m_i0-1 : m_i0-1+m_size;
                m_t0 -= m_delta;

                if (!m_tab[m_i0].empty()) {
                    std::cerr << "Error : While shifting the begining of the queue, circular array not empty at the end" << std::endl;
                    std::cerr << "        e=" << e << " , t=" << t << std::endl;
                    //print();
                    // TODO laisse passer le prog kan mm en cas de bug, insere le point ds le premier bucket en  premiere position, meme si n y appartien pas vraiment.
                    m_i0 = (m_i0+1) % m_size;
                    m_t0 += m_delta;
                    break;
                    //exit(2);
                }
            }
            i = m_i0;
        }
    }

    m_nb_elem++;
    m_tab[i].push_front(e);

    return i;
}

template <class Element>
void PriorityQueue<Element>::pop()
{
    if (empty()) return;

    // shift m_i0 in order to be on a non empty level of quantization
    while (m_tab[m_i0].empty()){
        m_i0 = (m_i0 + 1) % m_size;
        m_t0 += m_delta;
    }

    m_nb_elem--;
    m_tab[m_i0].pop_back();

}

template <class Element>
const Element & PriorityQueue<Element>::top()
{
    // shift m_i0 in order to be on a non empty level of quantization
    while (m_tab[m_i0].empty()){
        m_i0 = (m_i0 + 1) % m_size;
        m_t0 += m_delta;
    }

    return m_tab[m_i0].back();
}

template <class Element>
int PriorityQueue<Element>::increase_priority(const Element& e, int bucket, double t_new)
{
    if (fabs(t_new-m_t0) > m_inc_max) {
        std::cerr << "Error : Increment=" << t_new-m_t0 << " >> Inc max" << m_inc_max << std::endl;
        std::cerr << "        e=" << e << " , t=" << t_new << " , m_t0=" << m_t0 << std::endl;
        //print();
        //TODO laisse passer le prog kan mm en cas de bug.
        //exit(2);
    }

    // compute the indice in the circular array
    int  i = static_cast<int>(floor((t_new-m_t0) * m_size / m_inc_max));

    // compute the true indice in the regular array.
    if (i >= 0){
        i = (i >= m_size)? m_size-1 : i;
        i = (i + m_i0) % m_size;
    }
    else {
        // if t < m_t0, compute new values for m_i0, m_t0, and verify that the end of the circular array is empty of at least |i| bucket.
        while (i++ < 0){

            m_i0 = (m_i0-1 >= 0)? m_i0-1 : m_i0-1+m_size;
            m_t0 -= m_delta;

            if (!m_tab[m_i0].empty()) {
                std::cerr << "Error : While shifting the begining of the queue, circular array not empty at the end" << std::endl;
                std::cerr << "        e=" << e << " , t=" << t_new << std::endl;
                //print();
                // TODO laisse passer le prog kan mm en cas de bug, insere le point ds le premier bucket en  premiere position, meme si n y appartien pas vraiment.
                m_i0 = (m_i0+1) % m_size;
                m_t0 += m_delta;
                break;
                //exit(2);
            }
        }
        i = m_i0;
    }

    m_tab[bucket].remove(e);
    m_tab[i].push_front(e);

    return i;
}

template <class Element>
void PriorityQueue<Element>::clear()
{
    m_nb_elem = 0;
    m_t0      = 0;
    m_i0      = 0;
    for (int i=0 ; i<m_size ; i++)
        m_tab[i].clear();
}

template <class Element>
unsigned PriorityQueue<Element>::size() const
{
    return m_nb_elem;
}

template <class Element>
void PriorityQueue<Element>::print() const
{
    std::cout << "-------------------------------------"  << std::endl;
    std::cout << "m_t0 = " << m_t0 << std::endl;
    std::cout << "m_i0 = " << m_i0 << std::endl;
    std::cout << "size = " << m_nb_elem << std::endl;
    for (int i = 0; i < m_size; i++){
        if (!m_tab[i].empty()){
            // index in the circular array, starting at m_i0
            int j = (i-(int)m_i0 >= 0)? i-(int)m_i0 : i-(int)m_i0+m_size;

            // print infos concerning the bucket
            std::cout << "  i=" << std::setw(2) << i
                      << "| l=" << std::setw(2) << j
                      << "| p=" << std::setw(5) << m_t0 + j*m_delta
                      << "->"   << std::setw(5) << m_t0 + j*m_delta + m_delta;
            // print the elements in the bucket
            for (typename ElementList::iterator it = m_tab[i].begin() ; it!= m_tab[i].end() ; it++)
                std::cout << " " << *it ;

            std::cout << std::endl;
        }
    }
}


} //end of namespace levelset
