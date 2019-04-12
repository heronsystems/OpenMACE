#ifndef COST_H
#define COST_H

namespace mace {
namespace math {

class Cost
{
public:
    explicit Cost(const double &cost = 0.0):
        m_cost(cost)
    {
    }

    void setCost(const double &cost)
    {
        this->m_cost = cost;
    }

    double getCost() const
    {
        return m_cost;
    }

private:
    /** \brief The value of the cost */
    double m_cost;

};

} //end of namespace math
} //end of namespace mace

#endif // COST_H
