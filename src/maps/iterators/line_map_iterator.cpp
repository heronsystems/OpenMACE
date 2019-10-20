#include "line_map_iterator.h"

namespace mace{
namespace maps{

LineMapIterator::LineMapIterator(const maps::BaseGridMap *map, const pose::CartesianPosition_2D &start, const pose::CartesianPosition_2D &end)
{
    //Ken: In the future we need to check if its out of bounds
    it = new GenericMapIterator(map);
    it->setStartIndex(map->indexFromPos(start.getXPosition(), start.getYPosition()));
    it->setEndIndex(map->indexFromPos(end.getXPosition(), end.getYPosition()));
    initializeIterationParameters();
}

LineMapIterator::LineMapIterator(const LineMapIterator *copy)
{
    this->it = new GenericMapIterator(copy->it->parentMap);
    this->m_StartIndex = copy->m_StartIndex;
    this->m_EndIndex = copy->m_EndIndex;
    this->m_Increment1 = copy->m_Increment1;
    this->m_Increment2 = copy->m_Increment2;
    this->m_Numerator = copy->m_Numerator;
    this->m_Denominator = copy->m_Denominator;
    this->m_IterateNumerator = copy->m_IterateNumerator;
    this->m_CurrentCellCount = copy->m_CurrentCellCount;
    this->m_NumCells = copy->m_NumCells;
}

void LineMapIterator::initializeIterationParameters()
{
    m_CurrentCellCount = 0;
    //First find the index positions related to the row/column definitions of the vectorized buffer
    int startIndexX, startIndexY;
    int endIndexX, endIndexY;

    it->parentMap->getIndexDecomposed(it->getStartIndex(),startIndexX, startIndexY);
    it->parentMap->getIndexDecomposed(it->getEndIndex(),endIndexX, endIndexY);
    unsigned int deltaX = std::abs<int>(endIndexX - startIndexX);
    unsigned int deltaY = std::abs<int>(endIndexY - startIndexY); //in this case we are formulating our slope

    if(endIndexX >= startIndexX) //x is increasing in a positive fashion
    {
        m_Increment1(0) = 1;
        m_Increment2(0) = 1;
    }
    else //x is decreasing in a negative fashion
    {
        m_Increment1(0) = -1;
        m_Increment2(0) = -1;
    }

    if(endIndexY >= startIndexY) //y is increasing in a positive fashion
    {
        m_Increment1(0) = 1;
        m_Increment2(0) = 1;
    }
    else //y is decreasing in a negative fashion
    {
        m_Increment1(0) = -1;
        m_Increment2(0) = -1;
    }

    if (deltaX >= deltaY) {
      // There is at least one x-value for every y-value.
      m_Increment1.x() = 0; // Do not change the x when numerator >= denominator.
      m_Increment2.y() = 0; // Do not change the y for every iteration.
      m_Denominator = deltaX;
      m_Numerator = deltaX / 2;
      m_IterateNumerator = delta.y();
      m_NumCells = deltaX + 1; // There are more x-values than y-values.
    } else {
      // There is at least one y-value for every x-value
      m_Increment2.x() = 0; // Do not change the x for every iteration.
      m_Increment1.y() = 0; // Do not change the y when numerator >= denominator.
      m_Denominator = deltaY;
      m_Numerator = deltaY / 2;
      m_IterateNumerator = deltaX;
      m_NumCells = deltaY + 1; // There are more y-values than x-values.
    }
}

LineMapIterator LineMapIterator::begin() const
{
    LineMapIterator newIT(this);
    newIT.it->setCurrentIndex(newIT.it->getStartIndex());
    return newIT;
}

LineMapIterator LineMapIterator::end() const
{
    LineMapIterator newIT(this);
    newIT.it->setCurrentIndex(newIT.it->getEndIndex());
    return newIT;
}

bool LineMapIterator::isPastEnd() const
{
    return m_CurrentCellCount > m_NumCells;
}

LineMapIterator& LineMapIterator::operator =(const LineMapIterator& rhs)
{
    this->it = new GenericMapIterator(rhs.it->parentMap);
    this->m_StartIndex = rhs.m_StartIndex;
    this->m_EndIndex = rhs.m_EndIndex;
    this->m_Increment1 = rhs.m_Increment1;
    this->m_Increment2 = rhs.m_Increment2;
    this->m_Numerator = rhs.m_Numerator;
    this->m_Denominator = rhs.m_Denominator;
    this->m_IterateNumerator = rhs.m_IterateNumerator;
    this->m_CurrentCellCount = rhs.m_CurrentCellCount;
    this->m_NumCells = rhs.m_NumCells;
    return *this;
}

bool LineMapIterator::operator == (const LineMapIterator &rhs) const
{
    if(this->it != rhs.it){
        return false;
    }
    if(this->m_StartIndex != rhs.m_StartIndex){
        return false;
    }
    if(this->m_EndIndex != rhs.m_EndIndex){
        return false;
    }
    if(this->m_Increment1 != rhs.m_Increment1){
        return false;
    }
    if(this->m_Increment2 != rhs.m_Increment2){
        return false;
    }
    if(this->m_Numerator != rhs.m_Numerator){
        return false;
    }
    if(this->m_Denominator != rhs.m_Denominator){
        return false;
    }
    if(this->m_IterateNumerator != rhs.m_IterateNumerator){
        return false;
    }
    if(this->m_CurrentCellCount != rhs.m_CurrentCellCount){
        return false;
    }
    if(this->m_NumCells != rhs.m_NumCells){
        return false;
    }

    return true;
}

bool LineMapIterator::operator != (const LineMapIterator &rhs) const
{
    return !(*this == rhs);
}


LineMapIterator& LineMapIterator::operator ++()
{
    if(!it->isPastEnd()){
        m_Numerator += m_IterateNumerator;
        int currentIndex, cuerrentIndexX, cuerrentIndexY;
        if (m_Numerator >= m_Denominator) {
          m_Numerator -= m_Denominator;
          it->parentMap->getIndexDecomposed(it->getCurrentIndex(),cuerrentIndexX, cuerrentIndexY);
          cuerrentIndexX += m_Increment1(0); cuerrentIndexY += m_Increment1(1);
          it->parentMap->getVectorIndex(currentIndex, cuerrentIndexX, cuerrentIndexY);
          it->setCurrentIndex(currentIndex);
        }
        it->parentMap->getIndexDecomposed(it->getCurrentIndex(),cuerrentIndexX, cuerrentIndexY);
        cuerrentIndexX += m_Increment2(0); cuerrentIndexY += m_Increment2(1);
        ++m_CurrentCellCount;
    }

    return *this;
}

LineMapIterator LineMapIterator::operator ++(int)
{
    LineMapIterator old(*this);
    ++(*this);
    return old;
}

int LineMapIterator::operator *() const
{
    return *(*it);
}

void LineMapIterator::findValidStartIndex()
{

}

} //end of namespace maps
} //end of namespace mace
