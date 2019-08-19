#ifndef COLLECT_AND_TIMEOUT_TYPE_H
#define COLLECT_AND_TIMEOUT_TYPE_H

class CollectAfterTimeout
{
public:

    CollectAfterTimeout(int numMS){
        this->numMS = numMS;
    }

    int numMS;

};

#endif // COLLECT_AND_TIMEOUT_TYPE_H
