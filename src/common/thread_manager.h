#ifndef THREADMANAGER_H
#define THREADMANAGER_H
#include <thread>

class Thread {
public:
    Thread() :
        mThread(NULL), mToExit(false)
    {

    }

    virtual ~Thread() {
        stop();
    }

    virtual void run() = 0;

    virtual void start() {
        stop();
        mToExit = false;
        mThread = new std::thread([this]()
        {
            this->run();
        });
    }

    void stop(){
        if(mThread)
        {
            mToExit = true;
            mThread->join();
            delete mThread;
            mThread = NULL;
        }
    }

    bool isThreadActive()
    {
        if(mToExit == false)
        {
            return true;
        }
        return false;
    }
protected:
    std::thread *mThread;
    bool mToExit;

};

#endif // THREADMANAGER_H
