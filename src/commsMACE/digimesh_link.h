#ifndef DIGIMESH_LINK_H
#define DIGIMESH_LINK_H


#include "commsmace_global.h"

#include <chrono>
#include <mutex>
#include <iostream>
#include <thread>

#include <QCoreApplication>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QThread>

#include "digimesh_configuration.h"

#include "i_link_mace.h"

#include "MACEDigiMeshWrapper/mace_digimesh_wrapper.h"



namespace CommsMACE
{

extern char MACE_INSTANCE_STR[];
extern char VEHICLE_STR[];
extern char GROUNDSTATION_STR[];
extern char MLSTATION_STR[];
extern char RTA_STR[];
extern char EXTERNAL_LINK_STR[];

class COMMSMACESHARED_EXPORT DigiMeshLink : public ILink
{
public:

    DigiMeshLink(const DigiMeshConfiguration &config);

    ~DigiMeshLink();

    virtual void RequestReset();

    virtual void WriteBytes(const char *bytes, int length, const OptionalParameter<Resource> &target = OptionalParameter<Resource>());

    virtual void AddResource(const Resource &resource);

    virtual bool HasResource(const Resource &resource) const;

    virtual void RequestRemoteResources() const;


    //!
    //! \brief Determine the connection status
    //! \return True if the connection is established, false otherwise
    //!
    virtual bool isConnected() const;


    std::string getPortName() const;


    //!
    //! \brief Get the maximum connection speed for this interface.
    //! \return The nominal data rate of the interface in bit per second, 0 if unknown
    //!
    virtual uint64_t getConnectionSpeed() const;



    virtual bool Connect(void);

    virtual void Disconnect(void);

    virtual void MarshalOnThread(std::function<void()> func){
        func();
    }

private:

private:



private:
    DigiMeshConfiguration _config;

    MACEDigiMeshWrapper<MACE_INSTANCE_STR, VEHICLE_STR, GROUNDSTATION_STR, MLSTATION_STR, RTA_STR, EXTERNAL_LINK_STR> *m_Link;
};

} //END MAVLINKComms

#endif // DIGIMESH_LINK_H
