#ifndef ENTITIES_H
#define ENTITIES_H

#include "macewrapper_global.h"

#include <string>
#include <map>
#include <unordered_map>
#include <vector>
#include <stdint.h>
#include <mutex>
#include <thread>
#include <functional>

#include "digi_common/transmit_status_types.h"

class Component
{
private:

    std::unordered_map<int, uint64_t> m_VehicleIDToRadioAddr;
    std::mutex m_VehicleIDMutex;

    std::vector<int> m_ContainedVehicles;

public:

    void AddInternalItem(int vehicleID) {

        if(m_VehicleIDToRadioAddr.find(vehicleID) != m_VehicleIDToRadioAddr.cend()){
            throw std::runtime_error("Vehicle of given ID already exists");
        }

        m_VehicleIDMutex.lock();
        m_VehicleIDToRadioAddr.insert({vehicleID, 0x00});
        m_VehicleIDMutex.unlock();

        m_ContainedVehicles.push_back(vehicleID);
    }

    uint64_t GetAddr(int ID)
    {
        return m_VehicleIDToRadioAddr.at(ID);
    }

    bool HasAddr(int ID)
    {
        if(m_VehicleIDToRadioAddr.find(ID) == m_VehicleIDToRadioAddr.cend())
        {
            return false;
        }
        return true;
    }

    std::vector<int> ContainedIDs() const {
        return m_ContainedVehicles;
    }

    bool AddExternalItem(int vehicleID, uint64_t addr) {

        if(m_VehicleIDToRadioAddr.find(vehicleID) == m_VehicleIDToRadioAddr.end()) {
            m_VehicleIDMutex.lock();
            m_VehicleIDToRadioAddr.insert({vehicleID, addr});
            m_VehicleIDMutex.unlock();

            return true;
        }
        else {
            if(m_VehicleIDToRadioAddr[vehicleID] != addr) {
                throw std::runtime_error("Remote Vehicle ID passed to this node already exists");
            }

            return false;
        }
    }


    void RemoveInternalItem(int vehicleID)
    {
        if(m_VehicleIDToRadioAddr.find(vehicleID) == m_VehicleIDToRadioAddr.cend()){
            return;
        }

        m_VehicleIDMutex.lock();
        m_VehicleIDToRadioAddr.erase(m_VehicleIDToRadioAddr.find(vehicleID));
        m_VehicleIDMutex.unlock();

        for(auto it = m_ContainedVehicles.cbegin() ; it != m_ContainedVehicles.cend() ; ++it)
        {
            if(*it == vehicleID)
            {
                m_ContainedVehicles.erase(it);
                break;
            }
        }
    }

    void RemoveExternalItem(int vehicleID)
    {
        if(m_VehicleIDToRadioAddr.find(vehicleID) == m_VehicleIDToRadioAddr.cend()){
            return;
        }

        m_VehicleIDMutex.lock();
        m_VehicleIDToRadioAddr.erase(m_VehicleIDToRadioAddr.find(vehicleID));
        m_VehicleIDMutex.unlock();
    }
};

#endif // ENTITIES_H
