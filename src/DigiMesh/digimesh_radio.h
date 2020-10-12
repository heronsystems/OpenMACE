#ifndef MACE_DIGIMESH_WRAPPER_H
#define MACE_DIGIMESH_WRAPPER_H

#include "DigiMesh_global.h"
#include <vector>
#include <map>
#include <functional>
#include "digi_common/digi_mesh_baud_rates.h"

#include "serial_link.h"

#include "i_link_events.h"

#include "frame-persistance/behaviors/index.h"

#include "ATData/index.h"

#include "math_helper.h"
#include "callback.h"

#include "common/common.h"


#define START_BYTE 0x7e
#define BROADCAST_ADDRESS 0x000000000000ffff
// command types
#define FRAME_AT_COMMAND 0x08
#define FRAME_AT_COMMAND_RESPONSE 0x88
#define FRAME_REMOTE_AT_COMMAND 0x17
#define FRAME_REMOTE_AT_COMMAND_RESPONSE 0x97
#define FRAME_MODEM_STATUS 0x8a
#define LEGACY_TX_STATUS 0x89
#define FRAME_TRANSMIT_REQUEST 0x10
#define FRAME_TRANSMIT_STATUS 0x8b
#define FRAME_RECEIVE_PACKET 0x90
#define FRAME_EXPLICIT_RECEIVE_PACKET 0x91
#define CALLBACK_QUEUE_SIZE 256

class DIGIMESHSHARED_EXPORT DigiMeshRadio : public ILinkEvents
{
private:

    struct Frame{
        std::shared_ptr<FramePersistanceBehavior<>> framePersistance;
        bool inUse;
    };

    SerialLink *m_Link;

    std::vector<int> m_OwnVehicles;
    std::map<int, int> m_RemoteVehiclesToAddress;

    std::function<void(const int)> m_NewVehicleCallback;
    std::function<void(const std::vector<uint8_t> &)> m_NewDataCallback;

    Frame *m_CurrentFrames;
    int m_PreviousFrame;
    std::mutex m_FrameSelectionMutex;

    std::vector<uint8_t> m_CurrBuf;
    std::mutex m_CurrBuffMutex;

    std::vector<std::function<void(const ATData::Message&)>> m_MessageHandlers;

public:
    DigiMeshRadio(const std::string &commPort, const DigiMeshBaudRates &baudRate);

    ~DigiMeshRadio();

    /**
     * @brief SetOnNewVehicleCallback
     * Set lambda to be called when a new vehicle is discovered by DigiMesh
     * @param func lambda to call.
     */
    void SetOnNewVehicleCallback(std::function<void(const int)> func);


    /**
     * @brief SetNewDataCallback
     * Set callback to be notified when new data has been transmitted to this node
     * @param func Function to call upon new data
     */
    void SetNewDataCallback(std::function<void(const std::vector<uint8_t> &)> func);

    void AddMessageHandler(const std::function<void(const ATData::Message&)> &lambda)
    {
        m_MessageHandlers.push_back(lambda);
    }


    template <typename T, typename P>
    void GetATParameterAsync(const std::string &parameterName, const std::function<void(const std::vector<T> &)> &callback, const P &persistance = P())
    {
        static_assert(std::is_base_of<ATData::IATData, T>::value, "T must be a descendant of ATDATA::IATDATA");

        std::shared_ptr<FramePersistanceBehavior<P>> frameBehavior = std::make_shared<FramePersistanceBehavior<P>>(persistance);
        ((FramePersistanceBehavior<>*)frameBehavior.get())->setCallback<T>(callback);

//        int frame_id = AT_command_helper(parameterName, frameBehavior);
        AT_command_helper(parameterName, frameBehavior);
    }


    template <typename T, typename P>
    std::vector<T> GetATParameterSync(const std::string &parameterName, const P &persistance = P())
    {
        std::mutex locker;
        locker.lock();

        std::vector<T> rtn;
        GetATParameterAsync<T, P>(parameterName, [&rtn, &locker](const std::vector<T> &data){
            rtn = data;
            locker.unlock();
        }, persistance);

        locker.lock();
        return rtn;
    }

    template <typename T>
    void SetATParameterSync(const std::string &parameterName, const T &value){

        //create mutex and lock it
        std::mutex waitMutex;
        waitMutex.lock();

        //call function, unlock mutex in callback
        SetATParameterAsync<T>(parameterName, value, [&waitMutex](){waitMutex.unlock();});

        //lock mutex again, blocking this until above is called
        waitMutex.lock();
    }

    template <typename T>
    void SetATParameterAsync(const std::string &parameterName, const T &value, const std::function<void()> &cb= [](){})
    {
        static_assert(std::is_base_of<ATData::IATData, T>::value, "T must be a descendant of ATDATA::IATDATA");

        SetATParameterAsync<T, ATData::Void>(parameterName, value, [cb](const std::vector<ATData::Void>&){cb();});
    }


    template <typename T, typename RT>
    void SetATParameterAsync(const std::string &parameterName, const T &value, const std::function<void(const std::vector<RT> &)> &callback)
    {
        static_assert(std::is_base_of<ATData::IATData, T>::value, "T must be a descendant of ATDATA::IATDATA");

        std::shared_ptr<FramePersistanceBehavior<ShutdownFirstResponse>> frameBehavior = std::make_shared<FramePersistanceBehavior<ShutdownFirstResponse>>(ShutdownFirstResponse());
        ((FramePersistanceBehavior<>*)frameBehavior.get())->setCallback<RT>(callback);

        std::vector<uint8_t> data = value.Serialize();

        AT_command_helper(parameterName, frameBehavior, data);
    }

    void SendMessage(const std::vector<uint8_t> &data)
    {
        std::shared_ptr<FramePersistanceBehavior<ShutdownFirstResponse>> frameBehavior = std::make_shared<FramePersistanceBehavior<ShutdownFirstResponse>>(ShutdownFirstResponse());
        construct_message(data, BROADCAST_ADDRESS, frameBehavior);
    }

    void SendMessage(const std::vector<uint8_t> &data, const uint64_t &addr)
    {
        std::shared_ptr<FramePersistanceBehavior<ShutdownFirstResponse>> frameBehavior = std::make_shared<FramePersistanceBehavior<ShutdownFirstResponse>>(ShutdownFirstResponse());
        construct_message(data, addr, frameBehavior);
    }

    void SendMessage(const std::vector<uint8_t> &data, const uint64_t &addr, const std::function<void(const ATData::TransmitStatus &)> &callback)
    {
        std::shared_ptr<FramePersistanceBehavior<ShutdownFirstResponse>> frameBehavior = std::make_shared<FramePersistanceBehavior<ShutdownFirstResponse>>(ShutdownFirstResponse());
        ((FramePersistanceBehavior<>*)frameBehavior.get())->setCallback<ATData::TransmitStatus>([callback](const std::vector<ATData::TransmitStatus> &arr)
        {
            callback(arr.at(0));
        });

        construct_message(data, addr, frameBehavior);
    }



private:


    void construct_message(const std::vector<uint8_t> &data, const uint64_t &addr, const std::shared_ptr<FramePersistanceBehavior<ShutdownFirstResponse>> &frameBehavior)
    {
        int packet_length = 14 + data.size();
        int total_length = packet_length+4;
        char *tx_buf = new char[total_length];

        int frame_id = 0;
        if(frameBehavior->HasCallback())
        {
            frame_id = reserve_next_frame_id();
        }
        if(frame_id == -1)
        {
            throw std::runtime_error("Digimesh frame could not be established. Communication rate is likely too high for network to handle");
        }

        tx_buf[0] = START_BYTE;
        tx_buf[1] = (packet_length >> 8) & 0xFF;
        tx_buf[2] = packet_length & 0xFF;
        tx_buf[3] = FRAME_TRANSMIT_REQUEST;
        tx_buf[4] = frame_id;
        for(size_t i = 0 ; i < 8 ; i++) {
            uint64_t a = (addr & (0xFFll << (8*(7-i)))) >> (8*(7-i));
            tx_buf[5+i] = (char)a;
        }
        tx_buf[13] = 0xFF;
        tx_buf[14] = 0xFe;
        tx_buf[15] = 0x00;
        tx_buf[16] = 0x00;
        for(size_t i = 0 ; i < data.size() ; i++) {
            tx_buf[17+i] = data.at(i);
        }
        tx_buf[total_length-1] = MathHelper::calc_checksum(tx_buf, 3, total_length-1);

        frameBehavior->setFinishBehavior([this, frame_id](){
            m_CurrentFrames[frame_id].inUse = false;
        });

        m_Link->MarshalOnThread([this, tx_buf, total_length, frame_id, frameBehavior](){
            m_CurrentFrames[frame_id].framePersistance = frameBehavior;
            m_Link->WriteBytes(tx_buf, total_length);

            delete[] tx_buf;
        });
    }

    virtual void ReceiveData(SerialLink *link_ptr, const std::vector<uint8_t> &buffer);

    virtual void CommunicationError(const SerialLink* link_ptr, const std::string &type, const std::string &msg);

    virtual void CommunicationUpdate(const SerialLink *link_ptr, const std::string &name, const std::string &msg);

    virtual void Connected(const SerialLink* link_ptr);

    virtual void ConnectionRemoved(const SerialLink *link_ptr);

private:

    int AT_command_helper(const std::string &parameterName, const std::shared_ptr<FramePersistanceBehavior<>> &frameBehavior, const std::vector<uint8_t> &data = {})
    {
        int frame_id = reserve_next_frame_id();
        if(frame_id == -1)
        {
            throw std::runtime_error("Digimesh frame could not be established. Communication rate is likely too high for network to handle");
        }

        size_t param_len = data.size();

        char *tx_buf = new char[8 + param_len];
        tx_buf[0] = START_BYTE;
        tx_buf[1] = (0x04 + param_len) >> 8;
        tx_buf[2] = (0x04 + param_len) & 0xff;
        tx_buf[3] = FRAME_AT_COMMAND;
        tx_buf[4] = frame_id;
        tx_buf[5] = parameterName[0];
        tx_buf[6] = parameterName[1];

        // if we have a parameter, copy it over
        if (param_len > 0) {
            for(size_t i = 0 ; i < param_len ; i++) {
                tx_buf[7 + i] = data[i];
            }
        }

        tx_buf[7 + param_len] = MathHelper::calc_checksum<char>(tx_buf, 3, 8 + param_len-1);

        frameBehavior->setFinishBehavior([this, frame_id](){
            m_CurrentFrames[frame_id].inUse = false;
        });

        //console.log(tx_buf.toString('hex').replace(/(.{2})/g, "$1 "));
        m_Link->MarshalOnThread([this, tx_buf, param_len, frame_id, frameBehavior](){
            m_CurrentFrames[frame_id].framePersistance = frameBehavior;
            m_Link->WriteBytes(tx_buf, 8 + param_len);

            delete[] tx_buf;
        });

        return frame_id;
    }

    void handle_AT_command_response(const std::vector<uint8_t> &buff);

    void handle_transmit_status(const std::vector<uint8_t> &data);

    void handle_legacy_transmit_status(const std::vector<uint8_t> &data);

    void handle_receive_packet(const std::vector<uint8_t> &, const bool &explicitFrame = false);

    int reserve_next_frame_id();

    void find_and_invokve_frame(int frame_id, const std::vector<uint8_t> &data)
    {
        if(this->m_CurrentFrames[frame_id].inUse == true && this->m_CurrentFrames[frame_id].framePersistance != nullptr && this->m_CurrentFrames[frame_id].framePersistance->HasCallback() == true) {
            this->m_CurrentFrames[frame_id].framePersistance->AddFrameReturn(frame_id, data);
        }
    }


    void finish_frame(int frame_id);


};

#endif // MACE_DIGIMESH_WRAPPER_H
