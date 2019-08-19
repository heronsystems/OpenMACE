#include "digimesh_radio.h"

#include "serial_configuration.h"

#include <iostream>


DigiMeshRadio::DigiMeshRadio(const std::string &commPort, const DigiMeshBaudRates &baudRate) :
    m_Link(NULL)
{
    m_CurrentFrames = new Frame[CALLBACK_QUEUE_SIZE];
    for(int i = 0 ; i < CALLBACK_QUEUE_SIZE ; i++) {
        m_CurrentFrames[i].inUse = false;
    }
    m_PreviousFrame = 0;

    SerialConfiguration config;
    config.setBaud(baudRate);
    config.setPortName(commPort);
    config.setDataBits(8);
    config.setParity(QSerialPort::NoParity);
    config.setStopBits(1);
    config.setFlowControl(QSerialPort::NoFlowControl);

    m_Link = new SerialLink(config);
    m_Link->Connect();

    m_Link->AddListener(this);
}

DigiMeshRadio::~DigiMeshRadio() {
    delete[] m_CurrentFrames;

    if(m_Link != NULL) {
        delete m_Link;
    }
}

/**
 * @brief SetOnNewVehicleCallback
 * Set lambda to be called when a new vehicle is discovered by DigiMesh
 * @param func lambda to call.
 */
void DigiMeshRadio::SetOnNewVehicleCallback(std::function<void(const int)> func)
{
    this->m_NewVehicleCallback = func;
}


/**
 * @brief SetNewDataCallback
 * Set callback to be notified when new data has been transmitted to this node
 * @param func Function to call upon new data
 */
void DigiMeshRadio::SetNewDataCallback(std::function<void(const std::vector<uint8_t> &)> func)
{
    this->m_NewDataCallback = func;
}


void DigiMeshRadio::ReceiveData(SerialLink *link_ptr, const std::vector<uint8_t> &buffer)
{
    //add what we received to the current buffer.

    m_CurrBuffMutex.lock();
    for(size_t i = 0 ; i < buffer.size() ; i++) {
        m_CurrBuf.push_back(buffer.at(i));
    }
    m_CurrBuffMutex.unlock();


    //start infinite loop to pick up multiple packets sent at same time
    while(true) {

        m_CurrBuffMutex.lock();

        //if there aren't three bytes received we have to wait more
        if(m_CurrBuf.size() < 3) {
            m_CurrBuffMutex.unlock();
            break;
        }


        if(m_CurrBuf.at(0) != 0x7E) {
            m_CurrBuf.erase(m_CurrBuf.begin());
            m_CurrBuffMutex.unlock();
            continue;
        }



        // add 4 bytes for start, length, and checksum
        uint16_t packet_length = (((uint16_t)m_CurrBuf[1])<<8 | (uint16_t)m_CurrBuf[2]) + 4;

        //if we have received part of the packet, but not entire thing then we need to wait longer
        if(m_CurrBuf.size() < packet_length) {
            m_CurrBuffMutex.unlock();
            return;
        }

        //splice m_CurrBuff to just our packet we care about.
        std::vector<uint8_t> packet(
            std::make_move_iterator(m_CurrBuf.begin() + 3),
            std::make_move_iterator(m_CurrBuf.begin() + packet_length - 1));
        uint8_t checksum = m_CurrBuf.at(packet_length -1);

        /*
        if(packet[0] == LEGACY_TX_STATUS)
        {
            for(int i = 0 ; i < m_CurrBuf.size() ; i++)
            {
                printf("%x\n", m_CurrBuf.at(i));
            }
            printf("Dumped\n");
        }
        */

        m_CurrBuf.erase(m_CurrBuf.begin(), m_CurrBuf.begin() + packet_length);



        m_CurrBuffMutex.unlock();

        //check the checksum
        uint8_t dataCheck = checksum;
        for(auto it = packet.cbegin() ; it != packet.cend() ; ++it)
        {
            dataCheck += *it;
        }
        if(dataCheck != 0xFF)
        {
            printf("Digimesh Checksum Failed! Ignoring packet.\n");
            continue;
        }

        switch(packet[0])
        {
            case FRAME_AT_COMMAND_RESPONSE:
                handle_AT_command_response(packet);
                break;
            case FRAME_REMOTE_AT_COMMAND_RESPONSE:
                break;
            case FRAME_MODEM_STATUS:
                break;
            case FRAME_TRANSMIT_STATUS:
                handle_transmit_status(packet);
                break;
            case LEGACY_TX_STATUS:
                handle_legacy_transmit_status(packet);
                break;
            case FRAME_RECEIVE_PACKET:
                handle_receive_packet(packet);
                break;
            case FRAME_EXPLICIT_RECEIVE_PACKET:
                handle_receive_packet(packet, true);
                break;
        default:
            throw std::runtime_error("unknown packet type received: " + std::to_string(packet[0]));
        }



    }
}

void DigiMeshRadio::CommunicationError(const SerialLink* link_ptr, const std::string &type, const std::string &msg)
{

}

void DigiMeshRadio::CommunicationUpdate(const SerialLink *link_ptr, const std::string &name, const std::string &msg)
{

}

void DigiMeshRadio::Connected(const SerialLink* link_ptr)
{

}

void DigiMeshRadio::ConnectionRemoved(const SerialLink *link_ptr)
{

}


void DigiMeshRadio::handle_AT_command_response(const std::vector<uint8_t> &buf) {
    uint8_t frame_id = buf[1];

    uint8_t status = buf[4];
    if(status) {
        //error
    }

    std::string commandRespondingTo = "";
    commandRespondingTo.push_back(buf[2]);
    commandRespondingTo.push_back(buf[3]);

    /*
    std::string str = "";
    for(size_t i = 5 ; i < buf.size()-1 ; i++) {
        str += buf[i];
    }
    */

    std::vector<uint8_t> packet(
        std::make_move_iterator(buf.begin() + 5),
        std::make_move_iterator(buf.end()));

    find_and_invokve_frame(frame_id, packet);

}

void DigiMeshRadio::handle_transmit_status(const std::vector<uint8_t> &data)
{
    uint8_t frame_id = data[1];
    find_and_invokve_frame(frame_id, data);

    if(data[5] == 0x74)
    {
        printf(" PAYLOAD TOO LARGE !!!!\n");
        throw std::runtime_error("Transmit error, Payload too large");
    }
}

void DigiMeshRadio::handle_legacy_transmit_status(const std::vector<uint8_t> &data)
{
    printf("!!!!! LEGACY TRANSMIT STATUS SEEN");
    if(data[2] == 0x74)
    {
        printf(" PAYLOAD TOO LARGE !!!!\n");
    }
}


void DigiMeshRadio::handle_receive_packet(const std::vector<uint8_t> &data, const bool &explicitFrame)
{
    ATData::Message msg(data, explicitFrame);

    for(int i = 0 ; i < m_MessageHandlers.size() ; i++) {
        m_MessageHandlers.at(i)(msg);
    }
}

int DigiMeshRadio::reserve_next_frame_id()
{
    std::lock_guard<std::mutex> lock(m_FrameSelectionMutex);

    // save last id
    int framePrediction = m_PreviousFrame;
    // advance until we find an empty slot
    do {
        framePrediction++;

        //wrap around
        if (framePrediction > CALLBACK_QUEUE_SIZE-1){
            framePrediction = 1;
        }

        // if we've gone all the way around, then we can't send
        if (framePrediction == m_PreviousFrame)
        {
            return -1;
        }
    }
    while (m_CurrentFrames[framePrediction].inUse == true);

    m_CurrentFrames[framePrediction].inUse = true;
    m_PreviousFrame = framePrediction;

    return framePrediction;
}


void DigiMeshRadio::finish_frame(int frame_id)
{
    this->m_CurrentFrames[frame_id].inUse = false;
}
