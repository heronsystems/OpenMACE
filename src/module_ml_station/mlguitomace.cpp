#include "mlguitomace.h"

MLGUItoMACE::MLGUItoMACE(const MaceCore::IModuleCommandMLStation* ptrRef) :
    m_sendAddress(QHostAddress::LocalHost),
    m_sendPort(1234),
    goalSpace(nullptr),
    m_goalSampler(nullptr)
{
    m_parent = ptrRef;
    goalSpace = std::make_shared<mace::state_space::Cartesian2DSpace>();
    mace::state_space::Cartesian2DSpaceBounds bounds(-20,20,-10,10);
    goalSpace->setBounds(bounds);

    m_goalSampler = std::make_shared<mace::state_space::Cartesian2DSpace_Sampler>(goalSpace);
}

MLGUItoMACE::MLGUItoMACE(const MaceCore::IModuleCommandMLStation* ptrRef, const QHostAddress &sendAddress, const int &sendPort) :
    m_sendAddress(sendAddress),
    m_sendPort(sendPort)
{
    m_parent = ptrRef;
}

MLGUItoMACE::~MLGUItoMACE() {
}

//!
//! \brief setSendAddress Set the TCP send address for MACE-to-GUI comms
//! \param sendAddress TCP send address
//!
void MLGUItoMACE::setSendAddress(const QHostAddress &sendAddress) {
    m_sendAddress = sendAddress;
}

//!
//! \brief setSendPort Set the TCP send port for MACE-to-GUI comms
//! \param sendPort TCP send port
//!
void MLGUItoMACE::setSendPort(const int &sendPort) {
    m_sendPort = sendPort;
}

//!
//! \brief getEnvironmentBoundary Initiate a request to MACE core for the current environment boundary vertices
//!
void MLGUItoMACE::getEnvironmentBoundary() {
    std::shared_ptr<const MaceCore::MaceData> data = m_parent->getDataObject();
    if (data->EnvironmentBoundarySet()) {
        BoundaryItem::EnvironmentBoundary environmentBoundary = data->GetEnvironmentBoundary();

        QJsonObject json;
        json["message_type"] = QString::fromStdString(MLMessageString(MLMessageTypes::ENVIRONMENT_BOUNDARY));
        json["boundary_name"] = QString::fromStdString(environmentBoundary.getName());
        json["boundary_type"] = QString::fromStdString(environmentBoundary.getType());

        QJsonArray vertices;
        for(auto&& vertex : environmentBoundary.getVertices()) {
            QJsonObject obj;
            obj["lat"] = vertex.getLatitude();
            obj["lng"] = vertex.getLongitude();
            obj["alt"] = 0.0;

            vertices.push_back(obj);
        }

        json["vertices"] = vertices;

        QJsonDocument doc(json);
        bool bytesWritten = writeTCPData(doc.toJson());

        if(!bytesWritten){
            std::cout << "Write environment boundary failed..." << std::endl;
        }
    }
}



//!
//! \brief setEnvironmentVertices GUI command to set new environment boundary vertices
//! \param jsonObj JSON data containing the new environment vertices
//!
void MLGUItoMACE::setEnvironmentVertices(const QJsonDocument &data)
{
    UNUSED(data);
//    BoundaryItem::BoundaryList operationalBoundary;

//    mace::pose::GeodeticPosition_3D origin = m_parent->getDataObject()->GetGlobalOrigin();

//    foreach(const QJsonValue & v, data) {
//        std::cout << "Lat: " << v.toObject().value("lat").toDouble() << " / Lon: " << v.toObject().value("lng").toDouble() << std::endl;
//        double tmpLat = v.toObject().value("lat").toDouble();
//        double tmpLon = v.toObject().value("lng").toDouble();
//        double tmpAlt = v.toObject().value("alt").toDouble();

//        mace::pose::GeodeticPosition_3D vertexGlobal(tmpLat,tmpLon,tmpAlt);
//        mace::pose::CartesianPosition_3D vertexLocal3D;

//        if(origin.isAnyPositionValid()) {
//            mace::pose::DynamicsAid::GlobalPositionToLocal(&origin,&vertexGlobal,&vertexLocal3D);
//            mace::pose::CartesianPosition_2D vertexLocal2D(vertexLocal3D.getXPosition(),vertexLocal3D.getYPosition());

//            operationalBoundary.appendVertexItem(&vertexLocal2D);
//        }
//    }

//    BoundaryItem::BoundaryCharacterisic key(BoundaryItem::BOUNDARYTYPE::OPERATIONAL_FENCE);

//    if(operationalBoundary.getQueueSize() > 0) {
//        m_parent->NotifyListeners([&](MaceCore::IModuleEventsMLStation* ptr) {
//            ptr->Event_SetBoundary(m_parent, key, operationalBoundary);
//        });
//    }

//    // Get and send vertices to the GUI:
//    getEnvironmentBoundary();
}


//!
//! \brief getConnectedVehicles Initiate a request to MACE Core for the list of currently connected vehicles
//!
void MLGUItoMACE::getConnectedVehicles()
{
//    mLogs->debug("Module Ground Station saw a request for getting connected vehicles.");

    std::shared_ptr<const MaceCore::MaceData> macedata = m_parent->getDataObject();
    std::vector<unsigned int> vehicleIDs;
    macedata->GetAvailableVehicles(vehicleIDs);

    QJsonArray ids;
    QJsonArray modes;
    if(vehicleIDs.size() > 0){
        for (const uint& i : vehicleIDs) {
            ids.append((int)i);
            std::string mode;
            macedata->GetVehicleFlightMode(i, mode);
            modes.append(QString::fromStdString(mode));
        }
    }
    else {
        std::cout << "No vehicles currently available" << std::endl;
    }

    QJsonObject connectedVehicles;
    connectedVehicles["ids"] = ids;
    connectedVehicles["modes"] = modes;

    QJsonObject json;
    json["dataType"] = "ConnectedVehicles";
    json["vehicleID"] = 0;
    json["connectedVehicles"] = connectedVehicles;

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write New Vehicle Data failed..." << std::endl;
    }
}


//!
//! \brief parseTCPRequest Parse data that has been sent to MACE via the MACE GUI
//! \param jsonObj JSON data to parse from the MACE GUI
//!
void MLGUItoMACE::parseTCPRequest(const QJsonObject &jsonObj)
{
    qDebug() << jsonObj;

    std::string command = jsonObj["command"].toString().toStdString();
    QJsonArray data = jsonObj["data"].toArray();


    QJsonDocument tmpDoc = QJsonDocument::fromJson(data[0].toString().toUtf8());
    if (command == "SET_ENVIRONMENT_VERTICES")
    {
        setEnvironmentVertices(tmpDoc);
    }
    else if (command == "GET_CONNECTED_VEHICLES")
    {
        getConnectedVehicles();
    }
    else if (command == "GET_ENVIRONMENT_BOUNDARY")
    {
        getEnvironmentBoundary();
    }

}

//!
//! \brief writeTCPData Write data to the MACE GUI via TCP
//! \param data Data to be sent to the MACE GUI
//! \return True: success / False: failure
//!
bool MLGUItoMACE::writeTCPData(QByteArray data)
{
    std::shared_ptr<QTcpSocket> tcpSocket = std::make_shared<QTcpSocket>();

    tcpSocket->connectToHost(m_sendAddress, m_sendPort);
    tcpSocket->waitForConnected();
    if(tcpSocket->state() == QAbstractSocket::ConnectedState)
    {
        tcpSocket->write(data); //write the data itself
        tcpSocket->flush();
        tcpSocket->waitForBytesWritten();
        return true;
    }
    else
    {
        std::cout << "TCP socket not connected MLGUI TO MACE" << std::endl;
        std::cout << tcpSocket->errorString().toStdString() << std::endl;
        tcpSocket->close();
        return false;
    }
}


