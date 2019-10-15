#ifndef SERIALCONFIGURATION_H
#define SERIALCONFIGURATION_H

#include "DigiMesh_global.h"
#include <QSerialPort>
#include <QStringList>
#include <string>

#include "digi_common/digi_mesh_baud_rates.h"


class DIGIMESHSHARED_EXPORT SerialConfiguration
{

public:

    DigiMeshBaudRates  baud() const         { return _baud; }
    int  dataBits() const     { return _dataBits; }
    QSerialPort::FlowControl  flowControl() const  { return _flowControl; }    ///< QSerialPort Enums
    int  stopBits() const     { return _stopBits; }
    QSerialPort::Parity  parity() const       { return _parity; }         ///< QSerialPort Enums
    bool usbDirect() const    { return _usbDirect; }

    const std::string portName          () const { return _portName; }
    const std::string portDisplayName   () { return _portDisplayName; }

    void setBaud            (const DigiMeshBaudRates baud) {_baud = baud;}
    void setDataBits        (const int databits) {_dataBits = databits;}
    void setFlowControl     (const QSerialPort::FlowControl flowControl) {_flowControl = flowControl;}          ///< QSerialPort Enums
    void setStopBits        (const int stopBits) {_stopBits = stopBits; }
    void setParity          (const QSerialPort::Parity parity) {_parity = parity; }               ///< QSerialPort Enums
    void setPortName        (const std::string& portName) {_portName = portName;}
    void setUsbDirect       (const bool usbDirect) {_usbDirect = usbDirect;}


private:
    DigiMeshBaudRates _baud;
    int _dataBits;
    QSerialPort::FlowControl _flowControl;
    int _stopBits;
    QSerialPort::Parity _parity;
    std::string _portName;
    std::string _portDisplayName;
    bool _usbDirect;
};

#endif // SERIALCONFIGURATION_H
