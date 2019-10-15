#include "serial_configuration.h"

namespace Comms
{

static QStringList kSupportedBaudRates;

SerialConfiguration::SerialConfiguration(const std::string& name)
{
    UNUSED(name);
    _baud       = 57600;
    _flowControl= QSerialPort::NoFlowControl;
    _parity     = QSerialPort::NoParity;
    _dataBits   = 8;
    _stopBits   = 1;
    _usbDirect  = false;
}

SerialConfiguration::SerialConfiguration(SerialConfiguration* copy)
{
    _baud               = copy->baud();
    _flowControl        = copy->flowControl();
    _parity             = copy->parity();
    _dataBits           = copy->dataBits();
    _stopBits           = copy->stopBits();
    _portName           = copy->portName();
    _portDisplayName    = copy->portDisplayName();
    _usbDirect          = copy->_usbDirect;
}

void SerialConfiguration::copyFrom(LinkConfiguration* source)
{
    LinkConfiguration::copyFrom(source);
    SerialConfiguration* ssource = dynamic_cast<SerialConfiguration*>(source);
    Q_ASSERT(ssource != nullptr);
    _baud               = ssource->baud();
    _flowControl        = ssource->flowControl();
    _parity             = ssource->parity();
    _dataBits           = ssource->dataBits();
    _stopBits           = ssource->stopBits();
    _portName           = ssource->portName();
    _portDisplayName    = ssource->portDisplayName();
    _usbDirect          = ssource->_usbDirect;
}

void SerialConfiguration::setBaud(int baud)
{
    _baud = baud;
}

void SerialConfiguration::setDataBits(int databits)
{
    _dataBits = databits;
}

void SerialConfiguration::setFlowControl(int flowControl)
{
    _flowControl = flowControl;
}

void SerialConfiguration::setStopBits(int stopBits)
{
    _stopBits = stopBits;
}

void SerialConfiguration::setParity(int parity)
{
    _parity = parity;
}

void SerialConfiguration::setPortName(const std::string &portName)
{
    // No effect on a running connection
    std::string pname = portName;
    if (!pname.empty() && pname != _portName) {
        _portName = pname;
        _portDisplayName = cleanPortDisplayname(pname);
    }
}


void SerialConfiguration::setUsbDirect(bool usbDirect)
{
    if (_usbDirect != usbDirect) {
        _usbDirect = usbDirect;
    }
}


QStringList SerialConfiguration::supportedBaudRates()
{
    if(!kSupportedBaudRates.size())
        _initBaudRates();
    return kSupportedBaudRates;
}

std::string SerialConfiguration::cleanPortDisplayname(const std::string &name)
{
    QString pname = QString::fromStdString(name).trimmed();
#ifdef Q_OS_WIN
    pname.replace("\\\\.\\", "");
#else
    pname.replace("/dev/cu.", "");
    pname.replace("/dev/", "");
#endif
    return pname.toStdString();
}


void SerialConfiguration::_initBaudRates()
{
    kSupportedBaudRates.clear();
    #if USE_ANCIENT_RATES
    #if defined(Q_OS_UNIX) || defined(Q_OS_LINUX) || defined(Q_OS_DARWIN)
        kSupportedBaudRates << "50";
        kSupportedBaudRates << "75";
    #endif
        kSupportedBaudRates << "110";
    #if defined(Q_OS_UNIX) || defined(Q_OS_LINUX) || defined(Q_OS_DARWIN)
        kSupportedBaudRates << "134";
        kSupportedBaudRates << "150";
        kSupportedBaudRates << "200";
    #endif
        kSupportedBaudRates << "300";
        kSupportedBaudRates << "600";
        kSupportedBaudRates << "1200";
    #if defined(Q_OS_UNIX) || defined(Q_OS_LINUX) || defined(Q_OS_DARWIN)
        kSupportedBaudRates << "1800";
    #endif
    #endif
        kSupportedBaudRates << "2400";
        kSupportedBaudRates << "4800";
        kSupportedBaudRates << "9600";
    #if defined(Q_OS_WIN)
        kSupportedBaudRates << "14400";
    #endif
        kSupportedBaudRates << "19200";
        kSupportedBaudRates << "38400";
    #if defined(Q_OS_WIN)
        kSupportedBaudRates << "56000";
    #endif
        kSupportedBaudRates << "57600";
        kSupportedBaudRates << "115200";
    #if defined(Q_OS_WIN)
        kSupportedBaudRates << "128000";
    #endif
        kSupportedBaudRates << "230400";
    #if defined(Q_OS_WIN)
        kSupportedBaudRates << "256000";
    #endif
        kSupportedBaudRates << "460800";
    #if defined(Q_OS_LINUX)
        kSupportedBaudRates << "500000";
        kSupportedBaudRates << "576000";
    #endif
        kSupportedBaudRates << "921600";
}

} //END MAVLINKComms
