#include <QObject>
#include <QDebug>
#include <QCanBusFrame>
#include <QSerialPortInfo>
#include <QSettings>
#include <QStringBuilder>
#include <QElapsedTimer>

#include "carbusconnection.h"

// Protocol constants from carbus_lib
static const quint8 CMD_SYNC = 0xA5;
static const quint8 CMD_DEVICE_INFO = 0x06;
static const quint8 CMD_DEVICE_OPEN = 0x08;
static const quint8 CMD_DEVICE_CLOSE = 0x09;
static const quint8 CMD_CHANNEL_CONFIG = 0x11;
static const quint8 CMD_CHANNEL_OPEN = 0x18;
static const quint8 CMD_FILTER_SET = 0x21;
static const quint8 CMD_FILTER_CLEAR = 0x22;
static const quint8 CMD_MESSAGE = 0x40;
static const quint8 CMD_BUS_ERROR = 0x48;
static const quint8 CMD_ERROR = 0xFF;

static const quint8 ACK_BASE = 0x80;

// Bus message flags
static const quint32 FLAG_EXTID = 0x00000001;
static const quint32 FLAG_RTR = 0x00000002;
static const quint32 FLAG_FDF = 0x00000004;
static const quint32 FLAG_BRS = 0x00000008;
static const quint32 FLAG_ESI = 0x00000010;
static const quint32 FLAG_RX = 0x10000000;
static const quint32 FLAG_TX = 0x20000000;
static const quint32 FLAG_BLOCK_TX = 0x30000000;

// Channel header flags
static const quint16 CH1 = 0x2000;
static const quint16 CH2 = 0x4000;
static const quint16 CH3 = 0x6000;
static const quint16 CH4 = 0x8000;

// Device info params
static const quint32 DI_HW_ID = 0x01000000;
static const quint32 DI_FIRMWARE = 0x02000000;
static const quint32 DI_SERIAL = 0x03000000;
static const quint32 DI_FEATURES = 0x11000000;
static const quint32 DI_CHANNEL_MAP = 0x12000000;
static const quint32 DI_CHANNEL_FEATURES = 0x13000000;
static const quint32 CC_MULTIWORD = 0x80000000;

static QMap<int, int> nominalBitrateIndex = {
    {10000, 0}, {20000, 1}, {33300, 2}, {50000, 3},
    {62500, 4}, {83300, 5}, {95200, 6}, {100000, 7},
    {125000, 8}, {250000, 9}, {400000, 10}, {500000, 11},
    {800000, 12}, {1000000, 13}
};

static QMap<int, int> dataBitrateIndex = {
    {500000, 0}, {1000000, 1}, {2000000, 2},
    {4000000, 3}, {5000000, 4}
};

CarBusConnection::CarBusConnection(QString portName, int serialSpeed, int busSpeed, bool canFd, int dataRate) :
    CANConnection(portName, "carbus", CANCon::CARBUS_HACKER, serialSpeed, busSpeed, canFd, dataRate, 1, 4000, true),
    mTimer(this),
    mSerialSpeed(serialSpeed),
    mBusSpeed(busSpeed),
    mCanFd(canFd),
    mDataRate(dataRate),
    mSeqCounter(0),
    mDeviceOpened(false),
    mChannelOpened(false),
    mHwId(0),
    mNumHwBuses(1),
    mCanFdSupported(false),
    mConnState(STATE_IDLE),
    mStateTickCount(0)
{
    sendDebug("CarBusConnection()");
    serial = nullptr;
    mTimeBasis = 0;
    mBuildTimeBasis = 0;
}

CarBusConnection::~CarBusConnection()
{
    stop();
    sendDebug("~CarBusConnection()");
}

void CarBusConnection::sendDebug(const QString debugText)
{
    qDebug() << debugText;
    debugOutput(debugText);
}

void CarBusConnection::sendToSerial(const QByteArray &bytes)
{
    if (serial == nullptr)
    {
        sendDebug("Attempt to write to serial port when it has not been initialized!");
        return;
    }

    if (!serial->isOpen())
    {
        sendDebug("Attempt to write to serial port when it is not open!");
        return;
    }

    QString buildDebug;
    buildDebug = "Write to serial -> ";
    foreach (int byt, bytes) {
        byt = (unsigned char)byt;
        buildDebug = buildDebug % QString::number(byt, 16).rightJustified(2, '0') % " ";
    }
    sendDebug(buildDebug);

    serial->write(bytes);
}

quint8 CarBusConnection::nextSeq()
{
    mSeqCounter++;
    if (mSeqCounter == 0) mSeqCounter = 1;
    return mSeqCounter;
}

bool CarBusConnection::bitrateToIndex(int bitrate, bool dataRate, quint8 &index)
{
    if (dataRate) {
        if (dataBitrateIndex.contains(bitrate)) {
            index = dataBitrateIndex[bitrate];
            return true;
        }
    } else {
        if (nominalBitrateIndex.contains(bitrate)) {
            index = nominalBitrateIndex[bitrate];
            return true;
        }
    }
    return false;
}

bool CarBusConnection::sendCommand(quint8 cmd, quint16 flags, const QByteArray &payload, bool extendedHeader)
{
    quint8 seq = nextSeq();
    QByteArray frame;

    if (extendedHeader) {
        // Extended header: [CMD:1][SEQ:1][FLAGS:2][DSIZE:2]
        frame.append((char)cmd);
        frame.append((char)seq);
        frame.append((char)(flags & 0xFF));
        frame.append((char)((flags >> 8) & 0xFF));
        frame.append((char)(payload.length() & 0xFF));
        frame.append((char)((payload.length() >> 8) & 0xFF));
    } else {
        // Standard header: [CMD:1][SEQ:1][FLAGS:1][DSIZE:1]
        frame.append((char)cmd);
        frame.append((char)seq);
        frame.append((char)(flags & 0xFF));
        frame.append((char)(payload.length() & 0xFF));
    }
    frame.append(payload);

    sendToSerial(frame);
    return true;
}

void CarBusConnection::piStarted()
{
    connectDevice();
}

void CarBusConnection::piStop()
{
    mTimer.stop();
    disconnectDevice();
}

void CarBusConnection::piSuspend(bool pSuspend)
{
    setCapSuspended(pSuspend);
    if (isCapSuspended())
        getQueue().flush();
}

bool CarBusConnection::piGetBusSettings(int pBusIdx, CANBus& pBus)
{
    return getBusConfig(pBusIdx, pBus);
}

void CarBusConnection::piSetBusSettings(int pBusIdx, CANBus bus)
{
    if ((pBusIdx < 0) || pBusIdx >= getNumBuses())
        return;

    setBusConfig(pBusIdx, bus);

    if (mChannelOpened) {
        // Reopen channel with new settings
        sendDebug("Reopening CAN channel with new settings");

        quint8 bitrateIdx;
        if (!bitrateToIndex(bus.getSpeed(), false, bitrateIdx)) {
            sendDebug("Unsupported bitrate: " + QString::number(bus.getSpeed()));
            return;
        }

        quint8 modeVal = 0x00; // normal
        if (bus.isListenOnly()) modeVal = 0x01;

        quint8 frameMode = 0x00; // classic
        if (bus.isCanFD()) {
            frameMode = bus.isCanFD() ? 0x02 : 0x01; // BRS if data rate set
        }

        QByteArray payload;
        // CC_CAN_MODE
        quint32 ccMode = 0x11000000 | modeVal;
        payload.append((char)(ccMode & 0xFF));
        payload.append((char)((ccMode >> 8) & 0xFF));
        payload.append((char)((ccMode >> 16) & 0xFF));
        payload.append((char)((ccMode >> 24) & 0xFF));

        // CC_CAN_FRAME
        quint32 ccFrame = 0x12000000 | frameMode;
        payload.append((char)(ccFrame & 0xFF));
        payload.append((char)((ccFrame >> 8) & 0xFF));
        payload.append((char)((ccFrame >> 16) & 0xFF));
        payload.append((char)((ccFrame >> 24) & 0xFF));

        // CC_BUS_SPEED_N
        quint32 ccSpeedN = 0x01000000 | bitrateIdx;
        payload.append((char)(ccSpeedN & 0xFF));
        payload.append((char)((ccSpeedN >> 8) & 0xFF));
        payload.append((char)((ccSpeedN >> 16) & 0xFF));
        payload.append((char)((ccSpeedN >> 24) & 0xFF));

        if (bus.isCanFD()) {
            quint8 dataBitrateIdx;
            if (bitrateToIndex(bus.getDataRate(), true, dataBitrateIdx)) {
                quint32 ccSpeedD = 0x02000000 | dataBitrateIdx;
                payload.append((char)(ccSpeedD & 0xFF));
                payload.append((char)((ccSpeedD >> 8) & 0xFF));
                payload.append((char)((ccSpeedD >> 16) & 0xFF));
                payload.append((char)((ccSpeedD >> 24) & 0xFF));
            }
        }

        quint16 headerFlags = ((pBusIdx + 1) & 0x0F) * 0x20;
        sendCommand(CMD_CHANNEL_OPEN, headerFlags, payload, false);
    }
}

bool CarBusConnection::piSendFrame(const CANFrame& frame)
{
    if (serial == nullptr || !serial->isOpen()) return false;

    quint32 id = frame.frameId();
    quint32 msgFlags = FLAG_BLOCK_TX; // Don't echo back

    if (frame.hasExtendedFrameFormat()) {
        msgFlags |= FLAG_EXTID;
        id &= 0x1FFFFFFF;
    } else {
        id &= 0x7FF;
    }

    if (frame.frameType() == QCanBusFrame::RemoteRequestFrame) {
        msgFlags |= FLAG_RTR;
    }

    // Note: CANFrame (Qt5 QCanBusFrame) doesn't have isCanFD() method
    // CAN-FD is determined by payload length > 8 or bus configuration
    if (frame.payload().length() > 8 || mCanFd) {
        msgFlags |= FLAG_FDF;
        // BRS flag could be set based on configuration if needed
    }

    quint32 timestamp = 0;
    quint32 reserved = 0;
    quint32 dlc = frame.payload().length();

    QByteArray payload;
    // MSG_FLAGS
    payload.append((char)(msgFlags & 0xFF));
    payload.append((char)((msgFlags >> 8) & 0xFF));
    payload.append((char)((msgFlags >> 16) & 0xFF));
    payload.append((char)((msgFlags >> 24) & 0xFF));
    // TIMESTAMP
    payload.append((char)(timestamp & 0xFF));
    payload.append((char)((timestamp >> 8) & 0xFF));
    payload.append((char)((timestamp >> 16) & 0xFF));
    payload.append((char)((timestamp >> 24) & 0xFF));
    // RESERVED
    payload.append((char)(reserved & 0xFF));
    payload.append((char)((reserved >> 8) & 0xFF));
    payload.append((char)((reserved >> 16) & 0xFF));
    payload.append((char)((reserved >> 24) & 0xFF));
    // CAN_ID
    payload.append((char)(id & 0xFF));
    payload.append((char)((id >> 8) & 0xFF));
    payload.append((char)((id >> 16) & 0xFF));
    payload.append((char)((id >> 24) & 0xFF));
    // DLC
    payload.append((char)(dlc & 0xFF));
    payload.append((char)((dlc >> 8) & 0xFF));
    payload.append((char)((dlc >> 16) & 0xFF));
    payload.append((char)((dlc >> 24) & 0xFF));
    // DATA
    payload.append(frame.payload());

    quint16 headerFlags = ((frame.bus + 1) & 0x0F) * 0x20;
    sendCommand(CMD_MESSAGE, headerFlags, payload, true);

    return true;
}

void CarBusConnection::connectDevice()
{
    if (serial)
        disconnectDevice();

    sendDebug("Serial connection to CAN-Hacker device");
    serial = new QSerialPort(QSerialPortInfo(getPort()));
    if (!serial) {
        sendDebug("Can't open serial port " + getPort());
        return;
    }

    connect(serial, SIGNAL(readyRead()), this, SLOT(readSerialData()));
    connect(serial, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(serialError(QSerialPort::SerialPortError)));

    serial->setBaudRate(mSerialSpeed);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    if (!serial->open(QIODevice::ReadWrite)) {
        sendDebug("Error opening serial port: " + serial->errorString());
        return;
    }

    sendDebug("Serial port opened, sending SYNC...");
    mConnState = STATE_WAIT_SYNC;
    mStateTickCount = 0;

    // Send SYNC sequence
    QByteArray sync;
    sync.append((char)CMD_SYNC);
    sync.append((char)0x00);
    sync.append((char)CMD_SYNC);
    sync.append((char)0x00);
    sendToSerial(sync);

    // Setup periodic timer for keepalive/status
    connect(&mTimer, SIGNAL(timeout()), this, SLOT(handleTick()));
    mTimer.start(1000);
}

void CarBusConnection::disconnectDevice()
{
    mTimer.stop();
    mConnState = STATE_IDLE;

    if (mDeviceOpened) {
        sendCommand(CMD_DEVICE_CLOSE, 0, QByteArray(), false);
        mDeviceOpened = false;
    }
    mChannelOpened = false;

    if (serial) {
        if (serial->isOpen()) {
            serial->close();
        }
        serial->deleteLater();
        serial = nullptr;
    }

    CANConStatus stats;
    stats.conStatus = CANCon::NOT_CONNECTED;
    stats.numHardwareBuses = mNumHwBuses;
    emit status(stats);
}

void CarBusConnection::serialError(QSerialPort::SerialPortError err)
{
    if (err != QSerialPort::NoError) {
        sendDebug("Serial error: " + QString::number(err));
        disconnectDevice();
    }
}

void CarBusConnection::connectionTimeout()
{
    sendDebug("Connection timeout - retrying...");
    disconnectDevice();
    connectDevice();
}

void CarBusConnection::handleTick()
{
    // State machine timeout handling
    if (mConnState != STATE_IDLE && mConnState != STATE_CONNECTED) {
        mStateTickCount++;
        if (mStateTickCount >= 5) {
            sendDebug(QString("State timeout in state %1, retrying connection...").arg(mConnState));
            mStateTickCount = 0;
            disconnectDevice();
            connectDevice();
        }
    } else {
        mStateTickCount = 0;
    }
}

void CarBusConnection::readSerialData()
{
    if (!serial) return;

    QByteArray data = serial->readAll();
    mRxBuffer.append(data);

    QString debugBuild;
    for (int i = 0; i < data.length(); i++) {
        debugBuild = debugBuild % QString::number((unsigned char)data.at(i), 16).rightJustified(2, '0') % " ";
    }
    if (!debugBuild.isEmpty()) {
        sendDebug("RX raw: " + debugBuild);
    }

    parseReceivedData();
}

void CarBusConnection::parseReceivedData()
{
    while (mRxBuffer.size() >= 4) {
        // Check for SYNC response first
        if ((unsigned char)mRxBuffer.at(0) == 0x5A &&
            (unsigned char)mRxBuffer.at(1) == 0x00 &&
            (unsigned char)mRxBuffer.at(2) == 0x5A &&
            (unsigned char)mRxBuffer.at(3) == 0x00) {
            sendDebug("SYNC response received");
            mRxBuffer.remove(0, 4);
            // After SYNC, request device info
            if (mConnState == STATE_WAIT_SYNC) {
                mConnState = STATE_WAIT_DEVICE_INFO;
                mStateTickCount = 0;
                sendDebug("Requesting DEVICE_INFO...");
                sendCommand(CMD_DEVICE_INFO, 0, QByteArray(), false);
            }
            continue;
        }

        quint8 cmd = (unsigned char)mRxBuffer.at(0);

        // Check if this command needs extended header
        bool extendedHeader = (cmd == CMD_MESSAGE || cmd == CMD_BUS_ERROR);

        int headerSize = extendedHeader ? 6 : 4;
        if (mRxBuffer.size() < headerSize) break;

        quint8 seq = (unsigned char)mRxBuffer.at(1);
        quint16 flags;
        quint16 dsize;

        if (extendedHeader) {
            flags = (unsigned char)mRxBuffer.at(2) | ((unsigned char)mRxBuffer.at(3) << 8);
            dsize = (unsigned char)mRxBuffer.at(4) | ((unsigned char)mRxBuffer.at(5) << 8);
        } else {
            flags = (unsigned char)mRxBuffer.at(2);
            dsize = (unsigned char)mRxBuffer.at(3);
        }

        if (mRxBuffer.size() < headerSize + dsize) break;

        QByteArray payload = mRxBuffer.mid(headerSize, dsize);
        mRxBuffer.remove(0, headerSize + dsize);

        processPacket(cmd, seq, flags, payload);
    }
}

void CarBusConnection::processPacket(quint8 cmd, quint8 seq, quint16 flags, const QByteArray &payload)
{
    sendDebug(QString("Process packet cmd=0x%1 seq=%2 flags=0x%3 dsize=%4")
              .arg(cmd, 2, 16, QChar('0'))
              .arg(seq)
              .arg(flags, 4, 16, QChar('0'))
              .arg(payload.size()));

    // Check for ACK
    if (cmd >= ACK_BASE && cmd < 0xFF) {
        quint8 baseCmd = cmd & 0x7F;
        sendDebug(QString("ACK for cmd=0x%1").arg(baseCmd, 2, 16, QChar('0')));

        if (baseCmd == CMD_DEVICE_OPEN) {
            mDeviceOpened = true;
            sendDebug("DEVICE_OPEN ACK received");
            // After DEVICE_OPEN ACK, send CHANNEL_OPEN
            if (mConnState == STATE_WAIT_DEVICE_OPEN) {
                mConnState = STATE_WAIT_CHANNEL_OPEN;
                mStateTickCount = 0;
                sendDebug("Sending CHANNEL_OPEN...");
                CANBus bus;
                if (getBusConfig(0, bus)) {
                    piSetBusSettings(0, bus);
                } else {
                    bus.setSpeed(500000);
                    bus.setActive(true);
                    bus.setListenOnly(false);
                    bus.setCanFD(false);
                    bus.setDataRate(2000000);
                    setBusConfig(0, bus);
                    piSetBusSettings(0, bus);
                }
            }
        } else if (baseCmd == CMD_CHANNEL_OPEN) {
            mChannelOpened = true;
            mConnState = STATE_CONNECTED;
            mStateTickCount = 0;
            CANConStatus stats;
            stats.conStatus = CANCon::CONNECTED;
            stats.numHardwareBuses = mNumHwBuses;
            emit status(stats);
            sendDebug("CHANNEL_OPEN ACK received - CONNECTED");
        } else if (baseCmd == CMD_DEVICE_CLOSE) {
            mDeviceOpened = false;
            mChannelOpened = false;
            mConnState = STATE_IDLE;
            mStateTickCount = 0;
        }
        return;
    }

    switch (cmd) {
    case CMD_SYNC:
        sendDebug("SYNC packet received");
        break;

    case CMD_DEVICE_INFO:
        processDeviceInfo(payload);
        break;

    case CMD_MESSAGE:
        processCanMessage(flags, payload);
        break;

    case CMD_BUS_ERROR:
        processBusError(flags, payload);
        break;

    case CMD_ERROR:
        sendDebug("ERROR packet: " + payload.toHex(' '));
        break;

    default:
        sendDebug(QString("Unknown packet cmd=0x%1").arg(cmd, 2, 16, QChar('0')));
        break;
    }
}

void CarBusConnection::processDeviceInfo(const QByteArray &payload)
{
    sendDebug("DEVICE_INFO received, size=" + QString::number(payload.size()));

    if (payload.size() % 4 != 0) {
        sendDebug("Invalid DEVICE_INFO payload size");
        return;
    }

    int numWords = payload.size() / 4;
    QVector<quint32> words;
    for (int i = 0; i < numWords; i++) {
        quint32 word = (unsigned char)payload.at(i*4) |
                      ((unsigned char)payload.at(i*4+1) << 8) |
                      ((unsigned char)payload.at(i*4+2) << 16) |
                      ((unsigned char)payload.at(i*4+3) << 24);
        words.append(word);
    }

    int i = 0;
    while (i < numWords) {
        quint32 header = words[i++];
        quint32 paramCode = header & 0xFF000000;

        QVector<quint32> data;
        if (header & CC_MULTIWORD) {
            int length = (header >> 16) & 0xFF;
            for (int j = 0; j < length && i < numWords; j++) {
                data.append(words[i++]);
            }
        }

        if (paramCode == DI_HW_ID) {
            mHwId = header & 0xFF;
            sendDebug("HW ID: 0x" + QString::number(mHwId, 16));
        } else if (paramCode == DI_CHANNEL_MAP) {
            // Parse channel types
            quint8 b0 = header & 0xFF;
            quint8 b1 = (header >> 8) & 0xFF;
            quint8 b2 = (header >> 16) & 0xFF;
            mNumHwBuses = 0;
            if (b0 == 0x01 || b0 == 0x02) mNumHwBuses++;
            if (b1 == 0x01 || b1 == 0x02) mNumHwBuses++;
            if (b2 == 0x01 || b2 == 0x02) mNumHwBuses++;
            sendDebug("Number of CAN buses: " + QString::number(mNumHwBuses));
        } else if (paramCode == DI_FEATURES) {
            quint32 features = header & 0x00FFFFFF;
            sendDebug("Features: 0x" + QString::number(features, 16));
        }
    }

    // Update number of buses
    if (mNumHwBuses > 0) {
        mNumBuses = mNumHwBuses;
    }

    // After DEVICE_INFO, send DEVICE_OPEN
    if (mConnState == STATE_WAIT_DEVICE_INFO) {
        mConnState = STATE_WAIT_DEVICE_OPEN;
        mStateTickCount = 0;
        sendDebug("Sending DEVICE_OPEN...");
        quint32 mode = 0x01000000 | 0x01; // CAN only mode
        QByteArray payload;
        payload.append((char)(mode & 0xFF));
        payload.append((char)((mode >> 8) & 0xFF));
        payload.append((char)((mode >> 16) & 0xFF));
        payload.append((char)((mode >> 24) & 0xFF));
        sendCommand(CMD_DEVICE_OPEN, 0, payload, false);
    }
}

void CarBusConnection::processCanMessage(quint16 flags, const QByteArray &payload)
{
    if (payload.size() < 20) {
        sendDebug("Invalid CAN message payload size: " + QString::number(payload.size()));
        return;
    }

    quint32 msgFlags = (unsigned char)payload.at(0) |
                      ((unsigned char)payload.at(1) << 8) |
                      ((unsigned char)payload.at(2) << 16) |
                      ((unsigned char)payload.at(3) << 24);

    quint32 timestamp = (unsigned char)payload.at(4) |
                       ((unsigned char)payload.at(5) << 8) |
                       ((unsigned char)payload.at(6) << 16) |
                       ((unsigned char)payload.at(7) << 24);

    quint32 reserved = (unsigned char)payload.at(8) |
                      ((unsigned char)payload.at(9) << 8) |
                      ((unsigned char)payload.at(10) << 16) |
                      ((unsigned char)payload.at(11) << 24);

    quint32 canId = (unsigned char)payload.at(12) |
                   ((unsigned char)payload.at(13) << 8) |
                   ((unsigned char)payload.at(14) << 16) |
                   ((unsigned char)payload.at(15) << 24);

    quint32 dlc = (unsigned char)payload.at(16) |
                 ((unsigned char)payload.at(17) << 8) |
                 ((unsigned char)payload.at(18) << 16) |
                 ((unsigned char)payload.at(19) << 24);

    QByteArray data = payload.mid(20, dlc);

    // Determine channel
    int bus = 0;
    if (flags & CH1) bus = 0;
    else if (flags & CH2) bus = 1;
    else if (flags & CH3) bus = 2;
    else if (flags & CH4) bus = 3;

    // Build CANFrame
    CANFrame frame;
    frame.setFrameId(canId);
    frame.setExtendedFrameFormat(msgFlags & FLAG_EXTID);
    frame.setPayload(data);
    frame.bus = bus;
    frame.isReceived = (msgFlags & FLAG_RX) ? true : false;

    // Note: CANFrame (Qt5 QCanBusFrame) doesn't have setCanFD() method
    // CAN-FD is determined by payload length > 8 or FDF flag in protocol
    // The payload size is already set above via setPayload()

    if (msgFlags & FLAG_RTR) {
        frame.setFrameType(QCanBusFrame::RemoteRequestFrame);
    } else {
        frame.setFrameType(QCanBusFrame::DataFrame);
    }

    // Timestamp
    if (useSystemTime) {
        frame.setTimeStamp(QCanBusFrame::TimeStamp::fromMicroSeconds(QDateTime::currentMSecsSinceEpoch() * 1000ul));
    } else {
        frame.setTimeStamp(QCanBusFrame::TimeStamp(0, timestamp));
    }

    if (!isCapSuspended()) {
        CANFrame* frame_p = getQueue().get();
        if (frame_p) {
            *frame_p = frame;
            checkTargettedFrame(frame);
            getQueue().queue();
        }
    }
}

void CarBusConnection::processBusError(quint16 flags, const QByteArray &payload)
{
    sendDebug("BUS_ERROR: flags=0x" + QString::number(flags, 16) + " payload=" + payload.toHex(' '));
}

void CarBusConnection::debugInput(QByteArray bytes)
{
    sendToSerial(bytes);
}
