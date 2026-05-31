#ifndef CARBUSCONNECTION_H
#define CARBUSCONNECTION_H

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QThread>
#include <QDateTime>

#include "canframemodel.h"
#include "canconnection.h"
#include "canconmanager.h"

enum ConnState {
    STATE_IDLE,
    STATE_WAIT_SYNC,
    STATE_WAIT_DEVICE_INFO,
    STATE_WAIT_DEVICE_OPEN,
    STATE_WAIT_CHANNEL_OPEN,
    STATE_CONNECTED
};

class CarBusConnection : public CANConnection
{
    Q_OBJECT

public:
    CarBusConnection(QString portName, int serialSpeed, int busSpeed, bool canFd, int dataRate);
    virtual ~CarBusConnection();

protected:
    virtual void piStarted();
    virtual void piStop();
    virtual void piSetBusSettings(int pBusIdx, CANBus pBus);
    virtual bool piGetBusSettings(int pBusIdx, CANBus& pBus);
    virtual void piSuspend(bool pSuspend);
    virtual bool piSendFrame(const CANFrame&) ;

    void disconnectDevice();

public slots:
    void debugInput(QByteArray bytes);

private slots:
    void connectDevice();
    void readSerialData();
    void serialError(QSerialPort::SerialPortError err);
    void connectionTimeout();
    void handleTick();

private:
    void sendToSerial(const QByteArray &bytes);
    void sendDebug(const QString debugText);
    void parseReceivedData();
    bool sendCommand(quint8 cmd, quint16 flags, const QByteArray &payload, bool extendedHeader = false);
    void processPacket(quint8 cmd, quint8 seq, quint16 flags, const QByteArray &payload);
    void processCanMessage(quint16 flags, const QByteArray &payload);
    void processBusError(quint16 flags, const QByteArray &payload);
    void processDeviceInfo(const QByteArray &payload);
    quint8 nextSeq();
    bool bitrateToIndex(int bitrate, bool dataRate, quint8 &index);

    QSerialPort *serial;
    QTimer mTimer;

    int mSerialSpeed;
    int mBusSpeed;
    bool mCanFd;
    int mDataRate;
    quint8 mSeqCounter;
    bool mDeviceOpened;
    bool mChannelOpened;
    ConnState mConnState;
    int mStateTickCount;

    QByteArray mRxBuffer;
    qint64 mTimeBasis;
    uint32_t mBuildTimeBasis;

    // Device info
    int mHwId;
    int mNumHwBuses;
    bool mCanFdSupported;
};

#endif // CARBUSCONNECTION_H
