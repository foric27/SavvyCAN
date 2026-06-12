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

// Device info param codes
static const quint32 DI_HW_ID = 0x01000000;
static const quint32 DI_FIRMWARE = 0x02000000;
static const quint32 DI_SERIAL = 0x03000000;
static const quint32 DI_FEATURES = 0x11000000;
static const quint32 DI_CHANNEL_MAP = 0x12000000;
static const quint32 DI_CHANNEL_FEATURES = 0x13000000;
static const quint32 CC_MULTIWORD = 0x80000000;

// Channel features
static const quint32 DI_CHANNEL_FEATURE_TERMINATOR = 0x00000002;

// Terminator config
static const quint8 FLAG_CONFIG_TERMINATOR = 0x05;

class CarBusConnection : public CANConnection
{
    Q_OBJECT

public:
    CarBusConnection(QString portName, int serialSpeed, int busSpeed, bool canFd, int dataRate);
    virtual ~CarBusConnection();

    // Filter management
    bool setCanFilter(int channel, int index, quint32 canId, quint32 mask, bool extended);
    bool clearCanFilter(int channel, int index);
    bool clearAllFilters(int channel);

    // Terminator control
    bool setTerminator(int channel, bool enabled);

    // Device info
    QString getFirmwareVersion() const { return mFirmwareVersion; }
    QString getSerialNumber() const { return mSerialNumber; }
    QString getHardwareName() const { return mHardwareName; }
    bool isTerminatorSupported() const { return mTerminatorSupported; }
    bool isCanFdSupported() const { return mCanFdSupported; }

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
    quint32 dlcToCanFdDlc(int len);
    int canFdDlcToLength(quint32 dlc);

    QSerialPort *serial;
    QTimer mTimer;

    int mSerialSpeed;
    int mBusSpeed;
    bool mCanFd;
    int mDataRate;
    quint8 mSeqCounter;
    bool mDeviceOpened;
    bool mChannelOpened;
    bool mChannelConfigured;
    ConnState mConnState;
    int mStateTickCount;

    QByteArray mRxBuffer;
    qint64 mTimeBasis;
    uint32_t mBuildTimeBasis;

    // Device info
    int mHwId;
    int mNumHwBuses;
    bool mCanFdSupported;
    QString mFirmwareVersion;
    QString mSerialNumber;
    QString mHardwareName;
    bool mTerminatorSupported;
};

#endif // CARBUSCONNECTION_H
