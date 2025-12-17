/**
 * @file serialcommunication.h
 * @brief Serial Communication Handler for PumpBoard GUI
 *
 * Handles USB serial communication with the STM32F407xx PumpBoard Controller.
 */

#ifndef SERIALCOMMUNICATION_H
#define SERIALCOMMUNICATION_H

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QByteArray>
#include <QQueue>
#include <QMutex>
#include "pumpboard_protocol.h"

/**
 * @brief Feedback data structure matching firmware
 */
struct FeedbackData {
    uint32_t timestamp_ms = 0;
    int32_t angle_feedback = 0;
    int32_t angle_ref = 0;
    int32_t angle_diff = 0;
    int32_t pressure_feedback = 0;
    int32_t pressure_ref = 0;
    int32_t pressure_diff = 0;
    int32_t current_a_feedback = 0;
    int32_t current_a_ref = 0;
    int32_t current_b_feedback = 0;
    int32_t current_b_ref = 0;
    uint32_t pwm_output_a = 0;
    uint32_t pwm_output_b = 0;
    uint8_t enable_flags = 0;
    uint8_t error_code = 0;
};

/**
 * @brief System status structure
 */
struct SystemStatus {
    uint8_t version_major = 0;
    uint8_t version_minor = 0;
    uint8_t version_patch = 0;
    uint8_t status_flags = 0;
    uint8_t error_code = 0;
    uint8_t enable_flags = 0;
    bool simulation_mode = false;
};

/**
 * @brief PID parameters structure
 */
struct PIDParameters {
    int32_t kp = 0;
    int32_t kp_div = 1;
    int32_t ki = 0;
    int32_t ki_div = 1;
    int32_t kd = 0;
    int32_t kd_div = 1;
    int32_t kv = 0;
    int32_t kv_div = 1;
    int32_t output_max = 0;
    int32_t output_min = 0;
};

/**
 * @brief Calibration parameters structure
 */
struct CalibrationParameters {
    uint16_t angle_min_adc = 0;
    uint16_t angle_mid_adc = 0;
    uint16_t angle_max_adc = 0;
    uint16_t angle_range = 5000;
    uint16_t pressure_min_adc = 0;
    uint16_t pressure_mid_adc = 0;
    uint16_t pressure_max_adc = 0;
    uint16_t pressure_max_bar = 600;
};

/**
 * @brief Filter parameters structure
 */
struct FilterParameters {
    float angle_ref_filter = 0.0065f;
    float angle_fdb_filter = 0.015f;
    float pressure_fdb_filter = 0.059f;
    float current_fdb_filter = 0.001f;
};

/**
 * @brief Analog input values structure
 */
struct AnalogInputs {
    uint16_t sensor_overcurrent = 0;
    uint16_t analog_out1_check = 0;
    uint16_t analog_out2_check = 0;
    uint16_t current_a_adc = 0;
    uint16_t current_b_adc = 0;
    uint16_t angle_adc = 0;
    uint16_t pressure_adc = 0;
};

/**
 * @brief Serial Communication Class
 *
 * Provides high-level interface for communicating with PumpBoard controller.
 */
class SerialCommunication : public QObject
{
    Q_OBJECT

public:
    enum ConnectionState {
        Disconnected,
        Connecting,
        Connected
    };
    Q_ENUM(ConnectionState)

    enum ReceiveState {
        WaitStart,
        GetCmd,
        GetLength,
        GetPayload,
        GetCrcL,
        GetCrcH,
        GetEnd
    };

    explicit SerialCommunication(QObject *parent = nullptr);
    ~SerialCommunication();

    // Connection management
    static QStringList availablePorts();
    bool connectToDevice(const QString &portName, qint32 baudRate = 115200);
    void disconnect();
    bool isConnected() const { return m_state == Connected; }
    ConnectionState connectionState() const { return m_state; }

    // Data requests
    void requestFeedbackData();
    void requestSystemStatus();
    void requestAnalogInputs();
    void requestAnglePID();
    void requestPressurePID();
    void requestCalibration();
    void requestFilterParams();

    // Control commands
    void setAngleReference(int32_t angle);      // 0-10000
    void setPressureReference(int32_t pressure); // 0-10000
    void setCurrentReferences(int32_t currentA, int32_t currentB);
    void setEnableFlags(uint8_t flags);

    // PID tuning
    void setAnglePID(const PIDParameters &params);
    void setPressurePID(const PIDParameters &params);

    // Calibration
    void setCalibration(const CalibrationParameters &params);
    void setFilterParams(const FilterParameters &params);

    // Streaming control
    void startStreaming(uint8_t rateMs = 50, uint8_t dataMask = 0xFF);
    void stopStreaming();

    // Heartbeat
    void sendHeartbeat();

    // Getters for latest data
    const FeedbackData &latestFeedback() const { return m_feedbackData; }
    const SystemStatus &systemStatus() const { return m_systemStatus; }
    const PIDParameters &anglePID() const { return m_anglePID; }
    const PIDParameters &pressurePID() const { return m_pressurePID; }
    const CalibrationParameters &calibration() const { return m_calibration; }
    const FilterParameters &filterParams() const { return m_filterParams; }
    const AnalogInputs &analogInputs() const { return m_analogInputs; }

    // Statistics
    uint32_t rxFrameCount() const { return m_rxFrameCount; }
    uint32_t txFrameCount() const { return m_txFrameCount; }
    uint32_t errorCount() const { return m_errorCount; }

signals:
    void connectionStateChanged(ConnectionState state);
    void feedbackDataReceived(const FeedbackData &data);
    void systemStatusReceived(const SystemStatus &status);
    void analogInputsReceived(const AnalogInputs &inputs);
    void anglePIDReceived(const PIDParameters &params);
    void pressurePIDReceived(const PIDParameters &params);
    void calibrationReceived(const CalibrationParameters &params);
    void filterParamsReceived(const FilterParameters &params);
    void ackReceived(uint8_t command);
    void nackReceived(uint8_t command, uint8_t errorCode);
    void errorOccurred(const QString &error);
    void streamDataReceived(const FeedbackData &data);

private slots:
    void onReadyRead();
    void onErrorOccurred(QSerialPort::SerialPortError error);
    void onHeartbeatTimeout();

private:
    // Frame handling
    void processByte(uint8_t byte);
    void processFrame();
    void sendFrame(uint8_t cmd, const uint8_t *payload, uint8_t length);
    uint16_t calculateCRC(const uint8_t *data, uint16_t length);

    // State
    QSerialPort *m_serialPort;
    ConnectionState m_state;
    QTimer *m_heartbeatTimer;

    // Receive state machine
    ReceiveState m_rxState;
    uint8_t m_rxCmd;
    uint8_t m_rxLength;
    QByteArray m_rxPayload;
    uint16_t m_rxCrc;

    // Latest data
    FeedbackData m_feedbackData;
    SystemStatus m_systemStatus;
    PIDParameters m_anglePID;
    PIDParameters m_pressurePID;
    CalibrationParameters m_calibration;
    FilterParameters m_filterParams;
    AnalogInputs m_analogInputs;

    // Statistics
    uint32_t m_rxFrameCount;
    uint32_t m_txFrameCount;
    uint32_t m_errorCount;

    // Mutex for thread safety
    QMutex m_mutex;
};

#endif // SERIALCOMMUNICATION_H
