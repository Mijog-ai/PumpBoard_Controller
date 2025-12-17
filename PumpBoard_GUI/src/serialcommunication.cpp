/**
 * @file serialcommunication.cpp
 * @brief Serial Communication Handler Implementation
 */

#include "serialcommunication.h"
#include <QDebug>
#include <cstring>

SerialCommunication::SerialCommunication(QObject *parent)
    : QObject(parent)
    , m_serialPort(new QSerialPort(this))
    , m_state(Disconnected)
    , m_heartbeatTimer(new QTimer(this))
    , m_rxState(WaitStart)
    , m_rxCmd(0)
    , m_rxLength(0)
    , m_rxCrc(0)
    , m_rxFrameCount(0)
    , m_txFrameCount(0)
    , m_errorCount(0)
{
    connect(m_serialPort, &QSerialPort::readyRead,
            this, &SerialCommunication::onReadyRead);
    connect(m_serialPort, &QSerialPort::errorOccurred,
            this, &SerialCommunication::onErrorOccurred);
    connect(m_heartbeatTimer, &QTimer::timeout,
            this, &SerialCommunication::onHeartbeatTimeout);

    // Heartbeat every 1 second
    m_heartbeatTimer->setInterval(1000);
}

SerialCommunication::~SerialCommunication()
{
    disconnect();
}

QStringList SerialCommunication::availablePorts()
{
    QStringList ports;
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos) {
        ports.append(QString("%1 - %2")
                     .arg(info.portName())
                     .arg(info.description()));
    }
    return ports;
}

bool SerialCommunication::connectToDevice(const QString &portName, qint32 baudRate)
{
    if (m_state != Disconnected) {
        disconnect();
    }

    m_state = Connecting;
    emit connectionStateChanged(m_state);

    // Extract port name (before " - ")
    QString actualPort = portName.split(" - ").first();

    m_serialPort->setPortName(actualPort);
    m_serialPort->setBaudRate(baudRate);
    m_serialPort->setDataBits(QSerialPort::Data8);
    m_serialPort->setParity(QSerialPort::NoParity);
    m_serialPort->setStopBits(QSerialPort::OneStop);
    m_serialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (!m_serialPort->open(QIODevice::ReadWrite)) {
        m_state = Disconnected;
        emit connectionStateChanged(m_state);
        emit errorOccurred(QString("Failed to open port: %1")
                          .arg(m_serialPort->errorString()));
        return false;
    }

    m_state = Connected;
    emit connectionStateChanged(m_state);

    // Reset state
    m_rxState = WaitStart;
    m_rxPayload.clear();
    m_rxFrameCount = 0;
    m_txFrameCount = 0;
    m_errorCount = 0;

    // Start heartbeat timer
    m_heartbeatTimer->start();

    // Request initial status
    requestSystemStatus();

    return true;
}

void SerialCommunication::disconnect()
{
    m_heartbeatTimer->stop();

    if (m_serialPort->isOpen()) {
        stopStreaming();
        m_serialPort->close();
    }

    m_state = Disconnected;
    emit connectionStateChanged(m_state);
}

uint16_t SerialCommunication::calculateCRC(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}

void SerialCommunication::sendFrame(uint8_t cmd, const uint8_t *payload, uint8_t length)
{
    if (!m_serialPort->isOpen()) {
        return;
    }

    QByteArray frame;
    frame.reserve(length + 6);

    // Build CRC data
    QByteArray crcData;
    crcData.append(static_cast<char>(cmd));
    crcData.append(static_cast<char>(length));
    if (length > 0 && payload != nullptr) {
        crcData.append(reinterpret_cast<const char*>(payload), length);
    }

    uint16_t crc = calculateCRC(reinterpret_cast<const uint8_t*>(crcData.constData()),
                                static_cast<uint16_t>(crcData.size()));

    // Build frame
    frame.append(static_cast<char>(PROTOCOL_START_BYTE));
    frame.append(static_cast<char>(cmd));
    frame.append(static_cast<char>(length));
    if (length > 0 && payload != nullptr) {
        frame.append(reinterpret_cast<const char*>(payload), length);
    }
    frame.append(static_cast<char>(crc & 0xFF));
    frame.append(static_cast<char>((crc >> 8) & 0xFF));
    frame.append(static_cast<char>(PROTOCOL_END_BYTE));

    m_serialPort->write(frame);
    m_txFrameCount++;
}

void SerialCommunication::onReadyRead()
{
    QByteArray data = m_serialPort->readAll();
    for (int i = 0; i < data.size(); i++) {
        processByte(static_cast<uint8_t>(data[i]));
    }
}

void SerialCommunication::processByte(uint8_t byte)
{
    switch (m_rxState) {
    case WaitStart:
        if (byte == PROTOCOL_START_BYTE) {
            m_rxState = GetCmd;
            m_rxPayload.clear();
        }
        break;

    case GetCmd:
        m_rxCmd = byte;
        m_rxState = GetLength;
        break;

    case GetLength:
        m_rxLength = byte;
        if (m_rxLength > 0 && m_rxLength <= MAX_PAYLOAD_SIZE) {
            m_rxState = GetPayload;
        } else if (m_rxLength == 0) {
            m_rxState = GetCrcL;
        } else {
            m_errorCount++;
            m_rxState = WaitStart;
        }
        break;

    case GetPayload:
        m_rxPayload.append(static_cast<char>(byte));
        if (m_rxPayload.size() >= m_rxLength) {
            m_rxState = GetCrcL;
        }
        break;

    case GetCrcL:
        m_rxCrc = byte;
        m_rxState = GetCrcH;
        break;

    case GetCrcH:
        m_rxCrc |= (static_cast<uint16_t>(byte) << 8);
        m_rxState = GetEnd;
        break;

    case GetEnd:
        if (byte == PROTOCOL_END_BYTE) {
            // Verify CRC
            QByteArray crcData;
            crcData.append(static_cast<char>(m_rxCmd));
            crcData.append(static_cast<char>(m_rxLength));
            crcData.append(m_rxPayload);

            uint16_t calcCrc = calculateCRC(
                reinterpret_cast<const uint8_t*>(crcData.constData()),
                static_cast<uint16_t>(crcData.size()));

            if (calcCrc == m_rxCrc) {
                m_rxFrameCount++;
                processFrame();
            } else {
                m_errorCount++;
                emit errorOccurred("CRC mismatch");
            }
        } else {
            m_errorCount++;
        }
        m_rxState = WaitStart;
        m_rxPayload.clear();
        break;
    }
}

void SerialCommunication::processFrame()
{
    const uint8_t *data = reinterpret_cast<const uint8_t*>(m_rxPayload.constData());

    switch (m_rxCmd) {
    case CMD_ACK:
        if (m_rxLength >= 1) {
            emit ackReceived(data[0]);
        }
        break;

    case CMD_NACK:
        if (m_rxLength >= 2) {
            emit nackReceived(data[0], data[1]);
        }
        break;

    case CMD_GET_VERSION:
    case CMD_GET_STATUS:
        if (m_rxLength >= sizeof(SystemStatus_t)) {
            const SystemStatus_t *status = reinterpret_cast<const SystemStatus_t*>(data);
            m_systemStatus.version_major = status->version_major;
            m_systemStatus.version_minor = status->version_minor;
            m_systemStatus.version_patch = status->version_patch;
            m_systemStatus.status_flags = status->status_flags;
            m_systemStatus.error_code = status->error_code;
            m_systemStatus.enable_flags = status->enable_flags;
            m_systemStatus.simulation_mode = (status->simulation_mode != 0);
            emit systemStatusReceived(m_systemStatus);
        }
        break;

    case CMD_GET_ALL_FEEDBACK:
    case CMD_STREAM_DATA:
        if (m_rxLength >= sizeof(FeedbackData_t)) {
            const FeedbackData_t *fdb = reinterpret_cast<const FeedbackData_t*>(data);
            m_feedbackData.timestamp_ms = fdb->timestamp_ms;
            m_feedbackData.angle_feedback = fdb->angle_feedback;
            m_feedbackData.angle_ref = fdb->angle_ref;
            m_feedbackData.angle_diff = fdb->angle_diff;
            m_feedbackData.pressure_feedback = fdb->pressure_feedback;
            m_feedbackData.pressure_ref = fdb->pressure_ref;
            m_feedbackData.pressure_diff = fdb->pressure_diff;
            m_feedbackData.current_a_feedback = fdb->current_a_feedback;
            m_feedbackData.current_a_ref = fdb->current_a_ref;
            m_feedbackData.current_b_feedback = fdb->current_b_feedback;
            m_feedbackData.current_b_ref = fdb->current_b_ref;
            m_feedbackData.pwm_output_a = fdb->pwm_output_a;
            m_feedbackData.pwm_output_b = fdb->pwm_output_b;
            m_feedbackData.enable_flags = fdb->enable_flags;
            m_feedbackData.error_code = fdb->error_code;

            if (m_rxCmd == CMD_STREAM_DATA) {
                emit streamDataReceived(m_feedbackData);
            } else {
                emit feedbackDataReceived(m_feedbackData);
            }
        }
        break;

    case CMD_GET_ANALOG_INPUT:
        if (m_rxLength >= sizeof(AnalogInputs_t)) {
            const AnalogInputs_t *inputs = reinterpret_cast<const AnalogInputs_t*>(data);
            m_analogInputs.sensor_overcurrent = inputs->sensor_overcurrent;
            m_analogInputs.analog_out1_check = inputs->analog_out1_check;
            m_analogInputs.analog_out2_check = inputs->analog_out2_check;
            m_analogInputs.current_a_adc = inputs->current_a_adc;
            m_analogInputs.current_b_adc = inputs->current_b_adc;
            m_analogInputs.angle_adc = inputs->angle_adc;
            m_analogInputs.pressure_adc = inputs->pressure_adc;
            emit analogInputsReceived(m_analogInputs);
        }
        break;

    case CMD_GET_PID_ANGLE:
        if (m_rxLength >= sizeof(PIDParams_t)) {
            const PIDParams_t *pid = reinterpret_cast<const PIDParams_t*>(data);
            m_anglePID.kp = pid->kp;
            m_anglePID.kp_div = pid->kp_div;
            m_anglePID.ki = pid->ki;
            m_anglePID.ki_div = pid->ki_div;
            m_anglePID.kd = pid->kd;
            m_anglePID.kd_div = pid->kd_div;
            m_anglePID.kv = pid->kv;
            m_anglePID.kv_div = pid->kv_div;
            m_anglePID.output_max = pid->output_max;
            m_anglePID.output_min = pid->output_min;
            emit anglePIDReceived(m_anglePID);
        }
        break;

    case CMD_GET_PID_PRESSURE:
        if (m_rxLength >= sizeof(PIDParams_t)) {
            const PIDParams_t *pid = reinterpret_cast<const PIDParams_t*>(data);
            m_pressurePID.kp = pid->kp;
            m_pressurePID.kp_div = pid->kp_div;
            m_pressurePID.ki = pid->ki;
            m_pressurePID.ki_div = pid->ki_div;
            m_pressurePID.kd = pid->kd;
            m_pressurePID.kd_div = pid->kd_div;
            m_pressurePID.kv = pid->kv;
            m_pressurePID.kv_div = pid->kv_div;
            m_pressurePID.output_max = pid->output_max;
            m_pressurePID.output_min = pid->output_min;
            emit pressurePIDReceived(m_pressurePID);
        }
        break;

    case CMD_GET_CALIBRATION:
        if (m_rxLength >= sizeof(CalibrationParams_t)) {
            const CalibrationParams_t *cal = reinterpret_cast<const CalibrationParams_t*>(data);
            m_calibration.angle_min_adc = cal->angle_min_adc;
            m_calibration.angle_mid_adc = cal->angle_mid_adc;
            m_calibration.angle_max_adc = cal->angle_max_adc;
            m_calibration.angle_range = cal->angle_range;
            m_calibration.pressure_min_adc = cal->pressure_min_adc;
            m_calibration.pressure_mid_adc = cal->pressure_mid_adc;
            m_calibration.pressure_max_adc = cal->pressure_max_adc;
            m_calibration.pressure_max_bar = cal->pressure_max_bar;
            emit calibrationReceived(m_calibration);
        }
        break;

    case CMD_GET_FILTER_PARAMS:
        if (m_rxLength >= sizeof(FilterParams_t)) {
            const FilterParams_t *filter = reinterpret_cast<const FilterParams_t*>(data);
            m_filterParams.angle_ref_filter = filter->angle_ref_filter;
            m_filterParams.angle_fdb_filter = filter->angle_fdb_filter;
            m_filterParams.pressure_fdb_filter = filter->pressure_fdb_filter;
            m_filterParams.current_fdb_filter = filter->current_fdb_filter;
            emit filterParamsReceived(m_filterParams);
        }
        break;

    default:
        qDebug() << "Unknown command received:" << Qt::hex << m_rxCmd;
        break;
    }
}

void SerialCommunication::onErrorOccurred(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::NoError) {
        return;
    }

    QString errorStr = m_serialPort->errorString();
    emit errorOccurred(errorStr);

    if (error == QSerialPort::ResourceError) {
        disconnect();
    }
}

void SerialCommunication::onHeartbeatTimeout()
{
    sendHeartbeat();
}

// Request methods
void SerialCommunication::requestFeedbackData()
{
    sendFrame(CMD_GET_ALL_FEEDBACK, nullptr, 0);
}

void SerialCommunication::requestSystemStatus()
{
    sendFrame(CMD_GET_STATUS, nullptr, 0);
}

void SerialCommunication::requestAnalogInputs()
{
    sendFrame(CMD_GET_ANALOG_INPUT, nullptr, 0);
}

void SerialCommunication::requestAnglePID()
{
    sendFrame(CMD_GET_PID_ANGLE, nullptr, 0);
}

void SerialCommunication::requestPressurePID()
{
    sendFrame(CMD_GET_PID_PRESSURE, nullptr, 0);
}

void SerialCommunication::requestCalibration()
{
    sendFrame(CMD_GET_CALIBRATION, nullptr, 0);
}

void SerialCommunication::requestFilterParams()
{
    sendFrame(CMD_GET_FILTER_PARAMS, nullptr, 0);
}

// Control methods
void SerialCommunication::setAngleReference(int32_t angle)
{
    sendFrame(CMD_SET_ANGLE_REF, reinterpret_cast<const uint8_t*>(&angle), 4);
}

void SerialCommunication::setPressureReference(int32_t pressure)
{
    sendFrame(CMD_SET_PRESSURE_REF, reinterpret_cast<const uint8_t*>(&pressure), 4);
}

void SerialCommunication::setCurrentReferences(int32_t currentA, int32_t currentB)
{
    uint8_t payload[8];
    std::memcpy(&payload[0], &currentA, 4);
    std::memcpy(&payload[4], &currentB, 4);
    sendFrame(CMD_SET_CURRENT_REF, payload, 8);
}

void SerialCommunication::setEnableFlags(uint8_t flags)
{
    sendFrame(CMD_SET_ENABLE_FLAGS, &flags, 1);
}

// PID tuning
void SerialCommunication::setAnglePID(const PIDParameters &params)
{
    PIDParams_t pid;
    pid.kp = params.kp;
    pid.kp_div = params.kp_div;
    pid.ki = params.ki;
    pid.ki_div = params.ki_div;
    pid.kd = params.kd;
    pid.kd_div = params.kd_div;
    pid.kv = params.kv;
    pid.kv_div = params.kv_div;
    pid.output_max = params.output_max;
    pid.output_min = params.output_min;

    sendFrame(CMD_SET_PID_ANGLE, reinterpret_cast<const uint8_t*>(&pid), sizeof(pid));
}

void SerialCommunication::setPressurePID(const PIDParameters &params)
{
    PIDParams_t pid;
    pid.kp = params.kp;
    pid.kp_div = params.kp_div;
    pid.ki = params.ki;
    pid.ki_div = params.ki_div;
    pid.kd = params.kd;
    pid.kd_div = params.kd_div;
    pid.kv = params.kv;
    pid.kv_div = params.kv_div;
    pid.output_max = params.output_max;
    pid.output_min = params.output_min;

    sendFrame(CMD_SET_PID_PRESSURE, reinterpret_cast<const uint8_t*>(&pid), sizeof(pid));
}

// Calibration
void SerialCommunication::setCalibration(const CalibrationParameters &params)
{
    CalibrationParams_t cal;
    cal.angle_min_adc = params.angle_min_adc;
    cal.angle_mid_adc = params.angle_mid_adc;
    cal.angle_max_adc = params.angle_max_adc;
    cal.angle_range = params.angle_range;
    cal.pressure_min_adc = params.pressure_min_adc;
    cal.pressure_mid_adc = params.pressure_mid_adc;
    cal.pressure_max_adc = params.pressure_max_adc;
    cal.pressure_max_bar = params.pressure_max_bar;

    sendFrame(CMD_SET_CALIBRATION, reinterpret_cast<const uint8_t*>(&cal), sizeof(cal));
}

void SerialCommunication::setFilterParams(const FilterParameters &params)
{
    FilterParams_t filter;
    filter.angle_ref_filter = params.angle_ref_filter;
    filter.angle_fdb_filter = params.angle_fdb_filter;
    filter.pressure_fdb_filter = params.pressure_fdb_filter;
    filter.current_fdb_filter = params.current_fdb_filter;

    sendFrame(CMD_SET_FILTER_PARAMS, reinterpret_cast<const uint8_t*>(&filter), sizeof(filter));
}

// Streaming
void SerialCommunication::startStreaming(uint8_t rateMs, uint8_t dataMask)
{
    StreamConfig_t config;
    config.enable = 1;
    config.rate_ms = rateMs;
    config.data_mask = dataMask;
    config.reserved = 0;

    sendFrame(CMD_START_STREAM, reinterpret_cast<const uint8_t*>(&config), sizeof(config));
}

void SerialCommunication::stopStreaming()
{
    sendFrame(CMD_STOP_STREAM, nullptr, 0);
}

// Heartbeat
void SerialCommunication::sendHeartbeat()
{
    sendFrame(CMD_HEARTBEAT, nullptr, 0);
}
