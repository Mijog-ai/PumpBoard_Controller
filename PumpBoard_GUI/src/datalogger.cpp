/**
 * @file datalogger.cpp
 * @brief Data Logger Implementation
 */

#include "datalogger.h"
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QDir>

DataLogger::DataLogger(QObject *parent)
    : QObject(parent)
    , m_file(nullptr)
    , m_stream(nullptr)
    , m_isLogging(false)
{
}

DataLogger::~DataLogger()
{
    if (m_isLogging) {
        stopLogging();
    }
}

bool DataLogger::startLogging(const QString &filename)
{
    if (m_isLogging) {
        stopLogging();
    }

    m_filename = filename;
    if (m_filename.isEmpty()) {
        m_filename = QDir::homePath() + "/pumpboard_log_" +
                     QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss") + ".csv";
    }

    m_file = new QFile(m_filename);
    if (!m_file->open(QIODevice::WriteOnly | QIODevice::Text)) {
        emit errorOccurred(QString("Failed to open file: %1").arg(m_file->errorString()));
        delete m_file;
        m_file = nullptr;
        return false;
    }

    m_stream = new QTextStream(m_file);
    m_entries.clear();
    m_startTime = QDateTime::currentDateTime();
    m_isLogging = true;

    writeHeader();
    emit loggingStarted(m_filename);

    return true;
}

void DataLogger::stopLogging()
{
    if (!m_isLogging) return;

    m_isLogging = false;

    if (m_stream) {
        m_stream->flush();
        delete m_stream;
        m_stream = nullptr;
    }

    if (m_file) {
        m_file->close();
        delete m_file;
        m_file = nullptr;
    }

    emit loggingStopped();
}

void DataLogger::writeHeader()
{
    if (!m_stream) return;

    *m_stream << "Timestamp,TimestampMS,AngleFdb,AngleRef,AngleDiff,"
              << "PressureFdb,PressureRef,PressureDiff,"
              << "CurrentAFdb,CurrentARef,CurrentBFdb,CurrentBRef,"
              << "PWM_A,PWM_B,EnableFlags,ErrorCode\n";
}

void DataLogger::writeEntry(const FeedbackData &data)
{
    if (!m_stream) return;

    QDateTime now = QDateTime::currentDateTime();

    *m_stream << now.toString(Qt::ISODateWithMs) << ","
              << data.timestamp_ms << ","
              << data.angle_feedback << ","
              << data.angle_ref << ","
              << data.angle_diff << ","
              << data.pressure_feedback << ","
              << data.pressure_ref << ","
              << data.pressure_diff << ","
              << data.current_a_feedback << ","
              << data.current_a_ref << ","
              << data.current_b_feedback << ","
              << data.current_b_ref << ","
              << data.pwm_output_a << ","
              << data.pwm_output_b << ","
              << (int)data.enable_flags << ","
              << (int)data.error_code << "\n";

    m_stream->flush();
}

void DataLogger::logData(const FeedbackData &data)
{
    // Store in memory
    LogEntry entry;
    entry.timestamp = QDateTime::currentDateTime();
    entry.data = data;
    m_entries.append(entry);

    // Write to file if logging
    if (m_isLogging && m_stream) {
        writeEntry(data);
    }

    emit entryLogged(m_entries.size());
}

bool DataLogger::exportToCSV(const QString &filename) const
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return false;
    }

    QTextStream stream(&file);

    // Header
    stream << "Timestamp,TimestampMS,AngleFdb,AngleRef,AngleDiff,"
           << "PressureFdb,PressureRef,PressureDiff,"
           << "CurrentAFdb,CurrentARef,CurrentBFdb,CurrentBRef,"
           << "PWM_A,PWM_B,EnableFlags,ErrorCode\n";

    // Data
    for (const LogEntry &entry : m_entries) {
        stream << entry.timestamp.toString(Qt::ISODateWithMs) << ","
               << entry.data.timestamp_ms << ","
               << entry.data.angle_feedback << ","
               << entry.data.angle_ref << ","
               << entry.data.angle_diff << ","
               << entry.data.pressure_feedback << ","
               << entry.data.pressure_ref << ","
               << entry.data.pressure_diff << ","
               << entry.data.current_a_feedback << ","
               << entry.data.current_a_ref << ","
               << entry.data.current_b_feedback << ","
               << entry.data.current_b_ref << ","
               << entry.data.pwm_output_a << ","
               << entry.data.pwm_output_b << ","
               << (int)entry.data.enable_flags << ","
               << (int)entry.data.error_code << "\n";
    }

    file.close();
    return true;
}

bool DataLogger::exportToJSON(const QString &filename) const
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return false;
    }

    QJsonArray dataArray;

    for (const LogEntry &entry : m_entries) {
        QJsonObject obj;
        obj["timestamp"] = entry.timestamp.toString(Qt::ISODateWithMs);
        obj["timestamp_ms"] = (qint64)entry.data.timestamp_ms;
        obj["angle_feedback"] = entry.data.angle_feedback;
        obj["angle_ref"] = entry.data.angle_ref;
        obj["angle_diff"] = entry.data.angle_diff;
        obj["pressure_feedback"] = entry.data.pressure_feedback;
        obj["pressure_ref"] = entry.data.pressure_ref;
        obj["pressure_diff"] = entry.data.pressure_diff;
        obj["current_a_feedback"] = entry.data.current_a_feedback;
        obj["current_a_ref"] = entry.data.current_a_ref;
        obj["current_b_feedback"] = entry.data.current_b_feedback;
        obj["current_b_ref"] = entry.data.current_b_ref;
        obj["pwm_output_a"] = (qint64)entry.data.pwm_output_a;
        obj["pwm_output_b"] = (qint64)entry.data.pwm_output_b;
        obj["enable_flags"] = entry.data.enable_flags;
        obj["error_code"] = entry.data.error_code;

        dataArray.append(obj);
    }

    QJsonObject root;
    root["start_time"] = m_startTime.toString(Qt::ISODateWithMs);
    root["entry_count"] = m_entries.size();
    root["data"] = dataArray;

    QJsonDocument doc(root);
    file.write(doc.toJson(QJsonDocument::Indented));
    file.close();

    return true;
}

void DataLogger::clearLog()
{
    m_entries.clear();
}
