/**
 * @file datalogger.h
 * @brief Data Logger for Recording and Exporting Sensor Data
 */

#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <QObject>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QVector>
#include "serialcommunication.h"

struct LogEntry {
    QDateTime timestamp;
    FeedbackData data;
};

class DataLogger : public QObject
{
    Q_OBJECT

public:
    explicit DataLogger(QObject *parent = nullptr);
    ~DataLogger();

    bool startLogging(const QString &filename = QString());
    void stopLogging();
    bool isLogging() const { return m_isLogging; }

    void logData(const FeedbackData &data);

    bool exportToCSV(const QString &filename) const;
    bool exportToJSON(const QString &filename) const;

    void clearLog();
    int entryCount() const { return m_entries.size(); }
    const QVector<LogEntry>& entries() const { return m_entries; }

    QString currentFilename() const { return m_filename; }

signals:
    void loggingStarted(const QString &filename);
    void loggingStopped();
    void entryLogged(int count);
    void errorOccurred(const QString &error);

private:
    void writeHeader();
    void writeEntry(const FeedbackData &data);

    QFile *m_file;
    QTextStream *m_stream;
    QString m_filename;
    bool m_isLogging;
    QVector<LogEntry> m_entries;
    QDateTime m_startTime;
};

#endif // DATALOGGER_H
