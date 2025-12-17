/**
 * @file mainwindow.h
 * @brief Main Window for PumpBoard GUI Application
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTabWidget>
#include <QStatusBar>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QTimer>
#include "serialcommunication.h"

class DashboardWidget;
class ControlPanelWidget;
class PIDTuningWidget;
class RealTimeChartWidget;
class DataLogger;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onConnectClicked();
    void onConnectionStateChanged(SerialCommunication::ConnectionState state);
    void onFeedbackDataReceived(const FeedbackData &data);
    void onStreamDataReceived(const FeedbackData &data);
    void onSystemStatusReceived(const SystemStatus &status);
    void onErrorOccurred(const QString &error);
    void onRefreshPorts();
    void onStartLogging();
    void onStopLogging();

private:
    void setupUI();
    void setupConnections();
    void createToolBar();
    void createStatusBar();
    void updateStatusBar();

    // UI Components
    QTabWidget *m_tabWidget;
    DashboardWidget *m_dashboardWidget;
    ControlPanelWidget *m_controlPanelWidget;
    PIDTuningWidget *m_pidTuningWidget;
    RealTimeChartWidget *m_chartWidget;

    // Toolbar components
    QComboBox *m_portComboBox;
    QComboBox *m_baudRateComboBox;
    QPushButton *m_connectButton;
    QPushButton *m_refreshButton;
    QPushButton *m_streamButton;
    QPushButton *m_logButton;

    // Status bar labels
    QLabel *m_connectionStatusLabel;
    QLabel *m_rxCountLabel;
    QLabel *m_txCountLabel;
    QLabel *m_errorCountLabel;
    QLabel *m_timestampLabel;

    // Communication
    SerialCommunication *m_serial;

    // Data logging
    DataLogger *m_dataLogger;
    bool m_isLogging;
    bool m_isStreaming;

    // Update timer for status
    QTimer *m_statusTimer;
};

#endif // MAINWINDOW_H
