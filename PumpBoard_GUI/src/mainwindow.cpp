/**
 * @file mainwindow.cpp
 * @brief Main Window Implementation
 */

#include "mainwindow.h"
#include "dashboardwidget.h"
#include "controlpanelwidget.h"
#include "pidtuningwidget.h"
#include "realtimechartwidget.h"
#include "datalogger.h"

#include <QToolBar>
#include <QMessageBox>
#include <QFileDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , m_serial(new SerialCommunication(this))
    , m_dataLogger(new DataLogger(this))
    , m_isLogging(false)
    , m_isStreaming(false)
{
    setWindowTitle("PumpBoard Controller GUI");
    setMinimumSize(1200, 800);

    setupUI();
    setupConnections();
    createToolBar();
    createStatusBar();

    // Status update timer
    m_statusTimer = new QTimer(this);
    connect(m_statusTimer, &QTimer::timeout, this, &MainWindow::updateStatusBar);
    m_statusTimer->start(500);

    // Initial port refresh
    onRefreshPorts();
}

MainWindow::~MainWindow()
{
    if (m_isLogging) {
        m_dataLogger->stopLogging();
    }
    if (m_serial->isConnected()) {
        m_serial->disconnect();
    }
}

void MainWindow::setupUI()
{
    // Create central widget
    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);

    // Create tab widget
    m_tabWidget = new QTabWidget(this);

    // Create widgets
    m_dashboardWidget = new DashboardWidget(this);
    m_controlPanelWidget = new ControlPanelWidget(m_serial, this);
    m_pidTuningWidget = new PIDTuningWidget(m_serial, this);
    m_chartWidget = new RealTimeChartWidget(this);

    // Add tabs
    m_tabWidget->addTab(m_dashboardWidget, "Dashboard");
    m_tabWidget->addTab(m_controlPanelWidget, "Control Panel");
    m_tabWidget->addTab(m_pidTuningWidget, "PID Tuning");
    m_tabWidget->addTab(m_chartWidget, "Charts");

    mainLayout->addWidget(m_tabWidget);
}

void MainWindow::setupConnections()
{
    // Serial communication signals
    connect(m_serial, &SerialCommunication::connectionStateChanged,
            this, &MainWindow::onConnectionStateChanged);
    connect(m_serial, &SerialCommunication::feedbackDataReceived,
            this, &MainWindow::onFeedbackDataReceived);
    connect(m_serial, &SerialCommunication::streamDataReceived,
            this, &MainWindow::onStreamDataReceived);
    connect(m_serial, &SerialCommunication::systemStatusReceived,
            this, &MainWindow::onSystemStatusReceived);
    connect(m_serial, &SerialCommunication::errorOccurred,
            this, &MainWindow::onErrorOccurred);

    // PID data
    connect(m_serial, &SerialCommunication::anglePIDReceived,
            m_pidTuningWidget, &PIDTuningWidget::updateAnglePID);
    connect(m_serial, &SerialCommunication::pressurePIDReceived,
            m_pidTuningWidget, &PIDTuningWidget::updatePressurePID);
    connect(m_serial, &SerialCommunication::calibrationReceived,
            m_pidTuningWidget, &PIDTuningWidget::updateCalibration);
    connect(m_serial, &SerialCommunication::filterParamsReceived,
            m_pidTuningWidget, &PIDTuningWidget::updateFilterParams);
}

void MainWindow::createToolBar()
{
    QToolBar *toolbar = addToolBar("Main Toolbar");
    toolbar->setMovable(false);

    // Port selection
    QLabel *portLabel = new QLabel("Port: ", this);
    toolbar->addWidget(portLabel);

    m_portComboBox = new QComboBox(this);
    m_portComboBox->setMinimumWidth(200);
    toolbar->addWidget(m_portComboBox);

    m_refreshButton = new QPushButton("Refresh", this);
    toolbar->addWidget(m_refreshButton);
    connect(m_refreshButton, &QPushButton::clicked, this, &MainWindow::onRefreshPorts);

    toolbar->addSeparator();

    // Baud rate
    QLabel *baudLabel = new QLabel("Baud: ", this);
    toolbar->addWidget(baudLabel);

    m_baudRateComboBox = new QComboBox(this);
    m_baudRateComboBox->addItems({"9600", "19200", "38400", "57600", "115200", "230400", "460800"});
    m_baudRateComboBox->setCurrentText("115200");
    toolbar->addWidget(m_baudRateComboBox);

    toolbar->addSeparator();

    // Connect button
    m_connectButton = new QPushButton("Connect", this);
    m_connectButton->setCheckable(true);
    toolbar->addWidget(m_connectButton);
    connect(m_connectButton, &QPushButton::clicked, this, &MainWindow::onConnectClicked);

    toolbar->addSeparator();

    // Stream button
    m_streamButton = new QPushButton("Start Stream", this);
    m_streamButton->setEnabled(false);
    toolbar->addWidget(m_streamButton);
    connect(m_streamButton, &QPushButton::clicked, this, [this]() {
        if (m_isStreaming) {
            m_serial->stopStreaming();
            m_streamButton->setText("Start Stream");
            m_isStreaming = false;
        } else {
            m_serial->startStreaming(50, 0xFF);  // 50ms rate, all data
            m_streamButton->setText("Stop Stream");
            m_isStreaming = true;
        }
    });

    toolbar->addSeparator();

    // Logging button
    m_logButton = new QPushButton("Start Log", this);
    toolbar->addWidget(m_logButton);
    connect(m_logButton, &QPushButton::clicked, this, [this]() {
        if (m_isLogging) {
            onStopLogging();
        } else {
            onStartLogging();
        }
    });
}

void MainWindow::createStatusBar()
{
    QStatusBar *status = statusBar();

    m_connectionStatusLabel = new QLabel("Disconnected", this);
    m_connectionStatusLabel->setStyleSheet("color: red; font-weight: bold;");
    status->addWidget(m_connectionStatusLabel);

    status->addWidget(new QLabel(" | ", this));

    m_rxCountLabel = new QLabel("RX: 0", this);
    status->addWidget(m_rxCountLabel);

    m_txCountLabel = new QLabel("TX: 0", this);
    status->addWidget(m_txCountLabel);

    m_errorCountLabel = new QLabel("Err: 0", this);
    status->addWidget(m_errorCountLabel);

    status->addWidget(new QLabel(" | ", this));

    m_timestampLabel = new QLabel("Time: 0 ms", this);
    status->addPermanentWidget(m_timestampLabel);
}

void MainWindow::updateStatusBar()
{
    if (m_serial->isConnected()) {
        m_rxCountLabel->setText(QString("RX: %1").arg(m_serial->rxFrameCount()));
        m_txCountLabel->setText(QString("TX: %1").arg(m_serial->txFrameCount()));
        m_errorCountLabel->setText(QString("Err: %1").arg(m_serial->errorCount()));
        m_timestampLabel->setText(QString("Time: %1 ms")
                                 .arg(m_serial->latestFeedback().timestamp_ms));
    }
}

void MainWindow::onRefreshPorts()
{
    m_portComboBox->clear();
    m_portComboBox->addItems(SerialCommunication::availablePorts());
}

void MainWindow::onConnectClicked()
{
    if (m_serial->isConnected()) {
        m_serial->disconnect();
        m_connectButton->setChecked(false);
    } else {
        QString port = m_portComboBox->currentText();
        qint32 baudRate = m_baudRateComboBox->currentText().toInt();

        if (port.isEmpty()) {
            QMessageBox::warning(this, "Error", "Please select a port");
            m_connectButton->setChecked(false);
            return;
        }

        if (!m_serial->connectToDevice(port, baudRate)) {
            m_connectButton->setChecked(false);
        }
    }
}

void MainWindow::onConnectionStateChanged(SerialCommunication::ConnectionState state)
{
    switch (state) {
    case SerialCommunication::Disconnected:
        m_connectionStatusLabel->setText("Disconnected");
        m_connectionStatusLabel->setStyleSheet("color: red; font-weight: bold;");
        m_connectButton->setText("Connect");
        m_connectButton->setChecked(false);
        m_streamButton->setEnabled(false);
        m_controlPanelWidget->setEnabled(false);
        m_isStreaming = false;
        m_streamButton->setText("Start Stream");
        break;

    case SerialCommunication::Connecting:
        m_connectionStatusLabel->setText("Connecting...");
        m_connectionStatusLabel->setStyleSheet("color: orange; font-weight: bold;");
        break;

    case SerialCommunication::Connected:
        m_connectionStatusLabel->setText("Connected");
        m_connectionStatusLabel->setStyleSheet("color: green; font-weight: bold;");
        m_connectButton->setText("Disconnect");
        m_connectButton->setChecked(true);
        m_streamButton->setEnabled(true);
        m_controlPanelWidget->setEnabled(true);
        break;
    }
}

void MainWindow::onFeedbackDataReceived(const FeedbackData &data)
{
    m_dashboardWidget->updateData(data);
    m_controlPanelWidget->updateFromFeedback(data);

    if (m_isLogging) {
        m_dataLogger->logData(data);
    }
}

void MainWindow::onStreamDataReceived(const FeedbackData &data)
{
    m_dashboardWidget->updateData(data);
    m_chartWidget->addDataPoint(data);
    m_controlPanelWidget->updateFromFeedback(data);

    if (m_isLogging) {
        m_dataLogger->logData(data);
    }
}

void MainWindow::onSystemStatusReceived(const SystemStatus &status)
{
    m_dashboardWidget->updateSystemStatus(status);
}

void MainWindow::onErrorOccurred(const QString &error)
{
    statusBar()->showMessage(QString("Error: %1").arg(error), 5000);
}

void MainWindow::onStartLogging()
{
    QString filename = QFileDialog::getSaveFileName(this,
        "Save Log File",
        QDir::homePath() + "/pumpboard_log_" +
            QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss") + ".csv",
        "CSV Files (*.csv);;All Files (*)");

    if (!filename.isEmpty()) {
        if (m_dataLogger->startLogging(filename)) {
            m_isLogging = true;
            m_logButton->setText("Stop Log");
            m_logButton->setStyleSheet("background-color: #ffcccc;");
        }
    }
}

void MainWindow::onStopLogging()
{
    m_dataLogger->stopLogging();
    m_isLogging = false;
    m_logButton->setText("Start Log");
    m_logButton->setStyleSheet("");

    QMessageBox::information(this, "Logging Complete",
        QString("Log saved: %1\nEntries: %2")
            .arg(m_dataLogger->currentFilename())
            .arg(m_dataLogger->entryCount()));
}
