/**
 * @file realtimechartwidget.cpp
 * @brief Real-time Chart Widget Implementation
 */

#include "realtimechartwidget.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>

RealTimeChartWidget::RealTimeChartWidget(QWidget *parent)
    : QWidget(parent)
    , m_isPaused(false)
    , m_timeWindow(30.0)
    , m_maxPoints(3000)
{
    setupUI();
    setupChart();
    m_elapsedTimer.start();
}

void RealTimeChartWidget::setupUI()
{
    QVBoxLayout *mainLayout = new QVBoxLayout(this);

    // Chart view
    m_chart = new QChart();
    m_chartView = new QChartView(m_chart, this);
    m_chartView->setRenderHint(QPainter::Antialiasing);
    mainLayout->addWidget(m_chartView, 1);

    // Controls
    QHBoxLayout *controlLayout = new QHBoxLayout();

    // Channel visibility
    QGroupBox *channelGroup = new QGroupBox("Channels", this);
    QHBoxLayout *channelLayout = new QHBoxLayout(channelGroup);

    m_showAngleFdb = new QCheckBox("Angle Fdb", this);
    m_showAngleFdb->setChecked(true);
    m_showAngleFdb->setStyleSheet("color: blue;");
    connect(m_showAngleFdb, &QCheckBox::toggled, this, &RealTimeChartWidget::onChannelToggled);
    channelLayout->addWidget(m_showAngleFdb);

    m_showAngleRef = new QCheckBox("Angle Ref", this);
    m_showAngleRef->setChecked(true);
    m_showAngleRef->setStyleSheet("color: cyan;");
    connect(m_showAngleRef, &QCheckBox::toggled, this, &RealTimeChartWidget::onChannelToggled);
    channelLayout->addWidget(m_showAngleRef);

    m_showPressureFdb = new QCheckBox("Pressure Fdb", this);
    m_showPressureFdb->setChecked(true);
    m_showPressureFdb->setStyleSheet("color: red;");
    connect(m_showPressureFdb, &QCheckBox::toggled, this, &RealTimeChartWidget::onChannelToggled);
    channelLayout->addWidget(m_showPressureFdb);

    m_showPressureRef = new QCheckBox("Pressure Ref", this);
    m_showPressureRef->setChecked(true);
    m_showPressureRef->setStyleSheet("color: orange;");
    connect(m_showPressureRef, &QCheckBox::toggled, this, &RealTimeChartWidget::onChannelToggled);
    channelLayout->addWidget(m_showPressureRef);

    m_showCurrentA = new QCheckBox("Current A", this);
    m_showCurrentA->setStyleSheet("color: green;");
    connect(m_showCurrentA, &QCheckBox::toggled, this, &RealTimeChartWidget::onChannelToggled);
    channelLayout->addWidget(m_showCurrentA);

    m_showCurrentB = new QCheckBox("Current B", this);
    m_showCurrentB->setStyleSheet("color: purple;");
    connect(m_showCurrentB, &QCheckBox::toggled, this, &RealTimeChartWidget::onChannelToggled);
    channelLayout->addWidget(m_showCurrentB);

    controlLayout->addWidget(channelGroup);

    // Time window
    controlLayout->addWidget(new QLabel("Time Window:", this));
    m_timeWindowSpinBox = new QSpinBox(this);
    m_timeWindowSpinBox->setRange(5, 300);
    m_timeWindowSpinBox->setValue(30);
    m_timeWindowSpinBox->setSuffix(" s");
    connect(m_timeWindowSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &RealTimeChartWidget::onTimeWindowChanged);
    controlLayout->addWidget(m_timeWindowSpinBox);

    controlLayout->addStretch();

    // Buttons
    m_pauseButton = new QPushButton("Pause", this);
    connect(m_pauseButton, &QPushButton::clicked, this, &RealTimeChartWidget::onPauseClicked);
    controlLayout->addWidget(m_pauseButton);

    m_clearButton = new QPushButton("Clear", this);
    connect(m_clearButton, &QPushButton::clicked, this, &RealTimeChartWidget::onClearClicked);
    controlLayout->addWidget(m_clearButton);

    mainLayout->addLayout(controlLayout);
}

void RealTimeChartWidget::setupChart()
{
    m_chart->setTitle("Real-time Data");
    m_chart->legend()->setVisible(true);
    m_chart->legend()->setAlignment(Qt::AlignBottom);

    // Create series
    m_angleFdbSeries = new QLineSeries(this);
    m_angleFdbSeries->setName("Angle Fdb");
    m_angleFdbSeries->setColor(Qt::blue);

    m_angleRefSeries = new QLineSeries(this);
    m_angleRefSeries->setName("Angle Ref");
    m_angleRefSeries->setColor(Qt::cyan);

    m_pressureFdbSeries = new QLineSeries(this);
    m_pressureFdbSeries->setName("Pressure Fdb");
    m_pressureFdbSeries->setColor(Qt::red);

    m_pressureRefSeries = new QLineSeries(this);
    m_pressureRefSeries->setName("Pressure Ref");
    m_pressureRefSeries->setColor(QColor(255, 165, 0));  // Orange

    m_currentASeries = new QLineSeries(this);
    m_currentASeries->setName("Current A");
    m_currentASeries->setColor(Qt::green);

    m_currentBSeries = new QLineSeries(this);
    m_currentBSeries->setName("Current B");
    m_currentBSeries->setColor(QColor(128, 0, 128));  // Purple

    // Add series to chart
    m_chart->addSeries(m_angleFdbSeries);
    m_chart->addSeries(m_angleRefSeries);
    m_chart->addSeries(m_pressureFdbSeries);
    m_chart->addSeries(m_pressureRefSeries);
    m_chart->addSeries(m_currentASeries);
    m_chart->addSeries(m_currentBSeries);

    // Create axes
    m_timeAxis = new QValueAxis(this);
    m_timeAxis->setTitleText("Time (s)");
    m_timeAxis->setRange(0, m_timeWindow);
    m_chart->addAxis(m_timeAxis, Qt::AlignBottom);

    m_valueAxis = new QValueAxis(this);
    m_valueAxis->setTitleText("Value (%)");
    m_valueAxis->setRange(0, 100);
    m_chart->addAxis(m_valueAxis, Qt::AlignLeft);

    // Attach series to axes
    m_angleFdbSeries->attachAxis(m_timeAxis);
    m_angleFdbSeries->attachAxis(m_valueAxis);
    m_angleRefSeries->attachAxis(m_timeAxis);
    m_angleRefSeries->attachAxis(m_valueAxis);
    m_pressureFdbSeries->attachAxis(m_timeAxis);
    m_pressureFdbSeries->attachAxis(m_valueAxis);
    m_pressureRefSeries->attachAxis(m_timeAxis);
    m_pressureRefSeries->attachAxis(m_valueAxis);
    m_currentASeries->attachAxis(m_timeAxis);
    m_currentASeries->attachAxis(m_valueAxis);
    m_currentBSeries->attachAxis(m_timeAxis);
    m_currentBSeries->attachAxis(m_valueAxis);

    // Initially hide current series
    m_currentASeries->setVisible(false);
    m_currentBSeries->setVisible(false);
}

void RealTimeChartWidget::addDataPoint(const FeedbackData &data)
{
    if (m_isPaused) return;

    double time = m_elapsedTimer.elapsed() / 1000.0;

    // Store data
    m_timeData.append(time);
    m_angleFdbData.append(data.angle_feedback / 100.0);
    m_angleRefData.append(data.angle_ref / 100.0);
    m_pressureFdbData.append(data.pressure_feedback / 100.0);
    m_pressureRefData.append(data.pressure_ref / 100.0);
    m_currentAData.append(data.current_a_feedback / 30.0);  // Scale to 0-100%
    m_currentBData.append(data.current_b_feedback / 30.0);

    // Remove old data points
    while (m_timeData.size() > m_maxPoints) {
        m_timeData.removeFirst();
        m_angleFdbData.removeFirst();
        m_angleRefData.removeFirst();
        m_pressureFdbData.removeFirst();
        m_pressureRefData.removeFirst();
        m_currentAData.removeFirst();
        m_currentBData.removeFirst();
    }

    updateChart();
}

void RealTimeChartWidget::updateChart()
{
    if (m_timeData.isEmpty()) return;

    // Clear and rebuild series
    m_angleFdbSeries->clear();
    m_angleRefSeries->clear();
    m_pressureFdbSeries->clear();
    m_pressureRefSeries->clear();
    m_currentASeries->clear();
    m_currentBSeries->clear();

    double currentTime = m_timeData.last();
    double startTime = currentTime - m_timeWindow;

    for (int i = 0; i < m_timeData.size(); ++i) {
        if (m_timeData[i] >= startTime) {
            double t = m_timeData[i] - startTime;
            m_angleFdbSeries->append(t, m_angleFdbData[i]);
            m_angleRefSeries->append(t, m_angleRefData[i]);
            m_pressureFdbSeries->append(t, m_pressureFdbData[i]);
            m_pressureRefSeries->append(t, m_pressureRefData[i]);
            m_currentASeries->append(t, m_currentAData[i]);
            m_currentBSeries->append(t, m_currentBData[i]);
        }
    }

    m_timeAxis->setRange(0, m_timeWindow);
}

void RealTimeChartWidget::clearData()
{
    m_timeData.clear();
    m_angleFdbData.clear();
    m_angleRefData.clear();
    m_pressureFdbData.clear();
    m_pressureRefData.clear();
    m_currentAData.clear();
    m_currentBData.clear();

    m_angleFdbSeries->clear();
    m_angleRefSeries->clear();
    m_pressureFdbSeries->clear();
    m_pressureRefSeries->clear();
    m_currentASeries->clear();
    m_currentBSeries->clear();

    m_elapsedTimer.restart();
}

void RealTimeChartWidget::onChannelToggled(bool checked)
{
    Q_UNUSED(checked);

    m_angleFdbSeries->setVisible(m_showAngleFdb->isChecked());
    m_angleRefSeries->setVisible(m_showAngleRef->isChecked());
    m_pressureFdbSeries->setVisible(m_showPressureFdb->isChecked());
    m_pressureRefSeries->setVisible(m_showPressureRef->isChecked());
    m_currentASeries->setVisible(m_showCurrentA->isChecked());
    m_currentBSeries->setVisible(m_showCurrentB->isChecked());
}

void RealTimeChartWidget::onTimeWindowChanged(int seconds)
{
    m_timeWindow = seconds;
    updateChart();
}

void RealTimeChartWidget::onClearClicked()
{
    clearData();
}

void RealTimeChartWidget::onPauseClicked()
{
    m_isPaused = !m_isPaused;
    m_pauseButton->setText(m_isPaused ? "Resume" : "Pause");
}
