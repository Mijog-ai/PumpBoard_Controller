/**
 * @file dashboardwidget.cpp
 * @brief Dashboard Widget Implementation
 */

#include "dashboardwidget.h"
#include "gaugewidget.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFont>

DashboardWidget::DashboardWidget(QWidget *parent)
    : QWidget(parent)
{
    setupUI();
}

void DashboardWidget::setupUI()
{
    QGridLayout *mainLayout = new QGridLayout(this);
    mainLayout->setSpacing(10);

    // Top row - Gauges
    mainLayout->addWidget(createAngleGroup(), 0, 0);
    mainLayout->addWidget(createPressureGroup(), 0, 1);

    // Middle row - Current and PWM
    mainLayout->addWidget(createCurrentGroup(), 1, 0);
    mainLayout->addWidget(createPWMGroup(), 1, 1);

    // Bottom row - Status
    mainLayout->addWidget(createStatusGroup(), 2, 0, 1, 2);

    mainLayout->setRowStretch(0, 2);
    mainLayout->setRowStretch(1, 2);
    mainLayout->setRowStretch(2, 1);
}

QGroupBox* DashboardWidget::createAngleGroup()
{
    QGroupBox *group = new QGroupBox("Swash Plate Angle", this);
    QVBoxLayout *layout = new QVBoxLayout(group);

    // Gauge
    m_angleGauge = new GaugeWidget(this);
    m_angleGauge->setRange(0, 100);
    m_angleGauge->setTitle("Angle");
    m_angleGauge->setUnit("%");
    m_angleGauge->setWarningThreshold(80);
    m_angleGauge->setCriticalThreshold(95);
    layout->addWidget(m_angleGauge, 1);

    // Progress bar
    m_angleBar = new QProgressBar(this);
    m_angleBar->setRange(0, 10000);
    m_angleBar->setTextVisible(true);
    m_angleBar->setFormat("%v / 10000");
    layout->addWidget(m_angleBar);

    // Values
    QGridLayout *valueLayout = new QGridLayout();

    valueLayout->addWidget(new QLabel("Feedback:", this), 0, 0);
    m_angleValueLabel = new QLabel("0", this);
    m_angleValueLabel->setFont(QFont("Monospace", 12, QFont::Bold));
    valueLayout->addWidget(m_angleValueLabel, 0, 1);

    valueLayout->addWidget(new QLabel("Reference:", this), 1, 0);
    m_angleRefLabel = new QLabel("0", this);
    m_angleRefLabel->setFont(QFont("Monospace", 12));
    valueLayout->addWidget(m_angleRefLabel, 1, 1);

    valueLayout->addWidget(new QLabel("Diff:", this), 2, 0);
    m_angleDiffLabel = new QLabel("0", this);
    m_angleDiffLabel->setFont(QFont("Monospace", 12));
    valueLayout->addWidget(m_angleDiffLabel, 2, 1);

    layout->addLayout(valueLayout);

    return group;
}

QGroupBox* DashboardWidget::createPressureGroup()
{
    QGroupBox *group = new QGroupBox("Hydraulic Pressure", this);
    QVBoxLayout *layout = new QVBoxLayout(group);

    // Gauge
    m_pressureGauge = new GaugeWidget(this);
    m_pressureGauge->setRange(0, 600);
    m_pressureGauge->setTitle("Pressure");
    m_pressureGauge->setUnit("bar");
    m_pressureGauge->setWarningThreshold(500);
    m_pressureGauge->setCriticalThreshold(550);
    layout->addWidget(m_pressureGauge, 1);

    // Progress bar
    m_pressureBar = new QProgressBar(this);
    m_pressureBar->setRange(0, 10000);
    m_pressureBar->setTextVisible(true);
    m_pressureBar->setFormat("%v / 10000");
    layout->addWidget(m_pressureBar);

    // Values
    QGridLayout *valueLayout = new QGridLayout();

    valueLayout->addWidget(new QLabel("Feedback:", this), 0, 0);
    m_pressureValueLabel = new QLabel("0", this);
    m_pressureValueLabel->setFont(QFont("Monospace", 12, QFont::Bold));
    valueLayout->addWidget(m_pressureValueLabel, 0, 1);

    m_pressureBarLabel = new QLabel("(0.0 bar)", this);
    valueLayout->addWidget(m_pressureBarLabel, 0, 2);

    valueLayout->addWidget(new QLabel("Reference:", this), 1, 0);
    m_pressureRefLabel = new QLabel("0", this);
    m_pressureRefLabel->setFont(QFont("Monospace", 12));
    valueLayout->addWidget(m_pressureRefLabel, 1, 1);

    valueLayout->addWidget(new QLabel("Diff:", this), 2, 0);
    m_pressureDiffLabel = new QLabel("0", this);
    m_pressureDiffLabel->setFont(QFont("Monospace", 12));
    valueLayout->addWidget(m_pressureDiffLabel, 2, 1);

    layout->addLayout(valueLayout);

    return group;
}

QGroupBox* DashboardWidget::createCurrentGroup()
{
    QGroupBox *group = new QGroupBox("Solenoid Current", this);
    QGridLayout *layout = new QGridLayout(group);

    // Current A
    layout->addWidget(new QLabel("Current A:", this), 0, 0);
    m_currentAValueLabel = new QLabel("0 mA", this);
    m_currentAValueLabel->setFont(QFont("Monospace", 12, QFont::Bold));
    layout->addWidget(m_currentAValueLabel, 0, 1);

    layout->addWidget(new QLabel("Ref:", this), 0, 2);
    m_currentARefLabel = new QLabel("0 mA", this);
    layout->addWidget(m_currentARefLabel, 0, 3);

    m_currentABar = new QProgressBar(this);
    m_currentABar->setRange(0, 3000);
    m_currentABar->setFormat("%v mA");
    layout->addWidget(m_currentABar, 1, 0, 1, 4);

    // Current B
    layout->addWidget(new QLabel("Current B:", this), 2, 0);
    m_currentBValueLabel = new QLabel("0 mA", this);
    m_currentBValueLabel->setFont(QFont("Monospace", 12, QFont::Bold));
    layout->addWidget(m_currentBValueLabel, 2, 1);

    layout->addWidget(new QLabel("Ref:", this), 2, 2);
    m_currentBRefLabel = new QLabel("0 mA", this);
    layout->addWidget(m_currentBRefLabel, 2, 3);

    m_currentBBar = new QProgressBar(this);
    m_currentBBar->setRange(0, 3000);
    m_currentBBar->setFormat("%v mA");
    layout->addWidget(m_currentBBar, 3, 0, 1, 4);

    return group;
}

QGroupBox* DashboardWidget::createPWMGroup()
{
    QGroupBox *group = new QGroupBox("PWM Output", this);
    QGridLayout *layout = new QGridLayout(group);

    // PWM A
    layout->addWidget(new QLabel("PWM A:", this), 0, 0);
    m_pwmAValueLabel = new QLabel("0 (0.0%)", this);
    m_pwmAValueLabel->setFont(QFont("Monospace", 12, QFont::Bold));
    layout->addWidget(m_pwmAValueLabel, 0, 1);

    m_pwmABar = new QProgressBar(this);
    m_pwmABar->setRange(0, 105000);
    m_pwmABar->setFormat("%p%");
    layout->addWidget(m_pwmABar, 1, 0, 1, 2);

    // PWM B
    layout->addWidget(new QLabel("PWM B:", this), 2, 0);
    m_pwmBValueLabel = new QLabel("0 (0.0%)", this);
    m_pwmBValueLabel->setFont(QFont("Monospace", 12, QFont::Bold));
    layout->addWidget(m_pwmBValueLabel, 2, 1);

    m_pwmBBar = new QProgressBar(this);
    m_pwmBBar->setRange(0, 105000);
    m_pwmBBar->setFormat("%p%");
    layout->addWidget(m_pwmBBar, 3, 0, 1, 2);

    return group;
}

QGroupBox* DashboardWidget::createStatusGroup()
{
    QGroupBox *group = new QGroupBox("System Status", this);
    QHBoxLayout *layout = new QHBoxLayout(group);

    // Control enables
    QGroupBox *enablesBox = new QGroupBox("Control Enables", this);
    QGridLayout *enablesLayout = new QGridLayout(enablesBox);

    enablesLayout->addWidget(new QLabel("Pump Start:", this), 0, 0);
    m_pumpStartStatus = new QLabel("OFF", this);
    m_pumpStartStatus->setStyleSheet("color: red; font-weight: bold;");
    enablesLayout->addWidget(m_pumpStartStatus, 0, 1);

    enablesLayout->addWidget(new QLabel("Angle Leak:", this), 1, 0);
    m_angleleakStatus = new QLabel("OFF", this);
    m_angleleakStatus->setStyleSheet("color: red; font-weight: bold;");
    enablesLayout->addWidget(m_angleleakStatus, 1, 1);

    enablesLayout->addWidget(new QLabel("Pressure Loop:", this), 0, 2);
    m_pressureLoopStatus = new QLabel("OFF", this);
    m_pressureLoopStatus->setStyleSheet("color: red; font-weight: bold;");
    enablesLayout->addWidget(m_pressureLoopStatus, 0, 3);

    enablesLayout->addWidget(new QLabel("Power Loop:", this), 1, 2);
    m_powerLoopStatus = new QLabel("OFF", this);
    m_powerLoopStatus->setStyleSheet("color: red; font-weight: bold;");
    enablesLayout->addWidget(m_powerLoopStatus, 1, 3);

    layout->addWidget(enablesBox);

    // System info
    QGroupBox *infoBox = new QGroupBox("System Info", this);
    QGridLayout *infoLayout = new QGridLayout(infoBox);

    infoLayout->addWidget(new QLabel("Firmware:", this), 0, 0);
    m_firmwareVersionLabel = new QLabel("--", this);
    infoLayout->addWidget(m_firmwareVersionLabel, 0, 1);

    infoLayout->addWidget(new QLabel("Mode:", this), 1, 0);
    m_simulationModeLabel = new QLabel("--", this);
    infoLayout->addWidget(m_simulationModeLabel, 1, 1);

    infoLayout->addWidget(new QLabel("Error:", this), 0, 2);
    m_errorCodeLabel = new QLabel("0x00", this);
    infoLayout->addWidget(m_errorCodeLabel, 0, 3);

    infoLayout->addWidget(new QLabel("Timestamp:", this), 1, 2);
    m_timestampLabel = new QLabel("0 ms", this);
    infoLayout->addWidget(m_timestampLabel, 1, 3);

    layout->addWidget(infoBox);

    return group;
}

void DashboardWidget::updateData(const FeedbackData &data)
{
    // Angle
    double anglePercent = data.angle_feedback / 100.0;
    m_angleGauge->setValue(anglePercent);
    m_angleBar->setValue(data.angle_feedback);
    m_angleValueLabel->setText(QString::number(data.angle_feedback));
    m_angleRefLabel->setText(QString::number(data.angle_ref));
    m_angleDiffLabel->setText(QString::number(data.angle_diff));

    // Pressure
    double pressureBar = data.pressure_feedback / 10000.0 * 600.0;
    m_pressureGauge->setValue(pressureBar);
    m_pressureBar->setValue(data.pressure_feedback);
    m_pressureValueLabel->setText(QString::number(data.pressure_feedback));
    m_pressureBarLabel->setText(QString("(%1 bar)").arg(pressureBar, 0, 'f', 1));
    m_pressureRefLabel->setText(QString::number(data.pressure_ref));
    m_pressureDiffLabel->setText(QString::number(data.pressure_diff));

    // Current A
    m_currentAValueLabel->setText(QString("%1 mA").arg(data.current_a_feedback));
    m_currentARefLabel->setText(QString("%1 mA").arg(data.current_a_ref));
    m_currentABar->setValue(qAbs(data.current_a_feedback));

    // Current B
    m_currentBValueLabel->setText(QString("%1 mA").arg(data.current_b_feedback));
    m_currentBRefLabel->setText(QString("%1 mA").arg(data.current_b_ref));
    m_currentBBar->setValue(qAbs(data.current_b_feedback));

    // PWM
    double pwmAPercent = data.pwm_output_a / 105000.0 * 100.0;
    double pwmBPercent = data.pwm_output_b / 105000.0 * 100.0;
    m_pwmAValueLabel->setText(QString("%1 (%2%)").arg(data.pwm_output_a).arg(pwmAPercent, 0, 'f', 1));
    m_pwmBValueLabel->setText(QString("%1 (%2%)").arg(data.pwm_output_b).arg(pwmBPercent, 0, 'f', 1));
    m_pwmABar->setValue(data.pwm_output_a);
    m_pwmBBar->setValue(data.pwm_output_b);

    // Enable flags
    auto setStatus = [](QLabel *label, bool enabled) {
        label->setText(enabled ? "ON" : "OFF");
        label->setStyleSheet(enabled ? "color: green; font-weight: bold;" :
                                       "color: red; font-weight: bold;");
    };

    setStatus(m_pumpStartStatus, data.enable_flags & ENABLE_PUMP_START);
    setStatus(m_angleleakStatus, data.enable_flags & ENABLE_ANGLE_LEAK);
    setStatus(m_pressureLoopStatus, data.enable_flags & ENABLE_PRESSURE_LOOP);
    setStatus(m_powerLoopStatus, data.enable_flags & ENABLE_POWER_LOOP);

    // Error code
    m_errorCodeLabel->setText(QString("0x%1").arg(data.error_code, 2, 16, QChar('0')));
    if (data.error_code != 0) {
        m_errorCodeLabel->setStyleSheet("color: red; font-weight: bold;");
    } else {
        m_errorCodeLabel->setStyleSheet("color: green;");
    }

    // Timestamp
    m_timestampLabel->setText(QString("%1 ms").arg(data.timestamp_ms));
}

void DashboardWidget::updateSystemStatus(const SystemStatus &status)
{
    m_firmwareVersionLabel->setText(QString("v%1.%2.%3")
                                    .arg(status.version_major)
                                    .arg(status.version_minor)
                                    .arg(status.version_patch));

    m_simulationModeLabel->setText(status.simulation_mode ? "SIMULATION" : "NORMAL");
    m_simulationModeLabel->setStyleSheet(status.simulation_mode ?
                                         "color: orange; font-weight: bold;" : "");
}
