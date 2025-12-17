/**
 * @file pidtuningwidget.cpp
 * @brief PID Tuning Widget Implementation
 */

#include "pidtuningwidget.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFormLayout>
#include <QMessageBox>

PIDTuningWidget::PIDTuningWidget(SerialCommunication *serial, QWidget *parent)
    : QWidget(parent)
    , m_serial(serial)
{
    setupUI();
}

void PIDTuningWidget::setupUI()
{
    QVBoxLayout *mainLayout = new QVBoxLayout(this);

    m_tabWidget = new QTabWidget(this);
    m_tabWidget->addTab(createAnglePIDTab(), "Angle PID");
    m_tabWidget->addTab(createPressurePIDTab(), "Pressure PID");
    m_tabWidget->addTab(createCalibrationTab(), "Calibration");
    m_tabWidget->addTab(createFiltersTab(), "Filters");

    mainLayout->addWidget(m_tabWidget);
}

QWidget* PIDTuningWidget::createAnglePIDTab()
{
    QWidget *widget = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(widget);

    QGroupBox *paramGroup = new QGroupBox("Angle Loop PID Parameters", this);
    QGridLayout *grid = new QGridLayout(paramGroup);

    int row = 0;

    // Kp
    grid->addWidget(new QLabel("Kp:", this), row, 0);
    m_angleKp = new QSpinBox(this);
    m_angleKp->setRange(-100000, 100000);
    grid->addWidget(m_angleKp, row, 1);
    grid->addWidget(new QLabel("/ Div:", this), row, 2);
    m_angleKpDiv = new QSpinBox(this);
    m_angleKpDiv->setRange(1, 100000);
    grid->addWidget(m_angleKpDiv, row, 3);
    row++;

    // Ki
    grid->addWidget(new QLabel("Ki:", this), row, 0);
    m_angleKi = new QSpinBox(this);
    m_angleKi->setRange(-100000, 100000);
    grid->addWidget(m_angleKi, row, 1);
    grid->addWidget(new QLabel("/ Div:", this), row, 2);
    m_angleKiDiv = new QSpinBox(this);
    m_angleKiDiv->setRange(1, 100000);
    grid->addWidget(m_angleKiDiv, row, 3);
    row++;

    // Kd
    grid->addWidget(new QLabel("Kd:", this), row, 0);
    m_angleKd = new QSpinBox(this);
    m_angleKd->setRange(-100000, 100000);
    grid->addWidget(m_angleKd, row, 1);
    grid->addWidget(new QLabel("/ Div:", this), row, 2);
    m_angleKdDiv = new QSpinBox(this);
    m_angleKdDiv->setRange(1, 100000);
    grid->addWidget(m_angleKdDiv, row, 3);
    row++;

    // Kv (Feed-forward)
    grid->addWidget(new QLabel("Kv:", this), row, 0);
    m_angleKv = new QSpinBox(this);
    m_angleKv->setRange(-100000, 100000);
    grid->addWidget(m_angleKv, row, 1);
    grid->addWidget(new QLabel("/ Div:", this), row, 2);
    m_angleKvDiv = new QSpinBox(this);
    m_angleKvDiv->setRange(1, 100000);
    grid->addWidget(m_angleKvDiv, row, 3);
    row++;

    // Output limits
    grid->addWidget(new QLabel("Output Max:", this), row, 0);
    m_angleOutMax = new QSpinBox(this);
    m_angleOutMax->setRange(-100000, 100000);
    grid->addWidget(m_angleOutMax, row, 1);
    grid->addWidget(new QLabel("Output Min:", this), row, 2);
    m_angleOutMin = new QSpinBox(this);
    m_angleOutMin->setRange(-100000, 100000);
    grid->addWidget(m_angleOutMin, row, 3);

    layout->addWidget(paramGroup);

    // Buttons
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch();

    QPushButton *refreshBtn = new QPushButton("Refresh", this);
    connect(refreshBtn, &QPushButton::clicked, this, &PIDTuningWidget::onRefreshAnglePID);
    buttonLayout->addWidget(refreshBtn);

    QPushButton *applyBtn = new QPushButton("Apply", this);
    applyBtn->setStyleSheet("background-color: #4CAF50; color: white;");
    connect(applyBtn, &QPushButton::clicked, this, &PIDTuningWidget::onApplyAnglePID);
    buttonLayout->addWidget(applyBtn);

    layout->addLayout(buttonLayout);
    layout->addStretch();

    return widget;
}

QWidget* PIDTuningWidget::createPressurePIDTab()
{
    QWidget *widget = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(widget);

    QGroupBox *paramGroup = new QGroupBox("Pressure Loop PID Parameters", this);
    QGridLayout *grid = new QGridLayout(paramGroup);

    int row = 0;

    // Kp
    grid->addWidget(new QLabel("Kp:", this), row, 0);
    m_pressureKp = new QSpinBox(this);
    m_pressureKp->setRange(-100000, 100000);
    grid->addWidget(m_pressureKp, row, 1);
    grid->addWidget(new QLabel("/ Div:", this), row, 2);
    m_pressureKpDiv = new QSpinBox(this);
    m_pressureKpDiv->setRange(1, 100000);
    grid->addWidget(m_pressureKpDiv, row, 3);
    row++;

    // Ki
    grid->addWidget(new QLabel("Ki:", this), row, 0);
    m_pressureKi = new QSpinBox(this);
    m_pressureKi->setRange(-100000, 100000);
    grid->addWidget(m_pressureKi, row, 1);
    grid->addWidget(new QLabel("/ Div:", this), row, 2);
    m_pressureKiDiv = new QSpinBox(this);
    m_pressureKiDiv->setRange(1, 100000);
    grid->addWidget(m_pressureKiDiv, row, 3);
    row++;

    // Kd
    grid->addWidget(new QLabel("Kd:", this), row, 0);
    m_pressureKd = new QSpinBox(this);
    m_pressureKd->setRange(-100000, 100000);
    grid->addWidget(m_pressureKd, row, 1);
    grid->addWidget(new QLabel("/ Div:", this), row, 2);
    m_pressureKdDiv = new QSpinBox(this);
    m_pressureKdDiv->setRange(1, 100000);
    grid->addWidget(m_pressureKdDiv, row, 3);
    row++;

    // Kv
    grid->addWidget(new QLabel("Kv:", this), row, 0);
    m_pressureKv = new QSpinBox(this);
    m_pressureKv->setRange(-100000, 100000);
    grid->addWidget(m_pressureKv, row, 1);
    grid->addWidget(new QLabel("/ Div:", this), row, 2);
    m_pressureKvDiv = new QSpinBox(this);
    m_pressureKvDiv->setRange(1, 100000);
    grid->addWidget(m_pressureKvDiv, row, 3);
    row++;

    // Output limits
    grid->addWidget(new QLabel("Output Max:", this), row, 0);
    m_pressureOutMax = new QSpinBox(this);
    m_pressureOutMax->setRange(-100000, 100000);
    grid->addWidget(m_pressureOutMax, row, 1);
    grid->addWidget(new QLabel("Output Min:", this), row, 2);
    m_pressureOutMin = new QSpinBox(this);
    m_pressureOutMin->setRange(-100000, 100000);
    grid->addWidget(m_pressureOutMin, row, 3);

    layout->addWidget(paramGroup);

    // Buttons
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch();

    QPushButton *refreshBtn = new QPushButton("Refresh", this);
    connect(refreshBtn, &QPushButton::clicked, this, &PIDTuningWidget::onRefreshPressurePID);
    buttonLayout->addWidget(refreshBtn);

    QPushButton *applyBtn = new QPushButton("Apply", this);
    applyBtn->setStyleSheet("background-color: #4CAF50; color: white;");
    connect(applyBtn, &QPushButton::clicked, this, &PIDTuningWidget::onApplyPressurePID);
    buttonLayout->addWidget(applyBtn);

    layout->addLayout(buttonLayout);
    layout->addStretch();

    return widget;
}

QWidget* PIDTuningWidget::createCalibrationTab()
{
    QWidget *widget = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(widget);

    // Angle calibration
    QGroupBox *angleGroup = new QGroupBox("Angle Sensor Calibration", this);
    QGridLayout *angleGrid = new QGridLayout(angleGroup);

    angleGrid->addWidget(new QLabel("Min ADC:", this), 0, 0);
    m_angleMinAdc = new QSpinBox(this);
    m_angleMinAdc->setRange(0, 65535);
    angleGrid->addWidget(m_angleMinAdc, 0, 1);

    angleGrid->addWidget(new QLabel("Mid ADC:", this), 0, 2);
    m_angleMidAdc = new QSpinBox(this);
    m_angleMidAdc->setRange(0, 65535);
    angleGrid->addWidget(m_angleMidAdc, 0, 3);

    angleGrid->addWidget(new QLabel("Max ADC:", this), 1, 0);
    m_angleMaxAdc = new QSpinBox(this);
    m_angleMaxAdc->setRange(0, 65535);
    angleGrid->addWidget(m_angleMaxAdc, 1, 1);

    angleGrid->addWidget(new QLabel("Range:", this), 1, 2);
    m_angleRange = new QSpinBox(this);
    m_angleRange->setRange(0, 65535);
    angleGrid->addWidget(m_angleRange, 1, 3);

    layout->addWidget(angleGroup);

    // Pressure calibration
    QGroupBox *pressureGroup = new QGroupBox("Pressure Sensor Calibration", this);
    QGridLayout *pressureGrid = new QGridLayout(pressureGroup);

    pressureGrid->addWidget(new QLabel("Min ADC:", this), 0, 0);
    m_pressureMinAdc = new QSpinBox(this);
    m_pressureMinAdc->setRange(0, 65535);
    pressureGrid->addWidget(m_pressureMinAdc, 0, 1);

    pressureGrid->addWidget(new QLabel("Mid ADC:", this), 0, 2);
    m_pressureMidAdc = new QSpinBox(this);
    m_pressureMidAdc->setRange(0, 65535);
    pressureGrid->addWidget(m_pressureMidAdc, 0, 3);

    pressureGrid->addWidget(new QLabel("Max ADC:", this), 1, 0);
    m_pressureMaxAdc = new QSpinBox(this);
    m_pressureMaxAdc->setRange(0, 65535);
    pressureGrid->addWidget(m_pressureMaxAdc, 1, 1);

    pressureGrid->addWidget(new QLabel("Max Bar:", this), 1, 2);
    m_pressureMaxBar = new QSpinBox(this);
    m_pressureMaxBar->setRange(0, 1000);
    pressureGrid->addWidget(m_pressureMaxBar, 1, 3);

    layout->addWidget(pressureGroup);

    // Buttons
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch();

    QPushButton *refreshBtn = new QPushButton("Refresh", this);
    connect(refreshBtn, &QPushButton::clicked, this, &PIDTuningWidget::onRefreshCalibration);
    buttonLayout->addWidget(refreshBtn);

    QPushButton *applyBtn = new QPushButton("Apply", this);
    applyBtn->setStyleSheet("background-color: #4CAF50; color: white;");
    connect(applyBtn, &QPushButton::clicked, this, &PIDTuningWidget::onApplyCalibration);
    buttonLayout->addWidget(applyBtn);

    layout->addLayout(buttonLayout);
    layout->addStretch();

    return widget;
}

QWidget* PIDTuningWidget::createFiltersTab()
{
    QWidget *widget = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(widget);

    QGroupBox *filterGroup = new QGroupBox("Low-Pass Filter Time Constants", this);
    QFormLayout *form = new QFormLayout(filterGroup);

    m_angleRefFilter = new QDoubleSpinBox(this);
    m_angleRefFilter->setRange(0.0001, 1.0);
    m_angleRefFilter->setDecimals(4);
    m_angleRefFilter->setSingleStep(0.001);
    form->addRow("Angle Reference Filter:", m_angleRefFilter);

    m_angleFdbFilter = new QDoubleSpinBox(this);
    m_angleFdbFilter->setRange(0.0001, 1.0);
    m_angleFdbFilter->setDecimals(4);
    m_angleFdbFilter->setSingleStep(0.001);
    form->addRow("Angle Feedback Filter:", m_angleFdbFilter);

    m_pressureFdbFilter = new QDoubleSpinBox(this);
    m_pressureFdbFilter->setRange(0.0001, 1.0);
    m_pressureFdbFilter->setDecimals(4);
    m_pressureFdbFilter->setSingleStep(0.001);
    form->addRow("Pressure Feedback Filter:", m_pressureFdbFilter);

    m_currentFdbFilter = new QDoubleSpinBox(this);
    m_currentFdbFilter->setRange(0.0001, 1.0);
    m_currentFdbFilter->setDecimals(4);
    m_currentFdbFilter->setSingleStep(0.001);
    form->addRow("Current Feedback Filter:", m_currentFdbFilter);

    layout->addWidget(filterGroup);

    // Buttons
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch();

    QPushButton *refreshBtn = new QPushButton("Refresh", this);
    connect(refreshBtn, &QPushButton::clicked, this, &PIDTuningWidget::onRefreshFilters);
    buttonLayout->addWidget(refreshBtn);

    QPushButton *applyBtn = new QPushButton("Apply", this);
    applyBtn->setStyleSheet("background-color: #4CAF50; color: white;");
    connect(applyBtn, &QPushButton::clicked, this, &PIDTuningWidget::onApplyFilters);
    buttonLayout->addWidget(applyBtn);

    layout->addLayout(buttonLayout);
    layout->addStretch();

    return widget;
}

// Update methods
void PIDTuningWidget::updateAnglePID(const PIDParameters &params)
{
    m_angleKp->setValue(params.kp);
    m_angleKpDiv->setValue(params.kp_div);
    m_angleKi->setValue(params.ki);
    m_angleKiDiv->setValue(params.ki_div);
    m_angleKd->setValue(params.kd);
    m_angleKdDiv->setValue(params.kd_div);
    m_angleKv->setValue(params.kv);
    m_angleKvDiv->setValue(params.kv_div);
    m_angleOutMax->setValue(params.output_max);
    m_angleOutMin->setValue(params.output_min);
}

void PIDTuningWidget::updatePressurePID(const PIDParameters &params)
{
    m_pressureKp->setValue(params.kp);
    m_pressureKpDiv->setValue(params.kp_div);
    m_pressureKi->setValue(params.ki);
    m_pressureKiDiv->setValue(params.ki_div);
    m_pressureKd->setValue(params.kd);
    m_pressureKdDiv->setValue(params.kd_div);
    m_pressureKv->setValue(params.kv);
    m_pressureKvDiv->setValue(params.kv_div);
    m_pressureOutMax->setValue(params.output_max);
    m_pressureOutMin->setValue(params.output_min);
}

void PIDTuningWidget::updateCalibration(const CalibrationParameters &params)
{
    m_angleMinAdc->setValue(params.angle_min_adc);
    m_angleMidAdc->setValue(params.angle_mid_adc);
    m_angleMaxAdc->setValue(params.angle_max_adc);
    m_angleRange->setValue(params.angle_range);
    m_pressureMinAdc->setValue(params.pressure_min_adc);
    m_pressureMidAdc->setValue(params.pressure_mid_adc);
    m_pressureMaxAdc->setValue(params.pressure_max_adc);
    m_pressureMaxBar->setValue(params.pressure_max_bar);
}

void PIDTuningWidget::updateFilterParams(const FilterParameters &params)
{
    m_angleRefFilter->setValue(params.angle_ref_filter);
    m_angleFdbFilter->setValue(params.angle_fdb_filter);
    m_pressureFdbFilter->setValue(params.pressure_fdb_filter);
    m_currentFdbFilter->setValue(params.current_fdb_filter);
}

// Slot implementations
void PIDTuningWidget::onRefreshAnglePID()
{
    if (m_serial && m_serial->isConnected()) {
        m_serial->requestAnglePID();
    }
}

void PIDTuningWidget::onApplyAnglePID()
{
    if (!m_serial || !m_serial->isConnected()) return;

    PIDParameters params;
    params.kp = m_angleKp->value();
    params.kp_div = m_angleKpDiv->value();
    params.ki = m_angleKi->value();
    params.ki_div = m_angleKiDiv->value();
    params.kd = m_angleKd->value();
    params.kd_div = m_angleKdDiv->value();
    params.kv = m_angleKv->value();
    params.kv_div = m_angleKvDiv->value();
    params.output_max = m_angleOutMax->value();
    params.output_min = m_angleOutMin->value();

    m_serial->setAnglePID(params);
}

void PIDTuningWidget::onRefreshPressurePID()
{
    if (m_serial && m_serial->isConnected()) {
        m_serial->requestPressurePID();
    }
}

void PIDTuningWidget::onApplyPressurePID()
{
    if (!m_serial || !m_serial->isConnected()) return;

    PIDParameters params;
    params.kp = m_pressureKp->value();
    params.kp_div = m_pressureKpDiv->value();
    params.ki = m_pressureKi->value();
    params.ki_div = m_pressureKiDiv->value();
    params.kd = m_pressureKd->value();
    params.kd_div = m_pressureKdDiv->value();
    params.kv = m_pressureKv->value();
    params.kv_div = m_pressureKvDiv->value();
    params.output_max = m_pressureOutMax->value();
    params.output_min = m_pressureOutMin->value();

    m_serial->setPressurePID(params);
}

void PIDTuningWidget::onRefreshCalibration()
{
    if (m_serial && m_serial->isConnected()) {
        m_serial->requestCalibration();
    }
}

void PIDTuningWidget::onApplyCalibration()
{
    if (!m_serial || !m_serial->isConnected()) return;

    CalibrationParameters params;
    params.angle_min_adc = m_angleMinAdc->value();
    params.angle_mid_adc = m_angleMidAdc->value();
    params.angle_max_adc = m_angleMaxAdc->value();
    params.angle_range = m_angleRange->value();
    params.pressure_min_adc = m_pressureMinAdc->value();
    params.pressure_mid_adc = m_pressureMidAdc->value();
    params.pressure_max_adc = m_pressureMaxAdc->value();
    params.pressure_max_bar = m_pressureMaxBar->value();

    m_serial->setCalibration(params);
}

void PIDTuningWidget::onRefreshFilters()
{
    if (m_serial && m_serial->isConnected()) {
        m_serial->requestFilterParams();
    }
}

void PIDTuningWidget::onApplyFilters()
{
    if (!m_serial || !m_serial->isConnected()) return;

    FilterParameters params;
    params.angle_ref_filter = m_angleRefFilter->value();
    params.angle_fdb_filter = m_angleFdbFilter->value();
    params.pressure_fdb_filter = m_pressureFdbFilter->value();
    params.current_fdb_filter = m_currentFdbFilter->value();

    m_serial->setFilterParams(params);
}
