/**
 * @file controlpanelwidget.cpp
 * @brief Control Panel Widget Implementation
 */

#include "controlpanelwidget.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFormLayout>

ControlPanelWidget::ControlPanelWidget(SerialCommunication *serial, QWidget *parent)
    : QWidget(parent)
    , m_serial(serial)
    , m_ignoreSignals(false)
{
    setupUI();
    setEnabled(false);  // Disabled until connected
}

void ControlPanelWidget::setupUI()
{
    QVBoxLayout *mainLayout = new QVBoxLayout(this);

    QHBoxLayout *topLayout = new QHBoxLayout();
    topLayout->addWidget(createReferenceGroup());
    topLayout->addWidget(createCurrentGroup());
    mainLayout->addLayout(topLayout);

    mainLayout->addWidget(createEnableFlagsGroup());

    // Action buttons
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch();

    m_resetButton = new QPushButton("Reset to Current", this);
    connect(m_resetButton, &QPushButton::clicked, this, &ControlPanelWidget::onResetClicked);
    buttonLayout->addWidget(m_resetButton);

    m_applyButton = new QPushButton("Apply All", this);
    m_applyButton->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;");
    connect(m_applyButton, &QPushButton::clicked, this, &ControlPanelWidget::onApplyClicked);
    buttonLayout->addWidget(m_applyButton);

    mainLayout->addLayout(buttonLayout);
    mainLayout->addStretch();
}

QGroupBox* ControlPanelWidget::createReferenceGroup()
{
    QGroupBox *group = new QGroupBox("Reference Values", this);
    QGridLayout *layout = new QGridLayout(group);

    // Angle reference
    layout->addWidget(new QLabel("Angle Reference:", this), 0, 0);

    m_angleSlider = new QSlider(Qt::Horizontal, this);
    m_angleSlider->setRange(0, 10000);
    m_angleSlider->setTickInterval(1000);
    m_angleSlider->setTickPosition(QSlider::TicksBelow);
    layout->addWidget(m_angleSlider, 0, 1);

    m_angleSpinBox = new QSpinBox(this);
    m_angleSpinBox->setRange(0, 10000);
    m_angleSpinBox->setSingleStep(100);
    layout->addWidget(m_angleSpinBox, 0, 2);

    m_anglePercentLabel = new QLabel("0.0%", this);
    m_anglePercentLabel->setMinimumWidth(60);
    layout->addWidget(m_anglePercentLabel, 0, 3);

    connect(m_angleSlider, &QSlider::valueChanged, this, &ControlPanelWidget::onAngleSliderChanged);
    connect(m_angleSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &ControlPanelWidget::onAngleSpinBoxChanged);

    // Pressure reference
    layout->addWidget(new QLabel("Pressure Reference:", this), 1, 0);

    m_pressureSlider = new QSlider(Qt::Horizontal, this);
    m_pressureSlider->setRange(0, 10000);
    m_pressureSlider->setTickInterval(1000);
    m_pressureSlider->setTickPosition(QSlider::TicksBelow);
    layout->addWidget(m_pressureSlider, 1, 1);

    m_pressureSpinBox = new QSpinBox(this);
    m_pressureSpinBox->setRange(0, 10000);
    m_pressureSpinBox->setSingleStep(100);
    layout->addWidget(m_pressureSpinBox, 1, 2);

    QHBoxLayout *pressureLabelLayout = new QHBoxLayout();
    m_pressurePercentLabel = new QLabel("0.0%", this);
    m_pressurePercentLabel->setMinimumWidth(60);
    pressureLabelLayout->addWidget(m_pressurePercentLabel);

    m_pressureBarLabel = new QLabel("(0 bar)", this);
    pressureLabelLayout->addWidget(m_pressureBarLabel);

    layout->addLayout(pressureLabelLayout, 1, 3);

    connect(m_pressureSlider, &QSlider::valueChanged, this, &ControlPanelWidget::onPressureSliderChanged);
    connect(m_pressureSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &ControlPanelWidget::onPressureSpinBoxChanged);

    return group;
}

QGroupBox* ControlPanelWidget::createCurrentGroup()
{
    QGroupBox *group = new QGroupBox("Current References (mA)", this);
    QGridLayout *layout = new QGridLayout(group);

    // Current A
    layout->addWidget(new QLabel("Current A:", this), 0, 0);

    m_currentASlider = new QSlider(Qt::Horizontal, this);
    m_currentASlider->setRange(0, 3000);
    m_currentASlider->setTickInterval(500);
    m_currentASlider->setTickPosition(QSlider::TicksBelow);
    layout->addWidget(m_currentASlider, 0, 1);

    m_currentASpinBox = new QSpinBox(this);
    m_currentASpinBox->setRange(0, 3000);
    m_currentASpinBox->setSingleStep(50);
    m_currentASpinBox->setSuffix(" mA");
    layout->addWidget(m_currentASpinBox, 0, 2);

    connect(m_currentASlider, &QSlider::valueChanged, m_currentASpinBox, &QSpinBox::setValue);
    connect(m_currentASpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            m_currentASlider, &QSlider::setValue);
    connect(m_currentASpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &ControlPanelWidget::onCurrentAChanged);

    // Current B
    layout->addWidget(new QLabel("Current B:", this), 1, 0);

    m_currentBSlider = new QSlider(Qt::Horizontal, this);
    m_currentBSlider->setRange(0, 3000);
    m_currentBSlider->setTickInterval(500);
    m_currentBSlider->setTickPosition(QSlider::TicksBelow);
    layout->addWidget(m_currentBSlider, 1, 1);

    m_currentBSpinBox = new QSpinBox(this);
    m_currentBSpinBox->setRange(0, 3000);
    m_currentBSpinBox->setSingleStep(50);
    m_currentBSpinBox->setSuffix(" mA");
    layout->addWidget(m_currentBSpinBox, 1, 2);

    connect(m_currentBSlider, &QSlider::valueChanged, m_currentBSpinBox, &QSpinBox::setValue);
    connect(m_currentBSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            m_currentBSlider, &QSlider::setValue);
    connect(m_currentBSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &ControlPanelWidget::onCurrentBChanged);

    return group;
}

QGroupBox* ControlPanelWidget::createEnableFlagsGroup()
{
    QGroupBox *group = new QGroupBox("Control Enable Flags", this);
    QHBoxLayout *layout = new QHBoxLayout(group);

    m_pumpStartCheck = new QCheckBox("Pump Start", this);
    m_pumpStartCheck->setStyleSheet("QCheckBox { font-weight: bold; }");
    connect(m_pumpStartCheck, &QCheckBox::toggled, this, &ControlPanelWidget::onEnableFlagChanged);
    layout->addWidget(m_pumpStartCheck);

    m_angleLeakCheck = new QCheckBox("Angle Leak Compensation", this);
    connect(m_angleLeakCheck, &QCheckBox::toggled, this, &ControlPanelWidget::onEnableFlagChanged);
    layout->addWidget(m_angleLeakCheck);

    m_pressureLoopCheck = new QCheckBox("Pressure Loop", this);
    connect(m_pressureLoopCheck, &QCheckBox::toggled, this, &ControlPanelWidget::onEnableFlagChanged);
    layout->addWidget(m_pressureLoopCheck);

    m_powerLoopCheck = new QCheckBox("Power Loop", this);
    connect(m_powerLoopCheck, &QCheckBox::toggled, this, &ControlPanelWidget::onEnableFlagChanged);
    layout->addWidget(m_powerLoopCheck);

    layout->addStretch();

    return group;
}

void ControlPanelWidget::syncAngleControls(int value)
{
    if (m_ignoreSignals) return;
    m_ignoreSignals = true;

    m_angleSlider->setValue(value);
    m_angleSpinBox->setValue(value);
    m_anglePercentLabel->setText(QString("%1%").arg(value / 100.0, 0, 'f', 1));

    m_ignoreSignals = false;
}

void ControlPanelWidget::syncPressureControls(int value)
{
    if (m_ignoreSignals) return;
    m_ignoreSignals = true;

    m_pressureSlider->setValue(value);
    m_pressureSpinBox->setValue(value);
    m_pressurePercentLabel->setText(QString("%1%").arg(value / 100.0, 0, 'f', 1));
    m_pressureBarLabel->setText(QString("(%1 bar)").arg(value / 10000.0 * 600.0, 0, 'f', 0));

    m_ignoreSignals = false;
}

void ControlPanelWidget::onAngleSliderChanged(int value)
{
    syncAngleControls(value);
}

void ControlPanelWidget::onAngleSpinBoxChanged(int value)
{
    syncAngleControls(value);
}

void ControlPanelWidget::onPressureSliderChanged(int value)
{
    syncPressureControls(value);
}

void ControlPanelWidget::onPressureSpinBoxChanged(int value)
{
    syncPressureControls(value);
}

void ControlPanelWidget::onCurrentAChanged(int value)
{
    Q_UNUSED(value);
    // Optional: immediate update
}

void ControlPanelWidget::onCurrentBChanged(int value)
{
    Q_UNUSED(value);
    // Optional: immediate update
}

void ControlPanelWidget::onEnableFlagChanged()
{
    // Optional: immediate update
}

void ControlPanelWidget::onApplyClicked()
{
    if (!m_serial || !m_serial->isConnected()) return;

    // Apply angle reference
    m_serial->setAngleReference(m_angleSpinBox->value());

    // Apply pressure reference
    m_serial->setPressureReference(m_pressureSpinBox->value());

    // Apply current references
    m_serial->setCurrentReferences(m_currentASpinBox->value(), m_currentBSpinBox->value());

    // Apply enable flags
    uint8_t flags = 0;
    if (m_pumpStartCheck->isChecked()) flags |= ENABLE_PUMP_START;
    if (m_angleLeakCheck->isChecked()) flags |= ENABLE_ANGLE_LEAK;
    if (m_pressureLoopCheck->isChecked()) flags |= ENABLE_PRESSURE_LOOP;
    if (m_powerLoopCheck->isChecked()) flags |= ENABLE_POWER_LOOP;

    m_serial->setEnableFlags(flags);
}

void ControlPanelWidget::onResetClicked()
{
    if (!m_serial) return;

    const FeedbackData &data = m_serial->latestFeedback();
    updateFromFeedback(data);
}

void ControlPanelWidget::updateFromFeedback(const FeedbackData &data)
{
    m_ignoreSignals = true;

    syncAngleControls(data.angle_ref);
    syncPressureControls(data.pressure_ref);

    m_currentASpinBox->setValue(data.current_a_ref);
    m_currentASlider->setValue(data.current_a_ref);
    m_currentBSpinBox->setValue(data.current_b_ref);
    m_currentBSlider->setValue(data.current_b_ref);

    m_pumpStartCheck->setChecked(data.enable_flags & ENABLE_PUMP_START);
    m_angleLeakCheck->setChecked(data.enable_flags & ENABLE_ANGLE_LEAK);
    m_pressureLoopCheck->setChecked(data.enable_flags & ENABLE_PRESSURE_LOOP);
    m_powerLoopCheck->setChecked(data.enable_flags & ENABLE_POWER_LOOP);

    m_ignoreSignals = false;
}

void ControlPanelWidget::setEnabled(bool enabled)
{
    m_angleSlider->setEnabled(enabled);
    m_angleSpinBox->setEnabled(enabled);
    m_pressureSlider->setEnabled(enabled);
    m_pressureSpinBox->setEnabled(enabled);
    m_currentASlider->setEnabled(enabled);
    m_currentASpinBox->setEnabled(enabled);
    m_currentBSlider->setEnabled(enabled);
    m_currentBSpinBox->setEnabled(enabled);
    m_pumpStartCheck->setEnabled(enabled);
    m_angleLeakCheck->setEnabled(enabled);
    m_pressureLoopCheck->setEnabled(enabled);
    m_powerLoopCheck->setEnabled(enabled);
    m_applyButton->setEnabled(enabled);
    m_resetButton->setEnabled(enabled);
}
