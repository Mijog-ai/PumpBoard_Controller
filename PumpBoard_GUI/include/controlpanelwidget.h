/**
 * @file controlpanelwidget.h
 * @brief Control Panel Widget for User-adjustable Parameters
 */

#ifndef CONTROLPANELWIDGET_H
#define CONTROLPANELWIDGET_H

#include <QWidget>
#include <QSlider>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QPushButton>
#include <QGroupBox>
#include <QLabel>
#include "serialcommunication.h"

class ControlPanelWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ControlPanelWidget(SerialCommunication *serial, QWidget *parent = nullptr);

    void updateFromFeedback(const FeedbackData &data);
    void setEnabled(bool enabled);

signals:
    void angleReferenceChanged(int32_t value);
    void pressureReferenceChanged(int32_t value);
    void currentReferencesChanged(int32_t currentA, int32_t currentB);
    void enableFlagsChanged(uint8_t flags);

private slots:
    void onAngleSliderChanged(int value);
    void onAngleSpinBoxChanged(int value);
    void onPressureSliderChanged(int value);
    void onPressureSpinBoxChanged(int value);
    void onCurrentAChanged(int value);
    void onCurrentBChanged(int value);
    void onEnableFlagChanged();
    void onApplyClicked();
    void onResetClicked();

private:
    void setupUI();
    QGroupBox* createReferenceGroup();
    QGroupBox* createCurrentGroup();
    QGroupBox* createEnableFlagsGroup();
    void syncAngleControls(int value);
    void syncPressureControls(int value);

    SerialCommunication *m_serial;

    // Angle reference controls
    QSlider *m_angleSlider;
    QSpinBox *m_angleSpinBox;
    QLabel *m_anglePercentLabel;

    // Pressure reference controls
    QSlider *m_pressureSlider;
    QSpinBox *m_pressureSpinBox;
    QLabel *m_pressurePercentLabel;
    QLabel *m_pressureBarLabel;

    // Current reference controls
    QSpinBox *m_currentASpinBox;
    QSpinBox *m_currentBSpinBox;
    QSlider *m_currentASlider;
    QSlider *m_currentBSlider;

    // Enable flags
    QCheckBox *m_pumpStartCheck;
    QCheckBox *m_angleLeakCheck;
    QCheckBox *m_pressureLoopCheck;
    QCheckBox *m_powerLoopCheck;

    // Action buttons
    QPushButton *m_applyButton;
    QPushButton *m_resetButton;

    bool m_ignoreSignals;
};

#endif // CONTROLPANELWIDGET_H
