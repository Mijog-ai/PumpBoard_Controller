/**
 * @file dashboardwidget.h
 * @brief Dashboard Widget for Real-time Monitoring Display
 */

#ifndef DASHBOARDWIDGET_H
#define DASHBOARDWIDGET_H

#include <QWidget>
#include <QLabel>
#include <QProgressBar>
#include <QGroupBox>
#include <QGridLayout>
#include "serialcommunication.h"

class GaugeWidget;

class DashboardWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DashboardWidget(QWidget *parent = nullptr);

    void updateData(const FeedbackData &data);
    void updateSystemStatus(const SystemStatus &status);

private:
    void setupUI();
    QGroupBox* createAngleGroup();
    QGroupBox* createPressureGroup();
    QGroupBox* createCurrentGroup();
    QGroupBox* createPWMGroup();
    QGroupBox* createStatusGroup();

    // Angle display
    GaugeWidget *m_angleGauge;
    QLabel *m_angleValueLabel;
    QLabel *m_angleRefLabel;
    QLabel *m_angleDiffLabel;
    QProgressBar *m_angleBar;

    // Pressure display
    GaugeWidget *m_pressureGauge;
    QLabel *m_pressureValueLabel;
    QLabel *m_pressureRefLabel;
    QLabel *m_pressureDiffLabel;
    QLabel *m_pressureBarLabel;
    QProgressBar *m_pressureBar;

    // Current A display
    QLabel *m_currentAValueLabel;
    QLabel *m_currentARefLabel;
    QProgressBar *m_currentABar;

    // Current B display
    QLabel *m_currentBValueLabel;
    QLabel *m_currentBRefLabel;
    QProgressBar *m_currentBBar;

    // PWM display
    QLabel *m_pwmAValueLabel;
    QLabel *m_pwmBValueLabel;
    QProgressBar *m_pwmABar;
    QProgressBar *m_pwmBBar;

    // Status indicators
    QLabel *m_pumpStartStatus;
    QLabel *m_angleleakStatus;
    QLabel *m_pressureLoopStatus;
    QLabel *m_powerLoopStatus;
    QLabel *m_errorCodeLabel;
    QLabel *m_simulationModeLabel;
    QLabel *m_firmwareVersionLabel;
    QLabel *m_timestampLabel;
};

#endif // DASHBOARDWIDGET_H
