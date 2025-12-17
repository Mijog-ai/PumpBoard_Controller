/**
 * @file pidtuningwidget.h
 * @brief PID Tuning Widget for Controller Parameter Adjustment
 */

#ifndef PIDTUNINGWIDGET_H
#define PIDTUNINGWIDGET_H

#include <QWidget>
#include <QTabWidget>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QGroupBox>
#include <QLabel>
#include "serialcommunication.h"

class PIDTuningWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PIDTuningWidget(SerialCommunication *serial, QWidget *parent = nullptr);

    void updateAnglePID(const PIDParameters &params);
    void updatePressurePID(const PIDParameters &params);
    void updateCalibration(const CalibrationParameters &params);
    void updateFilterParams(const FilterParameters &params);

private slots:
    void onRefreshAnglePID();
    void onApplyAnglePID();
    void onRefreshPressurePID();
    void onApplyPressurePID();
    void onRefreshCalibration();
    void onApplyCalibration();
    void onRefreshFilters();
    void onApplyFilters();

private:
    void setupUI();
    QWidget* createAnglePIDTab();
    QWidget* createPressurePIDTab();
    QWidget* createCalibrationTab();
    QWidget* createFiltersTab();

    SerialCommunication *m_serial;
    QTabWidget *m_tabWidget;

    // Angle PID controls
    QSpinBox *m_angleKp;
    QSpinBox *m_angleKpDiv;
    QSpinBox *m_angleKi;
    QSpinBox *m_angleKiDiv;
    QSpinBox *m_angleKd;
    QSpinBox *m_angleKdDiv;
    QSpinBox *m_angleKv;
    QSpinBox *m_angleKvDiv;
    QSpinBox *m_angleOutMax;
    QSpinBox *m_angleOutMin;

    // Pressure PID controls
    QSpinBox *m_pressureKp;
    QSpinBox *m_pressureKpDiv;
    QSpinBox *m_pressureKi;
    QSpinBox *m_pressureKiDiv;
    QSpinBox *m_pressureKd;
    QSpinBox *m_pressureKdDiv;
    QSpinBox *m_pressureKv;
    QSpinBox *m_pressureKvDiv;
    QSpinBox *m_pressureOutMax;
    QSpinBox *m_pressureOutMin;

    // Calibration controls
    QSpinBox *m_angleMinAdc;
    QSpinBox *m_angleMidAdc;
    QSpinBox *m_angleMaxAdc;
    QSpinBox *m_angleRange;
    QSpinBox *m_pressureMinAdc;
    QSpinBox *m_pressureMidAdc;
    QSpinBox *m_pressureMaxAdc;
    QSpinBox *m_pressureMaxBar;

    // Filter controls
    QDoubleSpinBox *m_angleRefFilter;
    QDoubleSpinBox *m_angleFdbFilter;
    QDoubleSpinBox *m_pressureFdbFilter;
    QDoubleSpinBox *m_currentFdbFilter;
};

#endif // PIDTUNINGWIDGET_H
