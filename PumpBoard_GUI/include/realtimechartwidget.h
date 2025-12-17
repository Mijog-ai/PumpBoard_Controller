/**
 * @file realtimechartwidget.h
 * @brief Real-time Chart Widget for Data Visualization
 */

#ifndef REALTIMECHARTWIDGET_H
#define REALTIMECHARTWIDGET_H

#include <QWidget>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QSpinBox>
#include <QPushButton>
#include <QGroupBox>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QElapsedTimer>
#include <QVector>
#include "serialcommunication.h"

class RealTimeChartWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RealTimeChartWidget(QWidget *parent = nullptr);

    void addDataPoint(const FeedbackData &data);
    void clearData();

private slots:
    void onChannelToggled(bool checked);
    void onTimeWindowChanged(int seconds);
    void onClearClicked();
    void onPauseClicked();

private:
    void setupUI();
    void setupChart();
    void updateChart();

    // Chart components
    QChart *m_chart;
    QChartView *m_chartView;
    QValueAxis *m_timeAxis;
    QValueAxis *m_valueAxis;

    // Data series
    QLineSeries *m_angleFdbSeries;
    QLineSeries *m_angleRefSeries;
    QLineSeries *m_pressureFdbSeries;
    QLineSeries *m_pressureRefSeries;
    QLineSeries *m_currentASeries;
    QLineSeries *m_currentBSeries;

    // Channel visibility controls
    QCheckBox *m_showAngleFdb;
    QCheckBox *m_showAngleRef;
    QCheckBox *m_showPressureFdb;
    QCheckBox *m_showPressureRef;
    QCheckBox *m_showCurrentA;
    QCheckBox *m_showCurrentB;

    // Controls
    QSpinBox *m_timeWindowSpinBox;
    QPushButton *m_clearButton;
    QPushButton *m_pauseButton;

    // State
    bool m_isPaused;
    QElapsedTimer m_elapsedTimer;
    double m_timeWindow;  // seconds
    int m_maxPoints;

    // Data storage
    QVector<double> m_timeData;
    QVector<double> m_angleFdbData;
    QVector<double> m_angleRefData;
    QVector<double> m_pressureFdbData;
    QVector<double> m_pressureRefData;
    QVector<double> m_currentAData;
    QVector<double> m_currentBData;
};

#endif // REALTIMECHARTWIDGET_H
