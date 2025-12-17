/**
 * @file gaugewidget.h
 * @brief Custom Gauge Widget for Visual Display
 */

#ifndef GAUGEWIDGET_H
#define GAUGEWIDGET_H

#include <QWidget>
#include <QPainter>
#include <QPropertyAnimation>

class GaugeWidget : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(double value READ value WRITE setValue)
    Q_PROPERTY(double minValue READ minValue WRITE setMinValue)
    Q_PROPERTY(double maxValue READ maxValue WRITE setMaxValue)

public:
    explicit GaugeWidget(QWidget *parent = nullptr);

    double value() const { return m_value; }
    double minValue() const { return m_minValue; }
    double maxValue() const { return m_maxValue; }

    void setValue(double value);
    void setMinValue(double value);
    void setMaxValue(double value);
    void setRange(double min, double max);
    void setTitle(const QString &title);
    void setUnit(const QString &unit);
    void setWarningThreshold(double value);
    void setCriticalThreshold(double value);
    void setAnimated(bool animated);

    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    void drawBackground(QPainter &painter, const QRectF &rect);
    void drawScale(QPainter &painter, const QRectF &rect);
    void drawNeedle(QPainter &painter, const QRectF &rect);
    void drawValue(QPainter &painter, const QRectF &rect);
    double valueToAngle(double value) const;

    double m_value;
    double m_displayValue;  // For animation
    double m_minValue;
    double m_maxValue;
    double m_warningThreshold;
    double m_criticalThreshold;
    QString m_title;
    QString m_unit;
    bool m_animated;
    QPropertyAnimation *m_animation;
};

#endif // GAUGEWIDGET_H
