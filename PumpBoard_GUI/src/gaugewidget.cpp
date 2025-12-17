/**
 * @file gaugewidget.cpp
 * @brief Custom Gauge Widget Implementation
 */

#include "gaugewidget.h"
#include <QPainterPath>
#include <QtMath>

GaugeWidget::GaugeWidget(QWidget *parent)
    : QWidget(parent)
    , m_value(0)
    , m_displayValue(0)
    , m_minValue(0)
    , m_maxValue(100)
    , m_warningThreshold(80)
    , m_criticalThreshold(95)
    , m_animated(true)
    , m_animation(nullptr)
{
    setMinimumSize(150, 150);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

QSize GaugeWidget::sizeHint() const
{
    return QSize(200, 200);
}

QSize GaugeWidget::minimumSizeHint() const
{
    return QSize(150, 150);
}

void GaugeWidget::setValue(double value)
{
    value = qBound(m_minValue, value, m_maxValue);

    if (qFuzzyCompare(m_value, value)) return;

    m_value = value;
    m_displayValue = value;
    update();
}

void GaugeWidget::setMinValue(double value)
{
    m_minValue = value;
    update();
}

void GaugeWidget::setMaxValue(double value)
{
    m_maxValue = value;
    update();
}

void GaugeWidget::setRange(double min, double max)
{
    m_minValue = min;
    m_maxValue = max;
    update();
}

void GaugeWidget::setTitle(const QString &title)
{
    m_title = title;
    update();
}

void GaugeWidget::setUnit(const QString &unit)
{
    m_unit = unit;
    update();
}

void GaugeWidget::setWarningThreshold(double value)
{
    m_warningThreshold = value;
    update();
}

void GaugeWidget::setCriticalThreshold(double value)
{
    m_criticalThreshold = value;
    update();
}

void GaugeWidget::setAnimated(bool animated)
{
    m_animated = animated;
}

double GaugeWidget::valueToAngle(double value) const
{
    // Map value to angle (225 to -45 degrees, sweep of 270 degrees)
    double ratio = (value - m_minValue) / (m_maxValue - m_minValue);
    return 225.0 - ratio * 270.0;
}

void GaugeWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    int side = qMin(width(), height());
    QRectF rect(0, 0, side, side);
    rect.moveCenter(QPointF(width() / 2.0, height() / 2.0));

    drawBackground(painter, rect);
    drawScale(painter, rect);
    drawNeedle(painter, rect);
    drawValue(painter, rect);
}

void GaugeWidget::drawBackground(QPainter &painter, const QRectF &rect)
{
    painter.save();

    // Outer circle
    QLinearGradient gradient(rect.topLeft(), rect.bottomRight());
    gradient.setColorAt(0, QColor(60, 60, 60));
    gradient.setColorAt(1, QColor(30, 30, 30));

    painter.setBrush(gradient);
    painter.setPen(QPen(QColor(80, 80, 80), 2));
    painter.drawEllipse(rect.adjusted(5, 5, -5, -5));

    // Inner circle
    QRectF innerRect = rect.adjusted(15, 15, -15, -15);
    QRadialGradient innerGradient(innerRect.center(), innerRect.width() / 2);
    innerGradient.setColorAt(0, QColor(50, 50, 50));
    innerGradient.setColorAt(1, QColor(20, 20, 20));

    painter.setBrush(innerGradient);
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(innerRect);

    painter.restore();
}

void GaugeWidget::drawScale(QPainter &painter, const QRectF &rect)
{
    painter.save();

    QPointF center = rect.center();
    double radius = rect.width() / 2 - 25;

    // Draw colored arc segments
    double arcRadius = radius - 10;
    QRectF arcRect(center.x() - arcRadius, center.y() - arcRadius,
                   arcRadius * 2, arcRadius * 2);

    painter.setPen(QPen(Qt::green, 8, Qt::SolidLine, Qt::RoundCap));
    double normalEnd = valueToAngle(m_warningThreshold);
    painter.drawArc(arcRect, (int)(225 * 16), (int)((225 - normalEnd) * 16));

    painter.setPen(QPen(Qt::yellow, 8, Qt::SolidLine, Qt::RoundCap));
    double warningEnd = valueToAngle(m_criticalThreshold);
    painter.drawArc(arcRect, (int)(normalEnd * 16), (int)((normalEnd - warningEnd) * 16));

    painter.setPen(QPen(Qt::red, 8, Qt::SolidLine, Qt::RoundCap));
    painter.drawArc(arcRect, (int)(warningEnd * 16), (int)((warningEnd + 45) * 16));

    // Draw tick marks
    painter.setPen(QPen(Qt::white, 2));
    QFont font = painter.font();
    font.setPointSize(8);
    painter.setFont(font);

    int numTicks = 10;
    for (int i = 0; i <= numTicks; ++i) {
        double value = m_minValue + (m_maxValue - m_minValue) * i / numTicks;
        double angle = valueToAngle(value);
        double rad = qDegreesToRadians(angle);

        double innerRadius = radius - 20;
        double outerRadius = radius - 5;

        QPointF inner(center.x() + innerRadius * qCos(rad),
                      center.y() - innerRadius * qSin(rad));
        QPointF outer(center.x() + outerRadius * qCos(rad),
                      center.y() - outerRadius * qSin(rad));

        painter.drawLine(inner, outer);

        // Draw labels for major ticks
        if (i % 2 == 0) {
            double labelRadius = radius - 35;
            QPointF labelPos(center.x() + labelRadius * qCos(rad),
                            center.y() - labelRadius * qSin(rad));
            QString label = QString::number((int)value);
            QRectF labelRect(labelPos.x() - 15, labelPos.y() - 8, 30, 16);
            painter.drawText(labelRect, Qt::AlignCenter, label);
        }
    }

    painter.restore();
}

void GaugeWidget::drawNeedle(QPainter &painter, const QRectF &rect)
{
    painter.save();

    QPointF center = rect.center();
    double radius = rect.width() / 2 - 40;
    double angle = valueToAngle(m_displayValue);
    double rad = qDegreesToRadians(angle);

    // Draw needle shadow
    painter.translate(2, 2);
    painter.setBrush(QColor(0, 0, 0, 100));
    painter.setPen(Qt::NoPen);

    QPainterPath shadowPath;
    shadowPath.moveTo(center.x() + radius * qCos(rad),
                      center.y() - radius * qSin(rad));
    shadowPath.lineTo(center.x() + 8 * qCos(rad + M_PI/2),
                      center.y() - 8 * qSin(rad + M_PI/2));
    shadowPath.lineTo(center.x() + 8 * qCos(rad - M_PI/2),
                      center.y() - 8 * qSin(rad - M_PI/2));
    shadowPath.closeSubpath();
    painter.drawPath(shadowPath);

    painter.translate(-2, -2);

    // Draw needle
    QColor needleColor = Qt::white;
    if (m_value >= m_criticalThreshold) {
        needleColor = Qt::red;
    } else if (m_value >= m_warningThreshold) {
        needleColor = Qt::yellow;
    }

    painter.setBrush(needleColor);
    painter.setPen(QPen(needleColor.darker(), 1));

    QPainterPath needlePath;
    needlePath.moveTo(center.x() + radius * qCos(rad),
                      center.y() - radius * qSin(rad));
    needlePath.lineTo(center.x() + 6 * qCos(rad + M_PI/2),
                      center.y() - 6 * qSin(rad + M_PI/2));
    needlePath.lineTo(center.x() - 15 * qCos(rad),
                      center.y() + 15 * qSin(rad));
    needlePath.lineTo(center.x() + 6 * qCos(rad - M_PI/2),
                      center.y() - 6 * qSin(rad - M_PI/2));
    needlePath.closeSubpath();
    painter.drawPath(needlePath);

    // Draw center cap
    QRadialGradient capGradient(center, 12);
    capGradient.setColorAt(0, QColor(200, 200, 200));
    capGradient.setColorAt(1, QColor(100, 100, 100));
    painter.setBrush(capGradient);
    painter.setPen(QPen(QColor(60, 60, 60), 1));
    painter.drawEllipse(center, 10, 10);

    painter.restore();
}

void GaugeWidget::drawValue(QPainter &painter, const QRectF &rect)
{
    painter.save();

    QPointF center = rect.center();

    // Draw title
    if (!m_title.isEmpty()) {
        QFont titleFont = painter.font();
        titleFont.setPointSize(10);
        titleFont.setBold(true);
        painter.setFont(titleFont);
        painter.setPen(Qt::white);

        QRectF titleRect(center.x() - 50, center.y() - rect.height()/4, 100, 20);
        painter.drawText(titleRect, Qt::AlignCenter, m_title);
    }

    // Draw value
    QFont valueFont = painter.font();
    valueFont.setPointSize(14);
    valueFont.setBold(true);
    painter.setFont(valueFont);

    QColor valueColor = Qt::white;
    if (m_value >= m_criticalThreshold) {
        valueColor = Qt::red;
    } else if (m_value >= m_warningThreshold) {
        valueColor = Qt::yellow;
    }
    painter.setPen(valueColor);

    QString valueText = QString::number(m_value, 'f', 1);
    if (!m_unit.isEmpty()) {
        valueText += " " + m_unit;
    }

    QRectF valueRect(center.x() - 50, center.y() + rect.height()/6, 100, 25);
    painter.drawText(valueRect, Qt::AlignCenter, valueText);

    painter.restore();
}
