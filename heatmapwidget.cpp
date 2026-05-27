#include "heatmapwidget.h"


const int kRows = 24;
const int kCols = 24;
const double kMinRssi = -100.0;
const double kMaxRssi = -30.0;
const double kEspInfluenceRadiusMeters = 1.0;
const double kApInfluenceRadiusMeters  = 2.0;
const double kEpsilon = 1e-9;
const qreal kMargin = 16.0;
const qreal kTopBand = 50.0;
const double kApReferenceRssi  = -41.8;
const double kPathLossExponent = 2.5;


HeatmapWidget::HeatmapWidget(QWidget *parent)
    : QWidget(parent)
    , m_showBuildingPlan(false)
    , m_buildingPlanScale(1.0)
    , m_hasWifiAnchor(false)
    , m_draggingAnchor(false)
{
    setMinimumSize(640, 640);
    setMouseTracking(true);
}

void HeatmapWidget::setHeatSamples(const QVector<QPointF> &positions, const QVector<double> &values)
{
    if (positions.size() != values.size()) {
        return;
    }

    m_samplePositions = positions;
    m_sampleValues = values;
    update();
}

void HeatmapWidget::setEstimatedNodes(const QVector<QPointF> &cells, const QStringList &labels)
{
    m_estimatedNodes = cells;
    m_estimatedLabels = labels;
    update();
}

void HeatmapWidget::setBuildingPlan(const QPixmap &pixmap)
{
    m_buildingPlan = pixmap;
    update();
}

void HeatmapWidget::setBuildingPlanVisible(bool visible)
{
    m_showBuildingPlan = visible;
    update();
}

void HeatmapWidget::setWifiAnchor(const QPointF &cell)
{
    m_wifiAnchor    = cell.toPoint();
    m_hasWifiAnchor = true;
    update();
}

void HeatmapWidget::setBuildingPlanScale(double scale)
{
    m_buildingPlanScale = qMax(0.1, scale);
    update();
}

bool HeatmapWidget::hasWifiAnchor() const
{
    return m_hasWifiAnchor;
}

QPointF HeatmapWidget::wifiAnchorCell() const
{
    return QPointF(m_wifiAnchor);
}

void HeatmapWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const QRectF area = gridRect();
    const qreal cellW = area.width() / kCols;
    const qreal cellH = area.height() / kRows;

    if (m_showBuildingPlan && !m_buildingPlan.isNull()) {
        const int targetW = qMax(1, qRound(area.width() * m_buildingPlanScale));
        const int targetH = qMax(1, qRound(area.height() * m_buildingPlanScale));
        const QPoint targetTopLeft(qRound(area.center().x() - targetW / 2.0),
                                   qRound(area.center().y() - targetH / 2.0));
        const QRect targetRect(targetTopLeft, QSize(targetW, targetH));
        const QPixmap scaledPlan = m_buildingPlan.scaled(targetRect.size(),
                                                         Qt::IgnoreAspectRatio,
                                                         Qt::SmoothTransformation);
        painter.save();
        painter.setOpacity(1.0);
        painter.drawPixmap(targetRect.topLeft(), scaledPlan);
        painter.restore();
    }

    for (int row = 0; row < kRows; ++row) {
        for (int col = 0; col < kCols; ++col) {
            const QRectF cell(area.left() + col * cellW,
                              area.top() + row * cellH,
                              cellW,
                              cellH);

            CellType type = CellType::NoData;
            const double rssi = interpolatedValueAtCell(row, col, &type);

            if (type != CellType::NoData) {
                QColor c = colorForValue(rssi);
                c.setAlpha(m_showBuildingPlan ? 95 : 255);
                painter.fillRect(cell, c);

                if (type == CellType::InterpolatedEsp) {
                    painter.save();
                    painter.setPen(QPen(QColor(10, 10, 10, 170), 1.0));
                    painter.setFont(QFont(painter.font().family(), 8, QFont::Bold));
                    painter.drawText(cell.adjusted(1, 1, -1, -1),
                                     Qt::AlignTop | Qt::AlignRight,
                                     QStringLiteral("E"));
                    painter.restore();
                } else if (type == CellType::InterpolatedAp) {
                    painter.save();
                    painter.setPen(QPen(QColor(10, 10, 10, 130), 1.0));
                    painter.setFont(QFont(painter.font().family(), 8));
                    painter.drawText(cell.adjusted(1, 1, -1, -1),
                                     Qt::AlignTop | Qt::AlignRight,
                                     QStringLiteral("~"));
                    painter.restore();
                }
            } else {
                if (!m_showBuildingPlan)
                    painter.fillRect(cell, QColor(240, 240, 240));
            }
        }
    }

    painter.setPen(QPen(QColor(20, 20, 20, 70), 1.0));
    for (int row = 0; row <= kRows; ++row) {
        const qreal y = area.top() + row * cellH;
        painter.drawLine(QPointF(area.left(), y), QPointF(area.right(), y));
    }
    for (int col = 0; col <= kCols; ++col) {
        const qreal x = area.left() + col * cellW;
        painter.drawLine(QPointF(x, area.top()), QPointF(x, area.bottom()));
    }

    painter.setPen(QPen(Qt::black, 2.0));
    painter.drawRect(area);

    if (m_hasWifiAnchor) {
        const QRectF marker(area.left() + m_wifiAnchor.x() * cellW,
                            area.top() + m_wifiAnchor.y() * cellH,
                            cellW,
                            cellH);
        const QRectF inset = marker.adjusted(2, 2, -2, -2);

        painter.setBrush(QColor(255, 255, 255, 220));
        painter.setPen(QPen(Qt::black, 2.0));
        painter.drawEllipse(inset);
        painter.setFont(QFont(painter.font().family(), 9, QFont::Bold));
        painter.drawText(inset, Qt::AlignCenter, QStringLiteral("AP"));
    }

    for (int i = 0; i < m_estimatedNodes.size(); ++i) {
        const QPointF cellPos = m_estimatedNodes.at(i);
        const QRectF marker(area.left() + cellPos.x() * cellW,
                            area.top() + cellPos.y() * cellH,
                            cellW,
                            cellH);
        const QRectF inset = marker.adjusted(2, 2, -2, -2);

        painter.setBrush(QColor(10, 10, 10, 220));
        painter.setPen(QPen(Qt::white, 2.0));
        painter.drawRect(inset);

        painter.setPen(Qt::white);
        const QString label = (i < m_estimatedLabels.size()) ? m_estimatedLabels.at(i) : QString::number(i + 1);
        painter.drawText(inset, Qt::AlignCenter, label);
    }

    const qreal cx = rect().center().x();
    const QPointF top(cx, 20.0);
    const QPointF bottom(cx, 42.0);
    painter.setPen(QPen(Qt::black, 2.0));
    painter.drawLine(bottom, top);
    painter.drawLine(top, QPointF(cx - 5.0, 28.0));
    painter.drawLine(top, QPointF(cx + 5.0, 28.0));
    painter.setFont(QFont(painter.font().family(), 10, QFont::Bold));
    painter.drawText(QRectF(cx - 40.0, 0.0, 80.0, 14.0), Qt::AlignCenter, QStringLiteral("ПІВНІЧ"));

    const qreal legendW = 180.0;
    const qreal legendH = 14.0;
    const QRectF legendRect(rect().right() - legendW - 14.0, 10.0, legendW, legendH);
    const QRectF legendBg = legendRect.adjusted(-8.0, -6.0, 8.0, 22.0);
    painter.fillRect(legendBg, QColor(255, 255, 255, 220));
    painter.setPen(QPen(QColor(30, 30, 30, 140), 1.0));
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(legendBg);

    QLinearGradient grad(legendRect.left(), 0.0, legendRect.right(), 0.0);
    grad.setColorAt(0.0, QColor(40, 90, 255));
    grad.setColorAt(0.5, QColor(55, 210, 120));
    grad.setColorAt(1.0, QColor(240, 70, 55));
    painter.fillRect(legendRect, grad);
    painter.setPen(QPen(Qt::black, 1.0));
    painter.drawRect(legendRect);
    painter.setFont(QFont(painter.font().family(), 8));
    painter.drawText(QRectF(legendRect.left(), legendRect.bottom() + 2.0, 80.0, 14.0),
                     Qt::AlignLeft | Qt::AlignVCenter,
                     QStringLiteral("Слабкий (%1)").arg(static_cast<int>(kMinRssi)));
    painter.drawText(QRectF(legendRect.right() - 80.0, legendRect.bottom() + 2.0, 80.0, 14.0),
                     Qt::AlignRight | Qt::AlignVCenter,
                     QStringLiteral("Сильний (%1)").arg(static_cast<int>(kMaxRssi)));
}

void HeatmapWidget::mousePressEvent(QMouseEvent *event)
{
    const QPoint cell = cellAtPosition(event->pos());
    if (cell.x() < 0 || cell.y() < 0) {
        return;
    }

    if (event->button() == Qt::LeftButton) {
        m_wifiAnchor = cell;
        m_hasWifiAnchor = true;
        m_draggingAnchor = true;
        emit wifiAnchorChanged(QPointF(m_wifiAnchor));
        update();
        return;
    }

    if (event->button() == Qt::RightButton && m_hasWifiAnchor && m_wifiAnchor == cell) {
        m_hasWifiAnchor = false;
        m_draggingAnchor = false;
        emit wifiAnchorChanged(QPointF(-1.0, -1.0));
        update();
    }
}

void HeatmapWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (!m_draggingAnchor || !(event->buttons() & Qt::LeftButton)) {
        return;
    }

    const QPoint cell = cellAtPosition(event->pos());
    if (cell.x() < 0 || cell.y() < 0 || !m_hasWifiAnchor) {
        return;
    }

    if (cell != m_wifiAnchor) {
        m_wifiAnchor = cell;
        emit wifiAnchorChanged(QPointF(m_wifiAnchor));
        update();
    }
}

void HeatmapWidget::mouseReleaseEvent(QMouseEvent *event)
{
    Q_UNUSED(event)
    m_draggingAnchor = false;
}

QColor HeatmapWidget::colorForValue(double value) const
{
    const double clamped = qBound(kMinRssi, value, kMaxRssi);
    const double normalizedValue = (clamped - kMinRssi) / (kMaxRssi - kMinRssi);

    const int red = qRound(255.0 * normalizedValue);
    const int green = qRound(255.0 * (1.0 - qAbs(normalizedValue - 0.5) * 2.0));
    const int blue = qRound(255.0 * (1.0 - normalizedValue));
    return QColor(red, green, blue);
}

QRectF HeatmapWidget::gridRect() const
{
    return rect().adjusted(kMargin, kTopBand, -kMargin, -kMargin);
}

QPoint HeatmapWidget::cellAtPosition(const QPoint &pos) const
{
    const QRectF area = gridRect();
    if (!area.contains(pos)) {
        return {-1, -1};
    }

    const qreal cellW = area.width() / kCols;
    const qreal cellH = area.height() / kRows;

    int col = static_cast<int>((pos.x() - area.left()) / cellW);
    int row = static_cast<int>((pos.y() - area.top()) / cellH);

    col = qBound(0, col, kCols - 1);
    row = qBound(0, row, kRows - 1);
    return {col, row};
}

double HeatmapWidget::interpolatedValueAtCell(int row, int col, CellType *type) const
{
    if (type) *type = CellType::NoData;

    for (int i = 0; i < m_samplePositions.size(); ++i) {
        const QPointF &p = m_samplePositions.at(i);
        if (qAbs(p.x() - col) < kEpsilon && qAbs(p.y() - row) < kEpsilon) {
            if (type) *type = CellType::Exact;
            return m_sampleValues.value(i, kMinRssi);
        }
    }

    const QPointF target(col, row);
    double weightedSum = 0.0;
    double totalWeight = 0.0;
    const double espRadiusCells = kEspInfluenceRadiusMeters / m_metersPerCell;
    const double espSigmaCells  = espRadiusCells / 2.5;

    for (int i = 0; i < m_samplePositions.size(); ++i) {
        const QPointF &p = m_samplePositions.at(i);
        const double dx = target.x() - p.x();
        const double dy = target.y() - p.y();
        const double dist2 = dx * dx + dy * dy;
        if (qSqrt(dist2) > espRadiusCells) continue;
        const double gaussianWeight = qExp(-dist2 / (2.0 * espSigmaCells * espSigmaCells));
        weightedSum += gaussianWeight * m_sampleValues.value(i, kMinRssi);
        totalWeight += gaussianWeight;
    }

    if (totalWeight > 0.0) {
        if (type) *type = CellType::InterpolatedEsp;
        return weightedSum / totalWeight;
    }

    if (m_hasAp) {
        const double dx = col - m_apCell.x();
        const double dy = row - m_apCell.y();
        const double distCells = qSqrt(dx * dx + dy * dy);
        const double apRadiusCells = kApInfluenceRadiusMeters / m_metersPerCell;

        if (distCells > apRadiusCells) {
            if (type) *type = CellType::NoData;
            return kMinRssi;
        }

        const double distMeters = distCells * m_metersPerCell;
        if (distMeters < 0.01) {
            if (type) *type = CellType::InterpolatedAp;
            return kApReferenceRssi;
        }
        const double rssi = kApReferenceRssi
                            - 10.0 * kPathLossExponent * log10(distMeters);
        if (type) *type = CellType::InterpolatedAp;
        return qBound(kMinRssi, rssi, kMaxRssi);
    }

    return kMinRssi;
}


void HeatmapWidget::setApCell(const QPointF &cell, double metersPerCell)
{
    m_apCell        = cell;
    m_hasAp         = cell.x() >= 0.0 && cell.y() >= 0.0;
    m_metersPerCell = metersPerCell;
    update();
}
