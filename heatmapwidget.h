#ifndef HEATMAPWIDGET_H
#define HEATMAPWIDGET_H

#include <QColor>
#include <QPixmap>
#include <QPoint>
#include <QPointF>
#include <QStringList>
#include <QVector>
#include <QWidget>
#include <QFont>
#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QPen>
#include <QtMath>

class HeatmapWidget : public QWidget
{
    Q_OBJECT

public:
    explicit HeatmapWidget(QWidget *parent = nullptr);
    enum class CellType { NoData, Exact, InterpolatedEsp, InterpolatedAp };
    void setWifiAnchor(const QPointF &cell);
    void setHeatSamples(const QVector<QPointF> &positions, const QVector<double> &values);
    void setEstimatedNodes(const QVector<QPointF> &cells, const QStringList &labels);
    void setBuildingPlan(const QPixmap &pixmap);
    void setBuildingPlanVisible(bool visible);
    void setBuildingPlanScale(double scale);
    bool hasWifiAnchor() const;
    void setApCell(const QPointF &cell, double metersPerCell);
    QPixmap buildingPlan() const { return m_buildingPlan; }
    QPointF wifiAnchorCell() const;

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    QColor colorForValue(double value) const;
    QRectF gridRect() const;
    QPoint cellAtPosition(const QPoint &pos) const;
    double interpolatedValueAtCell(int row, int col, CellType *type) const;
    QPointF m_apCell;
    bool    m_hasAp = false;
    double  m_metersPerCell = 0.5;
    QVector<double> m_sampleValues;
    QVector<QPointF> m_samplePositions;
    QVector<QPointF> m_estimatedNodes;
    QStringList m_estimatedLabels;
    QPixmap m_buildingPlan;
    bool m_showBuildingPlan;
    double m_buildingPlanScale;
    QPoint m_wifiAnchor;
    bool m_hasWifiAnchor;
    bool m_draggingAnchor;

signals:
    void wifiAnchorChanged(const QPointF &cell);
};



#endif // HEATMAPWIDGET_H
