#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPointF>
#include <QString>
#include <QVector>

#include <QtMqtt/QMqttClient>
#include <QtMqtt/QMqttTopicName>

#include "chipdatamodel.h"
#include "heatmapwidget.h"
#include "mqttconfig.h"
#include "qjsonobject.h"
#include "ui_mainwindow.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QLabel>
#include <QHBoxLayout>
#include <QRandomGenerator>
#include <QSlider>
#include <QPushButton>
#include <QStandardPaths>
#include <QStringList>
#include <QTextEdit>
#include <QTextStream>
#include <QVBoxLayout>
#include <QtMath>
#include <QJsonArray>
#include <QJsonDocument>
#include <QBuffer>
#include <QButtonGroup>
#include <aboutwindow.h>
class HeatmapWidget;
class QLabel;
class QPushButton;
class QSlider;
class QTextEdit;
class QFile;

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onMqttStateChanged(QMqttClient::ClientState state);
    void onMqttErrorChanged(QMqttClient::ClientError error);
    void onMqttMessageReceived(const QByteArray &message, const QMqttTopicName &topic);
    void onWifiAnchorChanged(const QPointF &cell);
    void onBuildingPlanClicked();
    void onBuildingPlanToggled(bool checked);
    void onBuildingPlanScaleChanged(int value);
    double bleRssiToDistance(double rssi) const;
    void onMetersPerCellChanged(int value);
    void onMqttConnected();
    void onSaveHeatmap();
    void onLoadHeatmap();
    void onRefRssiChanged(int value);
    void onBleRefRssiChanged(int value);
    void onPathLossChanged(int value);
    void onEwBiasChanged(int id);
    void onNsBiasChanged(int id);
    void onAboutTriggered();

private:
    struct NodeEstimateParams {
        double dAp0, dAp1, dAp2;
        double targetD01, targetD02, targetD12;
        QPointF apCell;
    };
    double computePositionError(const QVector<QPointF> &p, const NodeEstimateParams &params) const;
    double sanitizeRssiSign(double rssi) const;
    static QPointF clampPoint(QPointF p);
    static double cellDist(const QPointF &a, const QPointF &b);
    QVector<QPointF> estimateNodePositions(const QVector<double> &values, const QPointF &apCell) const;
    double rssiToDistance(double rssi) const;
    double minSepPenalty(double dist, double minD) const;
    double ordErr(double expSmaller, double expLarger) const;
    double northViol(double masterY, double nodeY) const;
    double toSignedRssi(double v) const;
    QString formatCell(const QPointF &p) const;
    double rssiToDistanceCells(double rssi) const;
    double bleRssiToDistanceCells(double rssi) const;
    void updateVisualization();
    void updateDebugPanel(const QVector<double> &values, const QVector<QPointF> &estimatedNodes);
    void clearHeatHistory();
    void updateHeatHistory(const QVector<QPointF> &estimatedNodes, const QVector<double> &wifiSamples);
    void initLogging();
    void appendLog(const QVector<QPointF> &estimatedNodes,
                   const QVector<double> &values);
    void setupUiLayout();
    void setupMqtt();
    void updatePlanScaleLabel();
    bool saveHeatmapToJson(const QString &path) const;
    bool loadHeatmapFromJson(const QString &path);
    Ui::MainWindow *ui;
    QMqttClient *m_mqttClient;
    QString m_mqttTopic;
    ChipDataModel m_chipData;
    QPointF m_wifiAnchorCell;
    bool m_hasWifiAnchor;
    QVector<QPointF> m_lastEstimatedNodes;
    bool m_hasLastEstimate;
    double m_metersPerCell;
    double m_buildingPlanScale;
    double m_apWeightMaster;
    double m_apWeightNodes;
    double m_bleDirectedWeight;
    double m_masterStabilityWeight;
    double m_ewBias = 0.0;
    QVector<double> m_heatSums;
    QVector<int> m_heatCounts;
    QFile   *m_logFile;
    QString  m_logPath;
    double m_referenceRssi;
    double m_bleReferenceRssi;
    double m_pathLossExponent;
    double m_nsBias;
    void setupConnections();
};
#endif // MAINWINDOW_H
