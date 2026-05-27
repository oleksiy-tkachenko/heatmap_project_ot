#include "mainwindow.h"
const int kExpectedValues = 9;
const QString kBrokerHost = QStringLiteral("192.168.31.140");
const quint16 kBrokerPort = 1883;
const QString kTopic = QStringLiteral("esp32/rssi");
const int kGridMaxCell = 23;
const int kGridCols = 24;
const int kGridRows = 24;
const double kOrderWeight = 60.0;
const double kDefaultPlanScale = 1.0;
const double kDefaultRssiSmoothingAlpha = 0.6;
const double kDefaultMasterStabilityWeight = 0.15;
const double kDefaultApWeightMaster = 1.5;
const double kDefaultApWeightNodes  = 0.8;
const double kDefaultBleDirectedWeight = 6.0;
const double kPi = 3.14159265358979323846;
const double kOptimizerInitialStep   = 4.0;
const double kOptimizerMinStep       = 0.05;
const double kOptimizerStepDecay     = 0.9994;
const int    kOptimizerIterations    = 3000;
const int    kOptimizerRestarts      = 25;
const int    kOptimizerFreshRestarts = 12;
const double kMinNodeSepCells        = 1.0;
const double kMinNodeSepWeight       = 15.0;
const double kNorthPenaltyWeight = 20.0;
const double kSeedAngleStep1         = 2.1;
const double kSeedAngleStep2         = 4.2;
const double kNodeStabilityNudge     = 0.05;


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_mqttClient(new QMqttClient(this))
    , m_mqttTopic(kTopic)
    , m_chipData(kDefaultRssiSmoothingAlpha)
    , m_wifiAnchorCell(-1.0, -1.0)
    , m_hasWifiAnchor(false)
    , m_hasLastEstimate(false)
    , m_buildingPlanScale(kDefaultPlanScale)
    , m_apWeightMaster(kDefaultApWeightMaster)
    , m_apWeightNodes(kDefaultApWeightNodes)
    , m_bleDirectedWeight(kDefaultBleDirectedWeight)
    , m_masterStabilityWeight(kDefaultMasterStabilityWeight)
    , m_heatSums(kGridRows * kGridCols, 0.0)
    , m_heatCounts(kGridRows * kGridCols, 0)
    , m_referenceRssi(-41.8)
    , m_bleReferenceRssi(-68.7)
    , m_pathLossExponent(2.5)
    , m_nsBias(0.0)
{
    ui->setupUi(this);
    setupConnections();
    initLogging();
    setupMqtt();
}

MainWindow::~MainWindow()
{
    if (m_logFile) {
        m_logFile->flush();
        m_logFile->close();
        delete m_logFile;
        m_logFile = nullptr;
    }
    delete ui;
}

double MainWindow::sanitizeRssiSign(double rssi) const
{
    return rssi > 0.0 ? -rssi : rssi;
}

void MainWindow::onMqttStateChanged(QMqttClient::ClientState state)
{
    switch (state) {
    case QMqttClient::Disconnected:
        ui->statusLabel->setText(QStringLiteral("MQTT відключено"));
        break;
    case QMqttClient::Connecting:
        ui->statusLabel->setText(QStringLiteral("MQTT підключення до %1:%2...")
                                     .arg(m_mqttClient->hostname())
                                     .arg(m_mqttClient->port()));
        break;
    case QMqttClient::Connected:
        ui->statusLabel->setText(QStringLiteral("MQTT підключено. Підписка на %1").arg(m_mqttTopic));
        break;
    }
}

void MainWindow::onMqttErrorChanged(QMqttClient::ClientError error)
{
    if (error == QMqttClient::NoError) {
        return;
    }
    ui->statusLabel->setText(QStringLiteral("Код помилки MQTT: %1").arg(static_cast<int>(error)));
}

QPointF MainWindow::clampPoint(QPointF p)
{
    p.setX(qBound(0.0, p.x(), static_cast<double>(kGridMaxCell)));
    p.setY(qBound(0.0, p.y(), static_cast<double>(kGridMaxCell)));
    return p;
}

double MainWindow::cellDist(const QPointF &a, const QPointF &b)
{
    const double dx = a.x() - b.x();
    const double dy = a.y() - b.y();
    return qSqrt(dx * dx + dy * dy);
}

void MainWindow::onMqttMessageReceived(const QByteArray &message, const QMqttTopicName &topic)
{
    if (topic.name() != m_mqttTopic) {
        return;
    }

    ChipSampleForLog sample;
    if (!m_chipData.updateFromPayload(message, &sample)) {
        ui->statusLabel->setText(QStringLiteral("Невірний JSON пакет: %1")
                                     .arg(QString::fromUtf8(message).left(100)));
        return;
    }


    if (m_chipData.consumeUpdateAndCheckCycleReady()) {
        updateVisualization();
        appendLog(m_lastEstimatedNodes, m_chipData.composeHeatmapValues());
    }
}

void MainWindow::onWifiAnchorChanged(const QPointF &cell)
{
    const QPointF oldAnchor = m_wifiAnchorCell;
    const bool oldHasAnchor = m_hasWifiAnchor;

    if (cell.x() < 0.0 || cell.y() < 0.0) {
        m_hasWifiAnchor = false;
        m_wifiAnchorCell = QPointF(-1.0, -1.0);
        clearHeatHistory();
        m_lastEstimatedNodes.clear();
        m_hasLastEstimate = false;
        ui->heatmap->setApCell(QPointF(-1.0, -1.0), m_metersPerCell);
    } else {
        m_hasWifiAnchor = true;
        m_wifiAnchorCell = cell;

        if (!oldHasAnchor || oldAnchor != m_wifiAnchorCell) {
            clearHeatHistory();
            m_lastEstimatedNodes.clear();
            m_hasLastEstimate = false;
        }
        ui->heatmap->setApCell(m_wifiAnchorCell, m_metersPerCell);
    }

    updateVisualization();
}

double MainWindow::minSepPenalty(double dist, double minD) const
{
    const double violation = minD - dist;
    return violation > 0.0 ? violation * violation : 0.0;
}

double MainWindow::ordErr(double expSmaller, double expLarger) const
{
    const double violation = expSmaller - expLarger;
    return violation > 0.0 ? violation * violation : 0.0;
}

double MainWindow::northViol(double masterY, double nodeY) const
{
    const double violation = masterY - nodeY;
    return violation > 0.0 ? violation * violation : 0.0;
}

double MainWindow::computePositionError(const QVector<QPointF> &points,
                                    const NodeEstimateParams &params) const
{
    if (points.size() != 3) {
        return 1e18;
    }

    double error = 0.0;
    error += m_apWeightMaster * qPow(cellDist(points[0], params.apCell) - params.dAp0, 2.0);
    error += m_apWeightNodes  * qPow(cellDist(points[1], params.apCell) - params.dAp1, 2.0);
    error += m_apWeightNodes  * qPow(cellDist(points[2], params.apCell) - params.dAp2, 2.0);

    const double d01 = cellDist(points[0], points[1]);
    const double d02 = cellDist(points[0], points[2]);
    const double d12 = cellDist(points[1], points[2]);

    error += m_bleDirectedWeight * qPow(d01 - params.targetD01, 2.0);
    error += m_bleDirectedWeight * qPow(d02 - params.targetD02, 2.0);
    error += m_bleDirectedWeight * qPow(d12 - params.targetD12, 2.0);

    error += kMinNodeSepWeight * minSepPenalty(d01, kMinNodeSepCells);
    error += kMinNodeSepWeight * minSepPenalty(d02, kMinNodeSepCells);
    error += kMinNodeSepWeight * minSepPenalty(d12, kMinNodeSepCells);

    const double act0 = cellDist(points[0], params.apCell);
    const double act1 = cellDist(points[1], params.apCell);
    const double act2 = cellDist(points[2], params.apCell);

    error += kOrderWeight * (params.dAp0 < params.dAp1 ? ordErr(act0, act1) : ordErr(act1, act0));
    error += kOrderWeight * (params.dAp0 < params.dAp2 ? ordErr(act0, act2) : ordErr(act2, act0));
    error += kOrderWeight * (params.dAp1 < params.dAp2 ? ordErr(act1, act2) : ordErr(act2, act1));

    error += kNorthPenaltyWeight * northViol(points[0].y(), points[1].y());
    error += kNorthPenaltyWeight * northViol(points[0].y(), points[2].y());

    const double dApArr[3] = {params.dAp0, params.dAp1, params.dAp2};
    if (qAbs(m_ewBias) > 0.01) {
        for (int i = 0; i < 3; ++i) {
            const double preferredX = params.apCell.x() + m_ewBias * dApArr[i];
            error += 4.0 * qPow(points[i].x() - preferredX, 2.0);
        }
    } else {
        const double centroidX = (points[0].x() + points[1].x() + points[2].x()) / 3.0;
        error += 2.0 * qPow(centroidX - params.apCell.x(), 2.0);
    }

    if (m_hasLastEstimate && m_lastEstimatedNodes.size() == 3) {
        error += m_masterStabilityWeight * qPow(cellDist(points[0], m_lastEstimatedNodes[0]), 2.0);
        error += kNodeStabilityNudge     * qPow(cellDist(points[1], m_lastEstimatedNodes[1]), 2.0);
        error += kNodeStabilityNudge     * qPow(cellDist(points[2], m_lastEstimatedNodes[2]), 2.0);
    }

    if (qAbs(m_nsBias) > 0.01) {
        for (int i = 0; i < 3; ++i) {
            const double preferredY = params.apCell.y() + m_nsBias * dApArr[i];
            error += 4.0 * qPow(points[i].y() - preferredY, 2.0);
        }
    } else {
        const double centroidY = (points[0].y() + points[1].y() + points[2].y()) / 3.0;
        error += 1.0 * qPow(centroidY - params.apCell.y(), 2.0);
    }

    return error;
}

QVector<QPointF> MainWindow::estimateNodePositions(const QVector<double> &values, const QPointF &apCell) const
{
    if (values.size() != kExpectedValues) {
        return {};
    }

    const double wifi0 = sanitizeRssiSign(values.at(0));
    const double wifi1 = sanitizeRssiSign(values.at(3));
    const double wifi2 = sanitizeRssiSign(values.at(6));

    const double rssi01 = (sanitizeRssiSign(values.at(1)) + sanitizeRssiSign(values.at(4))) / 2.0;
    const double rssi02 = (sanitizeRssiSign(values.at(2)) + sanitizeRssiSign(values.at(7))) / 2.0;
    const double rssi12 = (sanitizeRssiSign(values.at(5)) + sanitizeRssiSign(values.at(8))) / 2.0;

    const double dAp0 = rssiToDistance(wifi0) / m_metersPerCell;
    const double dAp1 = rssiToDistance(wifi1) / m_metersPerCell;
    const double dAp2 = rssiToDistance(wifi2) / m_metersPerCell;

    const double targetD01 = bleRssiToDistance(rssi01) / m_metersPerCell;
    const double targetD02 = bleRssiToDistance(rssi02) / m_metersPerCell;
    const double targetD12 = bleRssiToDistance(rssi12) / m_metersPerCell;


    if (wifi0 <= -99.0 || wifi1 <= -99.0 || wifi2 <= -99.0)
        return {};

    if (rssi01 <= -99.0 || rssi02 <= -99.0 || rssi12 <= -99.0)
        return {};

    const NodeEstimateParams params{dAp0, dAp1, dAp2,
                                 targetD01, targetD02, targetD12,
                                 apCell};

    QVector<QPointF> best;
    double bestScore = 1e18;

    QRandomGenerator *rng = QRandomGenerator::global();
    for (int restart = 0; restart < kOptimizerRestarts; ++restart) {
        QVector<QPointF> candidates(3);
        const bool usePrev = m_hasLastEstimate
                             && m_lastEstimatedNodes.size() == 3
                             && restart < (kOptimizerRestarts - kOptimizerFreshRestarts);
        if (usePrev) {
            for (int i = 0; i < 3; ++i) {
                const double dx = (rng->generateDouble() * 2.0 - 1.0) * 3.0;
                const double dy = (rng->generateDouble() * 2.0 - 1.0) * 3.0;
                candidates[i] = clampPoint(QPointF(m_lastEstimatedNodes[i].x() + dx,
                                     m_lastEstimatedNodes[i].y() + dy));
            }
        } else {
            const double angle = (restart / 25.0) * 2.0 * kPi;
            candidates[0] = clampPoint(QPointF(apCell.x() + dAp0 * qCos(angle),
                                      apCell.y() + dAp0 * qSin(angle)));
            candidates[1] = clampPoint(QPointF(apCell.x() + dAp1 * qCos(angle + kSeedAngleStep1),
                                      apCell.y() + dAp1 * qSin(angle + kSeedAngleStep1)));
            candidates[2] = clampPoint(QPointF(apCell.x() + dAp2 * qCos(angle + kSeedAngleStep2),
                                      apCell.y() + dAp2 * qSin(angle + kSeedAngleStep2)));
        }

        double step = kOptimizerInitialStep;
        double currentScore = computePositionError(candidates, params);

        for (int iter = 0; iter < kOptimizerIterations; ++iter) {
            const int idx = rng->bounded(3);
            QVector<QPointF> trial = candidates;
            const double dx = (rng->generateDouble() * 2.0 - 1.0) * step;
            const double dy = (rng->generateDouble() * 2.0 - 1.0) * step;
            trial[idx] = clampPoint(QPointF(candidates[idx].x() + dx, candidates[idx].y() + dy));

            const double trialScore = computePositionError(trial, params);
            if (trialScore < currentScore) {
                candidates = trial;
                currentScore = trialScore;
            }
            step = qMax(kOptimizerMinStep, step * kOptimizerStepDecay);
        }

        if (currentScore < bestScore) {
            best = candidates;
            bestScore = currentScore;
        }
    }

    return best;
}

double MainWindow::rssiToDistance(double rssi) const
{
    return qPow(10.0, (m_referenceRssi - rssi) / (10.0 * m_pathLossExponent));
}

void MainWindow::updateHeatHistory(const QVector<QPointF> &estimatedNodes, const QVector<double> &wifiSamples)
{
    if (estimatedNodes.size() != wifiSamples.size()) {
        return;
    }

    for (int i = 0; i < estimatedNodes.size(); ++i) {
        int col = qRound(estimatedNodes.at(i).x());
        int row = qRound(estimatedNodes.at(i).y());
        col = qBound(0, col, kGridCols - 1);
        row = qBound(0, row, kGridRows - 1);
        const int idx = row * kGridCols + col;
        m_heatSums[idx] += wifiSamples.at(i);
        m_heatCounts[idx] += 1;
    }

    QVector<QPointF> historyPositions;
    QVector<double> historyValues;
    for (int row = 0; row < kGridRows; ++row) {
        for (int col = 0; col < kGridCols; ++col) {
            const int idx = row * kGridCols + col;
            if (m_heatCounts[idx] <= 0) {
                continue;
            }
            historyPositions.push_back(QPointF(col, row));
            historyValues.push_back(m_heatSums[idx] / static_cast<double>(m_heatCounts[idx]));
        }
    }

    ui->heatmap->setHeatSamples(historyPositions, historyValues);
}

void MainWindow::clearHeatHistory()
{
    m_heatSums.fill(0.0);
    m_heatCounts.fill(0);
    m_chipData.clearSamplingState();
    ui->heatmap->setHeatSamples({}, {});
}

void MainWindow::updatePlanScaleLabel()
{
    ui->planScaleLabel->setText(QStringLiteral("Масштаб плану: %1%")
                                    .arg(static_cast<int>(qRound(m_buildingPlanScale * 100.0))));
}

bool MainWindow::saveHeatmapToJson(const QString &path) const
{
    QJsonObject root;
    root[QStringLiteral("version")]      = 1;
    root[QStringLiteral("savedAt")]      = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
    root[QStringLiteral("metersPerCell")] = m_metersPerCell;
    root[QStringLiteral("gridCols")]     = kGridCols;
    root[QStringLiteral("gridRows")]     = kGridRows;

    if (m_hasWifiAnchor) {
        QJsonObject ap;
        ap[QStringLiteral("x")] = m_wifiAnchorCell.x();
        ap[QStringLiteral("y")] = m_wifiAnchorCell.y();
        root[QStringLiteral("apAnchor")] = ap;
    }
    if (!ui->heatmap->buildingPlan().isNull()) {
        QByteArray imgBytes;
        QBuffer buf(&imgBytes);
        buf.open(QIODevice::WriteOnly);
        ui->heatmap->buildingPlan().save(&buf, "PNG");
        root[QStringLiteral("buildingPlanBase64")] = QString::fromLatin1(imgBytes.toBase64());
        root[QStringLiteral("buildingPlanScale")]  = m_buildingPlanScale;
        root[QStringLiteral("buildingPlanVisible")] = ui->planToggleButton->isChecked();
    }
    QJsonArray cells;
    for (int row = 0; row < kGridRows; ++row) {
        for (int col = 0; col < kGridCols; ++col) {
            const int idx = row * kGridCols + col;
            if (m_heatCounts[idx] <= 0) continue;
            QJsonObject cell;
            cell[QStringLiteral("col")]   = col;
            cell[QStringLiteral("row")]   = row;
            cell[QStringLiteral("sum")]   = m_heatSums[idx];
            cell[QStringLiteral("count")] = m_heatCounts[idx];
            cells.append(cell);
        }
    }
    root[QStringLiteral("heatCells")] = cells;

    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text))
        return false;
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return true;
}

bool MainWindow::loadHeatmapFromJson(const QString &path)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
        return false;

    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject())
        return false;

    const QJsonObject root = doc.object();

    const int cols = root[QStringLiteral("gridCols")].toInt(kGridCols);
    const int rows = root[QStringLiteral("gridRows")].toInt(kGridRows);
    if (cols != kGridCols || rows != kGridRows) {
        ui->statusLabel->setText(QStringLiteral("Помилка завантаження: невідповідність сітки (%1x%2 vs %3x%4)")
                                     .arg(cols).arg(rows).arg(kGridCols).arg(kGridRows));
        return false;
    }

    clearHeatHistory();

    if (root.contains(QStringLiteral("apAnchor"))) {
        const QJsonObject ap = root[QStringLiteral("apAnchor")].toObject();
        const QPointF cell(ap[QStringLiteral("x")].toDouble(),
                           ap[QStringLiteral("y")].toDouble());
        m_wifiAnchorCell = cell;
        m_hasWifiAnchor  = true;
        ui->heatmap->setWifiAnchor(cell);
        ui->heatmap->setApCell(cell, m_metersPerCell);
    }

    if (root.contains(QStringLiteral("buildingPlanBase64"))) {
        const QByteArray imgBytes = QByteArray::fromBase64(
            root[QStringLiteral("buildingPlanBase64")].toString().toLatin1());
        QPixmap pixmap;
        if (pixmap.loadFromData(imgBytes, "PNG")) {
            ui->heatmap->setBuildingPlan(pixmap);
            const double planScale = root[QStringLiteral("buildingPlanScale")].toDouble(1.0);
            m_buildingPlanScale = planScale;
            ui->heatmap->setBuildingPlanScale(planScale);
            if (ui->planScaleSlider)
                ui->planScaleSlider->setValue(static_cast<int>(qRound(planScale * 100.0)));
            updatePlanScaleLabel();
            const bool visible = root[QStringLiteral("buildingPlanVisible")].toBool(true);
            ui->heatmap->setBuildingPlanVisible(visible);
            if (ui->planToggleButton)
                ui->planToggleButton->setChecked(visible);
        }
    }

    const QJsonArray cells = root[QStringLiteral("heatCells")].toArray();
    for (const QJsonValue &v : cells) {
        const QJsonObject c = v.toObject();
        const int col   = c[QStringLiteral("col")].toInt();
        const int row   = c[QStringLiteral("row")].toInt();
        if (col < 0 || col >= kGridCols || row < 0 || row >= kGridRows) continue;
        const int idx = row * kGridCols + col;
        m_heatSums[idx]   = c[QStringLiteral("sum")].toDouble();
        m_heatCounts[idx] = c[QStringLiteral("count")].toInt();
    }

    QVector<QPointF> positions;
    QVector<double>  values;
    for (int row = 0; row < kGridRows; ++row) {
        for (int col = 0; col < kGridCols; ++col) {
            const int idx = row * kGridCols + col;
            if (m_heatCounts[idx] <= 0) continue;
            positions.push_back(QPointF(col, row));
            values.push_back(m_heatSums[idx] / m_heatCounts[idx]);
        }
    }
    ui->heatmap->setHeatSamples(positions, values);
    return true;
}

void MainWindow::onSaveHeatmap()
{
    const QString path = QFileDialog::getSaveFileName(
        this,
        QStringLiteral("Зберегти теплову карту"),
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation),
        QStringLiteral("Heatmap JSON (*.json)"));
    if (path.isEmpty()) return;

    if (saveHeatmapToJson(path))
        ui->statusLabel->setText(QStringLiteral("Карту збережено: %1").arg(QFileInfo(path).fileName()));
    else
        ui->statusLabel->setText(QStringLiteral("Помилка збереження: %1").arg(path));
}

void MainWindow::onLoadHeatmap()
{
    const QString path = QFileDialog::getOpenFileName(
        this,
        QStringLiteral("Завантажити теплову карту"),
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation),
        QStringLiteral("Heatmap JSON (*.json)"));
    if (path.isEmpty()) return;

    if (loadHeatmapFromJson(path))
        ui->statusLabel->setText(QStringLiteral("Карту завантажено: %1").arg(QFileInfo(path).fileName()));
    else
        ui->statusLabel->setText(QStringLiteral("Помилка завантаження: %1").arg(path));
}

void MainWindow::onBuildingPlanClicked()
{
    const QString filePath = QFileDialog::getOpenFileName(
        this,
        QStringLiteral("Обрати план будівлі"),
        QString(),
        QStringLiteral("Images (*.png *.jpg *.jpeg *.bmp *.gif *.webp)"));
    if (filePath.isEmpty()) {
        return;
    }

    QPixmap pixmap(filePath);
    if (pixmap.isNull()) {
        ui->statusLabel->setText(QStringLiteral("Не вдалося завантажити зображення плану."));
        return;
    }

    ui->heatmap->setBuildingPlan(pixmap);
    if (ui->planToggleButton) {
        ui->planToggleButton->setChecked(true);
    }
    ui->heatmap->setBuildingPlanVisible(true);
    ui->statusLabel->setText(QStringLiteral("План завантажено: %1").arg(QFileInfo(filePath).fileName()));
}

void MainWindow::onBuildingPlanToggled(bool checked)
{
    ui->heatmap->setBuildingPlanVisible(checked);
}

void MainWindow::onBuildingPlanScaleChanged(int value)
{
    m_buildingPlanScale = qMax(0.1, static_cast<double>(value) / 100.0);
    updatePlanScaleLabel();
    ui->heatmap->setBuildingPlanScale(m_buildingPlanScale);
}

void MainWindow::onMetersPerCellChanged(int value)
{
    m_metersPerCell = value / 100.0;
    if (ui->metersPerCellLabel) {
        ui->metersPerCellLabel->setText(
            QStringLiteral("м/клітинка: %1 м").arg(m_metersPerCell, 0, 'f', 2));
    }
    clearHeatHistory();
    m_lastEstimatedNodes.clear();
    m_hasLastEstimate = false;
    updateVisualization();
    if (m_hasWifiAnchor) {
        ui->heatmap->setApCell(m_wifiAnchorCell, m_metersPerCell);
    }
}

void MainWindow::onMqttConnected()
{
    QMqttSubscription *subscription = m_mqttClient->subscribe(m_mqttTopic, 0);
    if (!subscription) {
        ui->statusLabel->setText(
            QStringLiteral("Підключено, але підписка не вдалась для %1").arg(m_mqttTopic));
    }
}

double MainWindow::toSignedRssi(double v) const
{
    return v > 0.0 ? -v : v;
}

QString MainWindow::formatCell(const QPointF &p) const
{
    return QStringLiteral("(%1,%2)").arg(p.x(), 0, 'f', 2).arg(p.y(), 0, 'f', 2);
}

double MainWindow::rssiToDistanceCells(double rssi) const
{
    return rssiToDistance(rssi) / m_metersPerCell;
}

double MainWindow::bleRssiToDistanceCells(double rssi) const
{
    return bleRssiToDistance(rssi) / m_metersPerCell;
}

void MainWindow::updateDebugPanel(const QVector<double> &values, const QVector<QPointF> &estimatedNodes)
{
    if (!ui->debugText) {
        return;
    }

    const double avgRssi01 = (toSignedRssi(values.value(1)) + toSignedRssi(values.value(4))) / 2.0;
    const double avgRssi02 = (toSignedRssi(values.value(2)) + toSignedRssi(values.value(7))) / 2.0;
    const double avgRssi12 = (toSignedRssi(values.value(5)) + toSignedRssi(values.value(8))) / 2.0;
    QString text;
    text += QStringLiteral("Шлях логу: %1\n").arg(m_logPath);
    text += QStringLiteral("Поточні значення RSSI (сирі/згладжені):\n");
    text += QStringLiteral("MASTER wifi=%1 ble1=%2 ble2=%3\n")
                .arg(values.value(0), 0, 'f', 2)
                .arg(values.value(1), 0, 'f', 2)
                .arg(values.value(2), 0, 'f', 2);
    text += QStringLiteral("NODE1  wifi=%1 bleM=%2 bleP=%3\n")
                .arg(values.value(3), 0, 'f', 2)
                .arg(values.value(4), 0, 'f', 2)
                .arg(values.value(5), 0, 'f', 2);
    text += QStringLiteral("NODE2  wifi=%1 bleM=%2 bleP=%3\n\n")
                .arg(values.value(6), 0, 'f', 2)
                .arg(values.value(7), 0, 'f', 2)
                .arg(values.value(8), 0, 'f', 2);
    text += QStringLiteral("RSSI -> відстань (клітинки):\n");
    text += QStringLiteral("Відстані до ТД: M=%1 N1=%2 N2=%3\n")
                .arg(rssiToDistanceCells(toSignedRssi(values.value(0))), 0, 'f', 2)
                .arg(rssiToDistanceCells(toSignedRssi(values.value(3))), 0, 'f', 2)
                .arg(rssiToDistanceCells(toSignedRssi(values.value(6))), 0, 'f', 2);
    text += QStringLiteral("BLE сер. d01=%1  d02=%2  d12=%3\n")
                .arg(bleRssiToDistanceCells(avgRssi01), 0, 'f', 2)
                .arg(bleRssiToDistanceCells(avgRssi02), 0, 'f', 2)
                .arg(bleRssiToDistanceCells(avgRssi12), 0, 'f', 2);
    if (estimatedNodes.size() == 3) {
        text += QStringLiteral("\nОцінені вузли: M=%1 N1=%2 N2=%3\n")
                    .arg(formatCell(estimatedNodes[0]))
                    .arg(formatCell(estimatedNodes[1]))
                    .arg(formatCell(estimatedNodes[2]));
    }
    ui->debugText->setPlainText(text);

}

void MainWindow::updateVisualization()
{
    const QVector<double> values = m_chipData.composeHeatmapValues();
    if (!m_hasWifiAnchor) {
        ui->heatmap->setEstimatedNodes({}, {});
        ui->heatmap->setHeatSamples({}, {});
        ui->statusLabel->setText(QStringLiteral("Спочатку встановіть точку доступу: ліва кнопка миші на сітці (верх = ПІВНІЧ)."));
        return;
    }

    const QVector<QPointF> estimatedNodes = estimateNodePositions(values, m_wifiAnchorCell);
    const QStringList labels{QStringLiteral("M"), QStringLiteral("N1"), QStringLiteral("N2")};
    m_lastEstimatedNodes = estimatedNodes;
    m_hasLastEstimate = (estimatedNodes.size() == 3);
    ui->heatmap->setEstimatedNodes(estimatedNodes, labels);


    QVector<double> wifiSamples;
    wifiSamples << values.value(0, -100.0) << values.value(3, -100.0) << values.value(6, -100.0);
    if (wifiSamples[0] <= -99.0 || wifiSamples[1] <= -99.0 || wifiSamples[2] <= -99.0)
        return;
    updateHeatHistory(estimatedNodes, wifiSamples);
    updateDebugPanel(values, estimatedNodes);

    ui->statusLabel->setText(QStringLiteral("Позиції вузлів оновлено за RSSI (наближена трилатерація)."));
}

void MainWindow::initLogging()
{
    QString baseDir = QCoreApplication::applicationDirPath();

    QDir dir(baseDir);
    dir.mkpath(QStringLiteral("logs"));

    const QString stamp = QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    m_logPath = dir.filePath(QStringLiteral("logs/session_%1.csv").arg(stamp));

    m_logFile = new QFile(m_logPath);
    if (m_logFile->open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(m_logFile);
        out << "timestamp_utc,"
                "master_wifi,master_ble1,master_ble2,"
                "node1_wifi,node1_ble_master,node1_ble_peer,"
                "node2_wifi,node2_ble_master,node2_ble_peer,"
                "master_x,master_y,node1_x,node1_y,node2_x,node2_y,"
                "dist_master_node1,dist_master_node2,dist_node1_node2\n";
        out.flush();
    }
}

void MainWindow::appendLog(const QVector<QPointF> &estimatedNodes,
                           const QVector<double>  &values)
{
    if (!m_logFile || !m_logFile->isOpen())
        return;

    const bool hasEstimate = estimatedNodes.size() == 3;

    const double d01 = hasEstimate ? cellDist(estimatedNodes[0], estimatedNodes[1]) * m_metersPerCell : 0.0;
    const double d02 = hasEstimate ? cellDist(estimatedNodes[0], estimatedNodes[2]) * m_metersPerCell : 0.0;
    const double d12 = hasEstimate ? cellDist(estimatedNodes[1], estimatedNodes[2]) * m_metersPerCell : 0.0;

    QTextStream out(m_logFile);
    out << QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs) << ','
        << values.value(0) << ','
        << values.value(1) << ','
        << values.value(2) << ','
        << values.value(3) << ','
        << values.value(4) << ','
        << values.value(5) << ','
        << values.value(6) << ','
        << values.value(7) << ','
        << values.value(8) << ',';

    if (hasEstimate) {
        out << estimatedNodes[0].x() << ',' << estimatedNodes[0].y() << ','
            << estimatedNodes[1].x() << ',' << estimatedNodes[1].y() << ','
            << estimatedNodes[2].x() << ',' << estimatedNodes[2].y() << ','
            << d01 << ',' << d02 << ',' << d12;
    } else {
        out << ",,,,,,,,,,";
    }

    out << '\n';
    out.flush();
}


void MainWindow::setupMqtt()
{
    const MqttConfig cfg = MqttConfigLoader::load(kBrokerHost, kBrokerPort, kTopic);
    m_mqttTopic = cfg.topic;

    m_mqttClient->setHostname(cfg.host);
    m_mqttClient->setPort(cfg.port);
    m_mqttClient->setUsername(cfg.username);
    m_mqttClient->setPassword(cfg.password);

    connect(m_mqttClient, &QMqttClient::stateChanged, this, &MainWindow::onMqttStateChanged);
    connect(m_mqttClient, &QMqttClient::errorChanged, this, &MainWindow::onMqttErrorChanged);
    connect(m_mqttClient, &QMqttClient::messageReceived, this, &MainWindow::onMqttMessageReceived);

    connect(m_mqttClient, &QMqttClient::connected, this, &MainWindow::onMqttConnected);

    if (cfg.fromFile) {
        ui->statusLabel->setText(QStringLiteral("Конфігурацію MQTT завантажено з %1").arg(cfg.sourcePath));
    } else {
        ui->statusLabel->setText(QStringLiteral("Використовується вбудована конфігурація MQTT. Файл не знайдено: %1").arg(cfg.sourcePath));
    }

    m_mqttClient->connectToHost();
}

void MainWindow::setupConnections()
{
    connect(ui->planLoadButton,       &QPushButton::clicked, this, &MainWindow::onBuildingPlanClicked);
    connect(ui->planToggleButton,     &QPushButton::toggled, this, &MainWindow::onBuildingPlanToggled);
    connect(ui->planScaleSlider,      &QSlider::valueChanged, this, &MainWindow::onBuildingPlanScaleChanged);
    connect(ui->metersPerCellSlider,  &QSlider::valueChanged, this, &MainWindow::onMetersPerCellChanged);
    connect(ui->saveHeatmapButton,    &QPushButton::clicked, this, &MainWindow::onSaveHeatmap);
    connect(ui->loadHeatmapButton,    &QPushButton::clicked, this, &MainWindow::onLoadHeatmap);
    connect(ui->heatmap, &HeatmapWidget::wifiAnchorChanged, this, &MainWindow::onWifiAnchorChanged);
    connect(ui->refRssiSlider,    &QSlider::valueChanged, this, &MainWindow::onRefRssiChanged);
    connect(ui->bleRefRssiSlider, &QSlider::valueChanged, this, &MainWindow::onBleRefRssiChanged);
    connect(ui->pathLossSlider,   &QSlider::valueChanged, this, &MainWindow::onPathLossChanged);
    connect(ui->actionAbout, &QAction::triggered, this, &MainWindow::onAboutTriggered);

    QButtonGroup *ewGroup = new QButtonGroup(this);
    ewGroup->addButton(ui->ewWestButton, -1);
    ewGroup->addButton(ui->ewNoneButton,  0);
    ewGroup->addButton(ui->ewEastButton,  1);
    ewGroup->setExclusive(true);
    connect(ewGroup, &QButtonGroup::idClicked, this, &MainWindow::onEwBiasChanged);

    QButtonGroup *nsGroup = new QButtonGroup(this);
    nsGroup->addButton(ui->nsNorthButton, -1);
    nsGroup->addButton(ui->nsNoneButton,   0);
    nsGroup->addButton(ui->nsSouthButton,  1);
    nsGroup->setExclusive(true);
    connect(nsGroup, &QButtonGroup::idClicked, this, &MainWindow::onNsBiasChanged);

    ui->heatmap->setBuildingPlanScale(m_buildingPlanScale);
    if (m_hasWifiAnchor) {
        ui->heatmap->setApCell(m_wifiAnchorCell, m_metersPerCell);
    }
    updatePlanScaleLabel();
    onMetersPerCellChanged(ui->metersPerCellSlider->value());
}

double MainWindow::bleRssiToDistance(double rssi) const
{
    return qPow(10.0, (m_bleReferenceRssi - rssi) / (10.0 * m_pathLossExponent));
}

void MainWindow::onRefRssiChanged(int value)
{
    m_referenceRssi = static_cast<double>(value);
    ui->refRssiLabel->setText(
        QStringLiteral("WiFi опорне: %1 дБм").arg(m_referenceRssi, 0, 'f', 1));
    updateVisualization();
}

void MainWindow::onBleRefRssiChanged(int value)
{
    m_bleReferenceRssi = static_cast<double>(value);
    ui->bleRefRssiLabel->setText(
        QStringLiteral("BLE опорне: %1 дБм").arg(m_bleReferenceRssi, 0, 'f', 1));
    updateVisualization();
}

void MainWindow::onPathLossChanged(int value)
{
    m_pathLossExponent = value / 10.0;
    ui->pathLossLabel->setText(
        QStringLiteral("Загасання: %1").arg(m_pathLossExponent, 0, 'f', 1));
    updateVisualization();
}

void MainWindow::onEwBiasChanged(int id)
{
    m_ewBias = static_cast<double>(id) * 0.3;
    updateVisualization();
}

void MainWindow::onNsBiasChanged(int id)
{
    m_nsBias = static_cast<double>(id);
    updateVisualization();
}

void MainWindow::onAboutTriggered()
{
    AboutWindow *w = new AboutWindow(this);
    w->setAttribute(Qt::WA_DeleteOnClose);
    w->show();
}
