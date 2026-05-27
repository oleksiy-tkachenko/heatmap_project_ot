#ifndef CHIPDATAMODEL_H
#define CHIPDATAMODEL_H

#include <QByteArray>
#include <QMap>
#include <QSet>
#include <QString>
#include <QVector>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QStringList>
#include <QtMath>

struct ChipSampleForLog
{
    QString chip;
    double rawWifi = 0.0;
    double rawA = 0.0;
    double rawB = 0.0;
    double usedWifi = 0.0;
    double usedA = 0.0;
    double usedB = 0.0;
    double smoothWifi = 0.0;
    double smoothA = 0.0;
    double smoothB = 0.0;
};

class ChipDataModel
{
public:
    explicit ChipDataModel(double smoothingAlpha = 0.6);

    void setSmoothingAlpha(double alpha);
    bool updateFromPayload(const QByteArray &payload, ChipSampleForLog *sampleForLog = nullptr);
    QVector<double> composeHeatmapValues() const;
    bool consumeUpdateAndCheckCycleReady();
    void clearSamplingState();

private:
    double jsonDouble(const QJsonObject &obj, const QString &key, bool *ok);
    double m_rssiSmoothingAlpha;
    QMap<QString, QVector<double>> m_chipValues;
    QMap<QString, int> m_chipSampleCounts;
    QString m_lastUpdatedChip;
    QSet<QString> m_cycleSeen;
};

#endif // CHIPDATAMODEL_H
