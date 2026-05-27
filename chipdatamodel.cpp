#include "chipdatamodel.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QStringList>
#include <QtMath>

const int kExpectedValues = 9;
const double kZeroEpsilon = 0.0001;

static bool parseObject(const QByteArray &data, QJsonObject *outObj)
{
    QJsonParseError error;
    const QJsonDocument doc = QJsonDocument::fromJson(data, &error);
    if (error.error != QJsonParseError::NoError || !doc.isObject()) {
        return false;
    }
    *outObj = doc.object();
    return true;
}

static bool parseLooseObject(const QString &text, QJsonObject *outObj)
{
    QString s = text.trimmed();
    if (!(s.startsWith('{') && s.endsWith('}'))) {
        return false;
    }

    s = s.mid(1, s.size() - 2).trimmed();
    if (s.isEmpty()) {
        return false;
    }

    QJsonObject obj;
    const QStringList pairs = s.split(',', Qt::SkipEmptyParts);
    for (const QString &rawPair : pairs) {
        const QString pair = rawPair.trimmed();
        const int colonPos = pair.indexOf(':');
        if (colonPos <= 0) {
            return false;
        }

        QString key = pair.left(colonPos).trimmed();
        QString value = pair.mid(colonPos + 1).trimmed();

        if ((key.startsWith('"') && key.endsWith('"')) || (key.startsWith('\'') && key.endsWith('\''))) {
            key = key.mid(1, key.size() - 2);
        }
        if (key.isEmpty()) {
            return false;
        }

        bool isNumber = false;
        const double number = value.toDouble(&isNumber);
        if (isNumber) {
            obj.insert(key, number);
        } else {
            if ((value.startsWith('"') && value.endsWith('"'))
                || (value.startsWith('\'') && value.endsWith('\''))) {
                value = value.mid(1, value.size() - 2);
            }
            obj.insert(key, value);
        }
    }

    if (obj.isEmpty()) {
        return false;
    }

    *outObj = obj;
    return true;
}

ChipDataModel::ChipDataModel(double smoothingAlpha)
    : m_rssiSmoothingAlpha(smoothingAlpha)
{
}

void ChipDataModel::setSmoothingAlpha(double alpha)
{
    m_rssiSmoothingAlpha = qBound(0.01, alpha, 0.95);
}

double ChipDataModel::jsonDouble(const QJsonObject &obj, const QString &key, bool *ok)
{
    const QJsonValue v = obj.value(key);
    if (!v.isDouble()) {
        *ok = false;
        return 0.0;
    }
    *ok = true;
    return v.toDouble();
}

bool ChipDataModel::updateFromPayload(const QByteArray &payload, ChipSampleForLog *sampleForLog)
{
    QJsonObject obj;
    if (!parseObject(payload, &obj)) {
        QString text = QString::fromUtf8(payload).trimmed();
        if (text.size() >= 2 && text.startsWith('\'') && text.endsWith('\'')) {
            text = text.mid(1, text.size() - 2);
        }
        if (text.size() >= 2 && text.startsWith('"') && text.endsWith('"')) {
            text = text.mid(1, text.size() - 2);
            text.replace(QStringLiteral("\\\""), QStringLiteral("\""));
            text.replace(QStringLiteral("\\\\"), QStringLiteral("\\"));
        }
        if (!parseObject(text.toUtf8(), &obj) && !parseLooseObject(text, &obj)) {
            return false;
        }
    }

    if (obj.isEmpty()) {
        return false;
    }

    const QString chip = obj.value(QStringLiteral("chip")).toString().trimmed().toUpper();
    if (chip.isEmpty()) {
        return false;
    }
    m_lastUpdatedChip = chip;


    bool okWifi = false;
    const double wifi = jsonDouble(obj, QStringLiteral("wifi"), &okWifi);
    if (!okWifi) {
        return false;
    }

    bool okA = false;
    bool okB = false;
    double a = 0.0;
    double b = 0.0;

    if (chip == QStringLiteral("MASTER")) {
        a = jsonDouble(obj, QStringLiteral("ble1"), &okA);
        b = jsonDouble(obj, QStringLiteral("ble2"), &okB);
    } else {
        a = jsonDouble(obj, QStringLiteral("ble_master"), &okA);
        b = jsonDouble(obj, QStringLiteral("ble_peer"), &okB);
    }

    if (!okA || !okB) {
        return false;
    }

    const bool hasPrev = m_chipValues.contains(chip) && m_chipValues.value(chip).size() == 3;
    const QVector<double> prev = hasPrev ? m_chipValues.value(chip) : QVector<double>{-100.0, -100.0, -100.0};

    const bool zeroWifi = qAbs(wifi) <= kZeroEpsilon;
    const bool zeroA = qAbs(a) <= kZeroEpsilon;
    const bool zeroB = qAbs(b) <= kZeroEpsilon;

    const double mergedWifi = zeroWifi ? prev.at(0) : wifi;
    const double mergedA = zeroA ? prev.at(1) : a;
    const double mergedB = zeroB ? prev.at(2) : b;

    QVector<double> nextValues;
    if (!hasPrev || m_chipSampleCounts.value(chip, 0) <= 0) {
        nextValues = QVector<double>{mergedWifi, mergedA, mergedB};
    } else {
        const double jumpWifi = zeroWifi ? 0.0 : qAbs(mergedWifi - prev.at(0));
        const double jumpA = zeroA ? 0.0 : qAbs(mergedA - prev.at(1));
        const double jumpB = zeroB ? 0.0 : qAbs(mergedB - prev.at(2));

        const double alpha = (jumpWifi > 15.0 || jumpA > 15.0 || jumpB > 15.0) ? 1.0 : m_rssiSmoothingAlpha;
        nextValues = QVector<double>{
            zeroWifi ? prev.at(0) : prev.at(0) + alpha * (mergedWifi - prev.at(0)),
            zeroA ? prev.at(1) : prev.at(1) + alpha * (mergedA - prev.at(1)),
            zeroB ? prev.at(2) : prev.at(2) + alpha * (mergedB - prev.at(2))
        };
    }

    m_chipValues.insert(chip, nextValues);
    m_chipSampleCounts.insert(chip, m_chipSampleCounts.value(chip, 0) + 1);

    if (sampleForLog) {
        sampleForLog->chip = chip;
        sampleForLog->rawWifi = wifi;
        sampleForLog->rawA = a;
        sampleForLog->rawB = b;
        sampleForLog->usedWifi = mergedWifi;
        sampleForLog->usedA = mergedA;
        sampleForLog->usedB = mergedB;
        sampleForLog->smoothWifi = nextValues.at(0);
        sampleForLog->smoothA = nextValues.at(1);
        sampleForLog->smoothB = nextValues.at(2);
    }

    return true;
}

QVector<double> ChipDataModel::composeHeatmapValues() const
{
    QVector<double> values;
    values.reserve(kExpectedValues);

    const QVector<double> fallback{-100.0, -100.0, -100.0};

    values << m_chipValues.value(QStringLiteral("MASTER"), fallback);
    QStringList nodeNames = m_chipValues.keys();
    nodeNames.removeOne(QStringLiteral("MASTER"));
    nodeNames.sort(Qt::CaseInsensitive);


    values << m_chipValues.value(nodeNames.value(0), fallback);
    values << m_chipValues.value(nodeNames.value(1), fallback);

    return values;
}

bool ChipDataModel::consumeUpdateAndCheckCycleReady()
{
    if (!m_lastUpdatedChip.isEmpty()) {
        m_cycleSeen.insert(m_lastUpdatedChip);
    }

    int nodeSeen = 0;
    for (const QString &name : m_cycleSeen) {
        if (name != QStringLiteral("MASTER")) {
            nodeSeen++;
        }
    }

    if (m_cycleSeen.contains(QStringLiteral("MASTER")) && nodeSeen >= 2) {
        m_cycleSeen.clear();
        return true;
    }
    return false;
}

void ChipDataModel::clearSamplingState()
{
    m_chipSampleCounts.clear();
    m_cycleSeen.clear();
}
