#ifndef MQTTCONFIG_H
#define MQTTCONFIG_H

#include <QString>
#include <QCoreApplication>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSettings>
#include <QTextStream>

struct MqttConfig
{
    QString host;
    quint16 port = 1883;
    QString topic;
    QString username;
    QString password;
    QString sourcePath;
    bool fromFile = false;
};

namespace MqttConfigLoader {
MqttConfig load(const QString &defaultHost, quint16 defaultPort, const QString &defaultTopic);
}

#endif // MQTTCONFIG_H
