#include "mqttconfig.h"



static void writeDefaultConfigIfMissing(const QString &path,
                                 const QString &defaultHost,
                                 quint16 defaultPort,
                                 const QString &defaultTopic)
{
    QFileInfo fi(path);
    if (fi.exists()) {
        return;
    }

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return;
    }

    QTextStream out(&file);
    out << "; MQTT configuration\n";
    out << "; Copy/edit this file and restart the app.\n";
    out << "[mqtt]\n";
    out << "host=" << defaultHost << "\n";
    out << "port=" << defaultPort << "\n";
    out << "topic=" << defaultTopic << "\n";
    out << "username=\n";
    out << "password=\n";
}

MqttConfig MqttConfigLoader::load(const QString &defaultHost, quint16 defaultPort, const QString &defaultTopic)
{
    MqttConfig cfg;
    cfg.host = defaultHost;
    cfg.port = defaultPort;
    cfg.topic = defaultTopic;

    const QString appConfig = QCoreApplication::applicationDirPath() + QStringLiteral("/mqtt_config.ini");
    const QString cwdConfig = QDir::currentPath() + QStringLiteral("/mqtt_config.ini");

    QFileInfo fi(appConfig);
    if (fi.exists() && fi.isFile()) {
        cfg.sourcePath = appConfig;
    } else {
        fi.setFile(cwdConfig);
        if (fi.exists() && fi.isFile()) {
            cfg.sourcePath = cwdConfig;
        } else {
            cfg.sourcePath = appConfig;
            writeDefaultConfigIfMissing(cfg.sourcePath, defaultHost, defaultPort, defaultTopic);
            return cfg;
        }
    }

    QSettings settings(cfg.sourcePath, QSettings::IniFormat);
    cfg.fromFile = true;

    const QString host = settings.value(QStringLiteral("mqtt/host"), cfg.host).toString().trimmed();
    if (!host.isEmpty()) {
        cfg.host = host;
    }

    bool okPort = false;
    const int port = settings.value(QStringLiteral("mqtt/port"), static_cast<int>(cfg.port)).toInt(&okPort);
    if (okPort && port > 0 && port <= 65535) {
        cfg.port = static_cast<quint16>(port);
    }

    const QString topic = settings.value(QStringLiteral("mqtt/topic"), cfg.topic).toString().trimmed();
    if (!topic.isEmpty()) {
        cfg.topic = topic;
    }

    cfg.username = settings.value(QStringLiteral("mqtt/username")).toString();
    cfg.password = settings.value(QStringLiteral("mqtt/password")).toString();
    return cfg;
}
