QT       += core gui mqtt

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    aboutwindow.cpp \
    chipdatamodel.cpp \
    heatmapwidget.cpp \
    main.cpp \
    mainwindow.cpp \
    mqttconfig.cpp

HEADERS += \
    aboutwindow.h \
    chipdatamodel.h \
    heatmapwidget.h \
    mainwindow.h \
    mqttconfig.h

FORMS += \
    aboutwindow.ui \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
