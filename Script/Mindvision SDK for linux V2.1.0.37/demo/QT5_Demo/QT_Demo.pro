# -------------------------------------------------
# Project created by QtCreator 2014-03-18T10:40:12
# -------------------------------------------------

QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = QT_Demo
TEMPLATE = app

INCLUDEPATH += "../../include/"
LIBS += -lMVSDK

SOURCES += main.cpp \
    mainwindow.cpp \
    capturethread.cpp
HEADERS += mainwindow.h \
    capturethread.h
FORMS += mainwindow.ui
