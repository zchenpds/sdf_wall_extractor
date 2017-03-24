QT += core
QT -= gui

CONFIG += c++11

TARGET = sdf_scene_extractor
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    tinyxml2.cpp

HEADERS += \
    tinyxml2.h
