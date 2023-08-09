#pragma once

#include <algorithm>

#include <QDebug>
#include <QThread>

#include <rcl_interfaces/srv/list_parameters.h>
#include <rclcpp/rclcpp.hpp>

#include "tools.h"

namespace rviz::panel::reconfigure {

class ParameterItem {
private:
    QString mName;
    QVariant mValue;
    bool mIsReadonly;

public:
    ParameterItem() = default;

    ParameterItem(QString name, QVariant value, bool isReadonly);

    ParameterItem(const ParameterItem& other);

public:
    inline QString getName() const { return mName; }
    inline QVariant getValue() const { return mValue; }
    inline bool isReadonly() const { return mIsReadonly; }
    inline void setData(const QVariant& value) { mValue = value; }

public:
    ParameterItem& operator=(const ParameterItem& other);
};

class EnumerationThread : public QThread {
    Q_OBJECT
private:
    std::function<void()> mCallback;

public:
    EnumerationThread(QObject* parent = nullptr)
        : QThread{parent}
    {
    }

    void setCallback(std::function<void()> callback) { mCallback = callback; }

protected:
    void run() override
    {
        if (mCallback) {
            mCallback();
        }
    };

private:
};

class RclEngine : public QObject {
    Q_OBJECT
private:
    std::shared_ptr<rclcpp::Node> mNode;
    EnumerationThread mThread{this};

public:
    RclEngine();

public:
    bool enumerateNodes();

    bool enumerateParameters(const QString& nodeName);

    bool setParameterValue(const QString& nodeName,
                           const QString& paramName,
                           const QVariant& value);

private:
    void setParameterValueFunc(QString nodeName, QString paramName, QVariant value);

    void enumParametersFunc(QString nodeName);

    void enumNodesFunc();

    bool hasNodeServices(std::string& nodeName, std::string& nodeNamespace);

signals:
    void nodeReceived(QStringList names);
    void parametersReceived(QVariant parameters);
    void parameterUpdated(const QString& name);
};

}  // namespace rviz::panel::reconfigure

Q_DECLARE_METATYPE(QList<rviz::panel::reconfigure::ParameterItem>);
Q_DECLARE_METATYPE(rviz::panel::reconfigure::ParameterItem);
