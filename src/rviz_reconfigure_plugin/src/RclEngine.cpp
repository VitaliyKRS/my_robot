#include "RclEngine.h"

namespace rviz::panel::reconfigure {

ParameterItem::ParameterItem(QString name, QVariant value, bool isReadonly)
    : mName{name}
    , mValue{value}
    , mIsReadonly{isReadonly}
{
}

ParameterItem::ParameterItem(const ParameterItem& other)
    : mName{other.mName}
    , mValue{other.mValue}
    , mIsReadonly{other.mIsReadonly}
{
}

ParameterItem& ParameterItem::operator=(const ParameterItem& other)
{
    if (this != &other) {
        mName = other.mName;
        mValue = other.mValue;
        mIsReadonly = other.mIsReadonly;
    }

    return *this;
}

RclEngine::RclEngine() { mNode = std::make_shared<rclcpp::Node>("reconfigure_node"); }

bool RclEngine::enumerateNodes()
{
    if (mThread.isRunning()) {
        return false;
    }

    mThread.setCallback(std::bind(&RclEngine::enumNodesFunc, this));
    mThread.start();
    return true;
}

bool RclEngine::enumerateParameters(const QString& nodeName)
{
    qDebug() << "Start enumerate parameters";
    if (mThread.isRunning()) {
        return false;
    }

    mThread.setCallback(std::bind(&RclEngine::enumParametersFunc, this, nodeName));
    mThread.start();
    return true;
}

bool RclEngine::setParameterValue(const QString& nodeName,
                                  const QString& parameterName,
                                  const QVariant& value)
{
    if (mThread.isRunning()) {
        return false;
    }

    mThread.setCallback(
        std::bind(&RclEngine::setParameterValueFunc, this, nodeName, parameterName, value));
    mThread.start();
    return true;
}

void RclEngine::setParameterValueFunc(QString nodeName, QString parameterName, QVariant value)
{
    auto client = std::make_shared<rclcpp::SyncParametersClient>(mNode, nodeName.toStdString());
    std::shared_ptr<rclcpp::ParameterValue> parameterValue;
    switch (value.type()) {
    case QVariant::Bool:
        parameterValue = std::make_shared<rclcpp::ParameterValue>(value.toBool());
        break;
    case QVariant::Int:
        parameterValue = std::make_shared<rclcpp::ParameterValue>(value.toInt());
        break;
    case QVariant::Double:
        parameterValue = std::make_shared<rclcpp::ParameterValue>(value.toDouble());
        break;
    case QVariant::String:
        parameterValue = std::make_shared<rclcpp::ParameterValue>(value.toString().toStdString());
        break;
    default:
        break;
    }

    rclcpp::Parameter parameter{parameterName.toStdString(), *parameterValue};
    auto response = client->set_parameters({parameter});
    if (!response.empty()) {
        if (response[0].successful) {
            qDebug() << "Parameter" << parameterName << "changed";

            emit parameterUpdated(parameterName);
        }
        else {
            qDebug() << "Failed to change parameter" << parameterName << "due to"
                     << response[0].reason.c_str();
        }
    }
}

void RclEngine::enumParametersFunc(QString nodeName)
{
    QList<ParameterItem> result;
    auto client = std::make_shared<rclcpp::SyncParametersClient>(mNode, nodeName.toStdString());
    auto parameters = client->list_parameters({}, 0);
    auto paramsTypes = client->get_parameter_types(parameters.names);
    auto paramsDesc = client->describe_parameters(parameters.names);
    for (size_t i = 0; i < parameters.names.size(); i++) {
        auto parameterName = parameters.names[i];
        auto paramType = paramsTypes[i];
        if (paramType == rclcpp::ParameterType::PARAMETER_NOT_SET) {
            qDebug() << "Type for" << parameterName.c_str();
            continue;
        }

        QVariant parameterValue;
        switch (paramType) {
        case rclcpp::ParameterType::PARAMETER_BOOL: {
            auto value = client->get_parameter(parameterName, false);
            parameterValue.setValue(value);
        } break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE: {
            auto value = client->get_parameter(parameterName, 0.0);
            parameterValue.setValue(value);
        } break;
        case rclcpp::ParameterType::PARAMETER_INTEGER: {
            auto value = client->get_parameter(parameterName, 0);
            parameterValue.setValue(value);
        } break;
        case rclcpp::ParameterType::PARAMETER_STRING: {
            auto value = client->get_parameter(parameterName, std::string{});
            parameterValue.setValue(QString::fromStdString(value));
        } break;
        default:
            break;
        }

        if (!parameterValue.isNull()) {
            auto pName = QString::fromStdString(parameterName);
            result.append({pName, parameterValue, paramsDesc[i].read_only});
        }
    }

    emit parametersReceived(QVariant::fromValue(result));
}

void RclEngine::enumNodesFunc()
{
    auto nodeNames = mNode->get_node_graph_interface()->get_node_names_and_namespaces();
    std::sort(nodeNames.begin(), nodeNames.end());
    std::unique(nodeNames.begin(), nodeNames.end());

    QStringList names;
    for (auto& name : nodeNames) {
        try {
            if (hasNodeServices(name.first, name.second)) {
                if (name.second.length() < 2) {
                    names.append(QString{(name.second + name.first).c_str()});
                }
                else {
                    names.append(QString{(name.second + "/" + name.first).c_str()});
                }
            }
        }
        catch (std::runtime_error& ex) {
            qWarning() << "Failed to query services for" << name.first.c_str()
                       << name.second.c_str();
            qWarning() << ex.what();
        }
    }

    emit nodeReceived(names);
}

bool RclEngine::hasNodeServices(std::string& nodeName, std::string& nodeNamespace)
{
    std::list<std::string> requiredServices = {"set_parameters", "get_parameters",
                                               "list_parameters"};
    auto counter = 0;
    auto services = mNode->get_node_graph_interface()->get_service_names_and_types_by_node(
        nodeName, nodeNamespace);
    for (auto service : services) {
        std::string serviceName, serviceNs;
        if (tools::getNameAndNamespace(service.first, serviceNs, serviceName)) {
            auto iter = std::find_if(
                requiredServices.begin(), requiredServices.end(),
                [&serviceName](const std::string& item) { return item == serviceName; });

            if (iter != requiredServices.end()) {
                counter++;
            }
        }
    }

    return counter == 3;
}

}  // namespace rviz::panel::reconfigure
