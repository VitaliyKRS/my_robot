#pragma once

#include <QSortFilterProxyModel>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>

#include "NodeModel.h"
#include "ParametersModel.h"
#include "RclEngine.h"

namespace Ui {
class PluginWidget;
}

namespace rviz::panel::reconfigure {

class PluginWidget final : public QWidget {
    Q_OBJECT
private:
    std::shared_ptr<Ui::PluginWidget> mUi;
    rclcpp::Node::SharedPtr mNode;
    RclEngine mRclEngine;
    NodeModel mNodesModel;
    ParametersModel mParametersModel;
    QSortFilterProxyModel mParametersProxyModel;
    QSortFilterProxyModel mNodesProxyModel;
    QString mCurrentNodeName;

public:
    explicit PluginWidget(QWidget* parent);

public:
    void setNode(rclcpp::Node::SharedPtr node);

public:
    void closeEvent(QCloseEvent* event) override;

private slots:
    void onRefreshClicked();
    void onNodeChanged(const QItemSelection& selected, const QItemSelection& deselected);
    void onParametersFilter();
    void onNodeFilter();
    void showContextMenu(const QPoint& pos);
    void onParameterChanged(const QString& name, const QVariant& value);
    void onParameterUpdated(const QString& name);
    void onParametersReceived(QVariant);
};

}  // namespace rviz::panel::reconfigure
