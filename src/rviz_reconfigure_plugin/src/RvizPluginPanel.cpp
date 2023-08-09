#include "RvizPluginPanel.h"

#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>

#include "PluginWidget.h"

namespace rviz::panel::reconfigure {

PluginPanel::PluginPanel(QWidget* parent)
    : Panel{parent}
{
    qRegisterMetaType<ParameterItem>("ParameterItem");
    qRegisterMetaType<QList<ParameterItem>>("ParameterItemList");

    auto metatype = QMetaType::fromType<QList<rviz::panel::reconfigure::ParameterItem>>();
    qDebug() << "Metatype:" << metatype.isValid();

    mWidget = std::make_shared<PluginWidget>(parent);
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget(mWidget.get());
    layout->setContentsMargins(10, 10, 10, 10);
    setLayout(layout);
}

void PluginPanel::onInitialize()
{
    auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    mWidget->setNode(node);
}

void PluginPanel::save(rviz_common::Config config) const { Panel::save(config); }

void PluginPanel::load(const rviz_common::Config& config) { Panel::load(config); }

}  // namespace rviz::panel::reconfigure

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::panel::reconfigure::PluginPanel, rviz_common::Panel)
