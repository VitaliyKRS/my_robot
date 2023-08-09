#pragma once

#include <memory>

#include <rviz_common/panel.hpp>

#include "RclEngine.h"

namespace rviz::panel::reconfigure {

class PluginWidget;

class PluginPanel : public rviz_common::Panel {
private:
    std::shared_ptr<PluginWidget> mWidget;

public:
    explicit PluginPanel(QWidget* parent = nullptr);

public:  // rviz_common::Panel
    void onInitialize() override;
    void save(rviz_common::Config config) const override;
    void load(const rviz_common::Config& conf) override;
};

}  // namespace rviz::panel::reconfigure

