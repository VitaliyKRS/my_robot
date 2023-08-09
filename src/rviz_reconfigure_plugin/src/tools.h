#pragma once

#include <string>

namespace rviz::panel::reconfigure {
namespace tools {

bool getNameAndNamespace(const std::string& fullName, std::string ns, std::string& name);

}  // namespace tools
}  // namespace rviz::panel::reconfigure
