#include "tools.h"

namespace rviz::panel::reconfigure {
namespace tools {

bool getNameAndNamespace(const std::string& fullName, std::string ns, std::string& name)
{
    if (fullName.empty()) {
        return false;
    }

    auto index = fullName.find_last_of("/");
    if (index == std::string::npos) {
        return false;
    }

    index++;

    name = fullName.substr(index);
    if (index == 1) {
        ns = fullName.substr(0, index);
    }
    else {
        ns = fullName.substr(0, index - 1);
    }

    return true;
}

}  // namespace tools
}  // namespace rviz::panel::reconfigure
