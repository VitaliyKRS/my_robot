//=============================================================================
//    Copyright (C) 2021-2022 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#include <fields2cover.h>
#include <vector>

#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>

namespace conversor {

namespace GeometryMsgs = geometry_msgs::msg;
namespace NavMsgs = nav_msgs::msg;

class ROS {
public:
    static void to(const F2CPoint& _point, GeometryMsgs::Point32& _p32);
    static void to(const F2CPoint& _point, GeometryMsgs::Point& _p64);

    template <class T>
    static void to(const T& _curve, GeometryMsgs::Polygon& _poly);

    static void to(const F2CCell& _poly, std::vector<GeometryMsgs::Polygon>& _ros_poly);

    static void to(const F2CCells& _polys,
                   std::vector<std::vector<GeometryMsgs::Polygon>>& _ros_polys);

    static void to(const F2CLineString& _line, NavMsgs::Path& _path);

    // static void to(const F2CLineString& _line,
    //  visualization_msgs::MarkerArray& _markers);
    // static void to(const OGRMultiLineString& _line,
    //  visualization_msgs::MarkerArray& _markers);
};

template <class T>
inline void ROS::to(const T& _curve, GeometryMsgs::Polygon& _poly)
{
    GeometryMsgs::Point32 p32;
    for (const auto& p : _curve) {
        to(p, p32);
        _poly.points.push_back(p32);
    }
}

}  // namespace conversor
