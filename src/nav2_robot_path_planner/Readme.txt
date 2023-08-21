To use this path planner Nav2 plugin just replace in the nav2_params.yaml (located 
tracked_robot_navigation/params/nav2_params.yaml) the string 
'plugin: "nav2_navfn_planner/NavfnPlanner"'
to 
'plugin: "robot_global_planner::RobotGlobalPlanner"' in the planner_server section