
# git subtrees

The commands I ran for my reference:

    git subtree add --prefix ros/src/clam https://github.com/robomakery/clam.git indigo-devel --squash
    git subtree add --prefix ros/src/moveit_simple_grasps https://github.com/davetcoleman/moveit_simple_grasps.git indigo-devel --squash
    git subtree add --prefix ros/src/moveit_visual_tools https://github.com/davetcoleman/moveit_visual_tools.git f6023d8312798c4eb9718875bb024a7bd348f688 --squash
    git subtree add --prefix ros/src/graph_msgs https://github.com/davetcoleman/graph_msgs.git indigo-devel --squash
    git subtree add --prefix ros/src/rviz_visual_tools https://github.com/davetcoleman/rviz_visual_tools.git indigo-devel --squash
