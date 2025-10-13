^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tiago_pro_head_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2025-08-27)
------------------
* remove realsense overlay
* Contributors: antoniobrandi

1.5.0 (2025-06-18)
------------------

1.4.2 (2025-03-25)
------------------

1.4.1 (2025-02-26)
------------------
* Put back old eps because this should be fixed for heads after the first one
* Add Realsense D455 to tiago pro head
* Update tiago_pro_head.urdf.xacro
* Fix head_2_link oriontation
* Fix and reduce collision meshhes
* Fix screen link + base_link + update link_2 position stl
* Increase the safety factor due to the cable getting compressed at joint limits
* Remove unecessary link + fix base_link inertia + update collision meshes + add new head_base_link meshes
* Add libgazebo_ros_video plugin to TIAGo Pro Head
* Contributors: thomas.peyrucain, thomaspeyrucain

1.4.0 (2025-01-16)
------------------
* Merge branch 'tpe/simplify-3d-model' into 'humble-devel'
  Add simplyfied models
  See merge request robots/tiago_pro_head_robot!13
* revert collision mesh
* Add simplyfied models
* Contributors: thomas.peyrucain, thomaspeyrucain

1.3.0 (2025-01-08)
------------------

1.2.0 (2024-12-02)
------------------

1.1.0 (2024-12-02)
------------------
* Rotate sellion link
* Contributors: Aina

1.0.0 (2024-11-29)
------------------
* Update module priority
* Contributors: Aina

0.0.2 (2024-11-28)
------------------

0.0.1 (2024-11-18)
------------------
* Add condition for cameras in urdf
* Fix path for ros2 controller gazebo cfg file
* Add gazebo plugin in ros2_control
* Update pm2
* Fix control system name
* Update dependency from xacro
* Add proper collision files for the base
* Align with tiago_pro fix on the origins of the base links for the head
* Add camera argument
* Add xacro tests
* Add proper urdfs to description pkg & ros2 control
* Change official name
* Contributors: Aina, davidterkuile
