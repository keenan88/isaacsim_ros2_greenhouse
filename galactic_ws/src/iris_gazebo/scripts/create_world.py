#!/usr/bin/env python
import os
import rospkg
import rospy
import math

rospack = rospkg.RosPack()
iris_gazebo_path = rospack.get_path('iris_gazebo')

# parse parameters
number_of_rows = rospy.get_param('world/number_of_rows', 10)
row_width = rospy.get_param('world/row_width', 1.5)
row_length = rospy.get_param('plants/length', 50)
aisle_width = rospy.get_param('world/aisle_width', 3.5)
aisle_origin_x = rospy.get_param('world/aisle_origin_x', 0.0)
world_name = rospy.get_param('world/name', 'greenhouse')
obstacles = rospy.get_param('obstacles', [])


def _xacro2sdf():

    # xacro to urdf
    os.system("xacro " + iris_gazebo_path + "/urdf/plants_single.urdf.xacro > " +
              iris_gazebo_path + "/urdf/plants.urdf")
    os.system("xacro " + iris_gazebo_path + "/urdf/rails_single.urdf.xacro > " +
              iris_gazebo_path + "/urdf/rails.urdf")

    # urdf to sdf
    os.system("gz sdf -p " + iris_gazebo_path + "/urdf/plants.urdf > " +
              iris_gazebo_path + "/models/plants_model/plants.sdf")
    os.system("gz sdf -p " + iris_gazebo_path + "/urdf/rails.urdf > " +
              iris_gazebo_path + "/models/rails_model/rails.sdf")


def _save_world():
    fin = open(iris_gazebo_path + "/worlds/param_world.txt", "rt")
    fout = open(iris_gazebo_path + f"/worlds/{world_name}.world", "wt")

    rails_total_width = number_of_rows * row_width
    plants_total_width = number_of_rows * row_width - row_width
    global_map_size_x = 2 * row_length + \
        aisle_width + abs(2*aisle_origin_x) + 1
    global_map_size_y = rails_total_width + abs(2*aisle_origin_x) + 1
    row_offset = row_width/2 - plants_total_width/2
    if global_map_size_x > global_map_size_y:
        map_size = math.ceil(global_map_size_x)
    else:
        map_size = math.ceil(global_map_size_x)

    # replace values from parametric world
    for line in fin:
        fout.write(line.replace('rails_total_width', str(rails_total_width)).
                   replace('rails_rows', str(number_of_rows)).
                   replace('plants_total_width', str(plants_total_width)).
                   replace('plants_rows', str(number_of_rows - 1)).
                   replace('row_offset', str(row_offset)).
                   replace('global_map_size_x', str(map_size)).
                   replace('global_map_size_y', str(map_size)).
                   replace('aisle_origin_x', str(aisle_origin_x)))

        if line.find('obstacles') != -1:
            for obstacle in obstacles:
                if 'box' in obstacle:
                    fout.writelines(_create_box_obstacle(
                        obstacles[obstacle]['pos_x'], obstacles[obstacle]['pos_y'], obstacle))

    fin.close()
    fout.close()


def _create_box_obstacle(pos_x, pos_y, name):
    obstacle_definition = [
        f'        <model name="{name}">\n',
        f'            <pose>{pos_x} {pos_y} 1 0 0 0</pose>\n',
        '            <static>false</static>\n',
        '            <link name="link">\n',
        '                <inertial>\n',
        '                    <mass>1.0</mass>\n',
        '                </inertial>\n',
        '                <collision name="collision">\n',
        '                    <geometry>\n',
        '                        <box>\n',
        '                            <size>0.8 0.8 1.6</size>\n',
        '                        </box>\n',
        '                    </geometry>\n',
        '                </collision>\n',
        '                <visual name="visual">\n',
        '                    <geometry>\n',
        '                        <box>\n',
        '                            <size>0.8 0.8 1.6</size>\n',
        '                        </box>\n',
        '                    </geometry>\n',
        '                </visual>\n',
        '            </link>\n',
        '        </model>\n',
    ]

    return obstacle_definition


if __name__ == '__main__':
    try:
        _xacro2sdf()
        _save_world()
    except rospy.ROSInterruptException:
        pass
