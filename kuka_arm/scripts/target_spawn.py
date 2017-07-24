#! /usr/bin/env python

# Author: Harsh Pandya

# import ros modules
import yaml
import rospy
import rospkg
import sys
from random import randint


def update_spawn_location():
    # open the spawn locations file
    rospack = rospkg.RosPack()
    config_path = rospack.get_path('kuka_arm') + "/config/target_spawn_locations.yaml"
    with open(config_path, 'r') as doc:
        location = yaml.load(doc)

    # get select_target_spawn parameter
    select_target_spawn = rospy.get_param('/target_spawn/select_target_spawn')

    if select_target_spawn < 0 or select_target_spawn > 9:
        sys.exit("Invalid Target Spawn point selected, valid range is 0-9")
    elif select_target_spawn == 0:
        print("Random Target Spawn point selected")
        select = randint(1, 9)
    else:
        select = select_target_spawn

    x_select = location["locations"][select - 1][0]
    y_select = location["locations"][select - 1][1]
    z_select = location["locations"][select - 1][2]

    # set bin location param
    rospy.set_param('target_drop_location', {'x': 0.0, 'y': 2.5, 'z': 0.0})

    # set selected location param
    rospy.set_param('target_spawn_location', {'x': x_select, 'y': y_select, 'z': z_select})

    # set target description argument
    target_desc_arg = "-urdf -param target_description -x " + str(x_select) + " -y " + str(y_select) + " -z " + str(z_select) + " -model target_model"
    rospy.set_param('target_description_argument', target_desc_arg)

    print"Target will spawn at location# %d: %f %f %f" % (select, x_select, y_select, z_select)


if __name__ == '__main__':
    update_spawn_location()
