#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32

def main():
    rospy.init_node('milestone1')
    vertices_to_visit = [1, 9, 17, 25, 33, 41, 49, 57, 65, 66, 67, 59]
    pub = rospy.Publisher('/goal', Int32)
    rospy.sleep(1)
    move_to_vertex(pub, 9)
    new_costs = read_edge_costs()


def read_edge_costs():
    rospy.loginfo('Reading in the current edge costs')
    costs = rospy.wait_for_message('/edge_costs', String).data
    costs_split = costs.split('\n')
    cost_list = []
    for c in costs_split:
        cost = c.split('\t')
        if len(cost) == 1:
            continue
        current_node, neighboring_node, edge_cost = int(cost[0]), int(cost[1]), float(cost[2])
        cost_list.append([current_node, neighboring_node, edge_cost])
        rospy.loginfo('%d -> %d : %f' % (current_node, neighboring_node, edge_cost))
    return cost_list


def move_to_vertex(pub, v):
    rospy.loginfo('Attempting to move to vertex ' + str(v))
    pub.publish(v)
    while True:
        node = rospy.wait_for_message('/current_node', String).data
        #print('Current Node', int(node.data))
        if int(node) == v:
            break
        rospy.sleep(0.5)
    rospy.loginfo('Successfully moved to vertex ' + str(v))



if __name__ == "__main__":
    main()