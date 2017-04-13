#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
import math
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


def add_to_graph(graph, key, value, cost):
    if int(key) in graph:
        graph[int(key)].append((int(value), float(cost)))
    else:
        graph[int(key)] = [(int(value), float(cost))]


def find_path_a_star(number_of_vertices, start, end, graph, coordinates):
    print('Performing A Star search with euclidean distance heuristic')
    return find_path(number_of_vertices, start, end, graph, coordinates, 1, compute_euclidean_distance)


def compute_euclidean_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def find_path(number_of_vertices, start, end, graph, coordinates, weight, heuristic):
    open_list = [start]
    closed_list = []

    cost_to_come = {}
    back_trace = {}
    for v in graph.keys():
        cost_to_come[v] = math.inf
        back_trace[v] = 0

    cost_to_come[start] = 0
    itr_count = 0
    while end not in closed_list:
        '''s = 'Open List  : '
        cst = 'Cost Of    : '
        for c in open_list:
            s += '{0: <5} '.format(c)
            cst += '%.3f ' % cost_to_come[c - 1]
        print(s)
        print(cst)
        print('Closed List: ', closed_list)'''
        # Get the lowest cost to come from open list
        if heuristic is None:
            selected_node = min(open_list, key=lambda x: cost_to_come[x])
        else:
            selected_node = min(open_list, key=lambda x: cost_to_come[x] + weight * heuristic(coordinates[x - 1],
                                                                                                  coordinates[end - 1]))

        # Remove it from the open list and store it in the closed list
        open_list.remove(selected_node)
        closed_list.append(selected_node)

        if selected_node in graph:
            #print('Min:', selected_node, '->', graph[selected_node])
            for n in graph[selected_node]:
                if n[0] not in closed_list:
                    neighbor = n[0]
                    cost = n[1]

                    vnew = cost + cost_to_come[selected_node]
                    if vnew < cost_to_come[neighbor]:
                        #print('Replacing the current entry for', neighbor, 'with old cost', cost_to_come[neighbor - 1],
                        #      'with new cost', vnew)
                        cost_to_come[neighbor] = vnew
                        back_trace[neighbor] = selected_node
                    if neighbor not in open_list:
                        open_list.append(neighbor)
        else:
            
        itr_count += 1

    path_cost = cost_to_come[end]
    path = [end]
    itr = back_trace[end]
    while itr != 0:
        path.append(itr)
        itr = back_trace[itr]
    path.reverse()
    print(path)
    return path, path_cost, itr_count

if __name__ == "__main__":
    main()