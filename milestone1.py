#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
import math
import pprint

pp = pprint.PrettyPrinter(indent=4)
resolution = rospy.get_param('/grid_step')
grid_size_x = rospy.get_param('/grid_size_x') + 1
grid_size_y = rospy.get_param('/grid_size_y') + 1

def main():
    rospy.init_node('milestone1')
    goals = [59, 11, 16, 9]
    rospy.loginfo('Current configuration is %d by %d with resolution %f' % (grid_size_x, grid_size_y, resolution))
    pub = rospy.Publisher('/goal', Int32, queue_size=10)
    rospy.sleep(2)
    graph = {}
    coordinates = {}
    coordinates[goals[0]] = convert_vertex_to_xy(goals[0])
    rospy.loginfo('Attempting to move to goal position %d' % goals[0])
    while True:
        new_costs, current_vertex = read_edge_costs()
        if current_vertex == goals[0]:
            rospy.loginfo('Successfully moved to goal position %d' % goals[0])
            del goals[0]
            if len(goals) == 0:
                rospy.loginfo('Successfully moved to all goal locations')
                return
            else:
                rospy.loginfo('Attempting to move to goal position %d' % goals[0])
                coordinates[goals[0]] = convert_vertex_to_xy(goals[0])
        if current_vertex not in graph:
            coordinates[current_vertex] = convert_vertex_to_xy(current_vertex)
            for e in new_costs:
                coordinates[e[1]] = convert_vertex_to_xy(e[1])
            for c in new_costs:
                add_to_graph(graph, c[0], c[1], c[2])
  #      pp.pprint(graph)
        path, path_cost, itr_count = find_path(current_vertex, goals[0], graph, coordinates, 1, compute_euclidean_distance)
        for v in path:
            move_to_vertex(pub, v)


def convert_vertex_to_xy(v):
    x = math.ceil(float(v)/float(grid_size_y))
    y = v % grid_size_y
    if y == 0:
        y = grid_size_y
    return [int(x), int(y)]

def read_edge_costs():
 #   rospy.loginfo('Reading in the current edge costs')
    costs = rospy.wait_for_message('/edge_costs', String).data
    costs_split = costs.split('\n')
    cost_list = []
    curr_node = None
    for c in costs_split:
        cost = c.split('\t')
        if len(cost) == 1:
            continue
        current_node, neighboring_node, edge_cost = int(cost[0]), int(cost[1]), float(cost[2])
        curr_node = current_node
        cost_list.append([current_node, neighboring_node, edge_cost])
 #       rospy.loginfo('%d -> %d : %f' % (current_node, neighboring_node, edge_cost))

    return cost_list, curr_node


def move_to_vertex(pub, v):
    rospy.loginfo('Moving to vertex ' + str(v))
    pub.publish(v)
    while True:
        node = rospy.wait_for_message('/current_node', String).data
        #print('Current Node', int(node.data))
        if int(node) == v:
            break
        rospy.sleep(0.5)
    rospy.loginfo('Completed move to vertex ' + str(v))


def add_to_graph(graph, key, value, cost):
    if int(key) in graph:
        graph[int(key)].append((int(value), float(cost)))
    else:
        graph[int(key)] = [(int(value), float(cost))]


def find_path_a_star(start, end, graph, coordinates):
    print('Performing A Star search with euclidean distance heuristic')
    return find_path(start, end, graph, coordinates, 1, None)


def compute_euclidean_distance(p1, p2):
    val = math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    return val


def find_path(start, end, graph, coordinates, weight, heuristic):
    rospy.loginfo('Running A*')
    open_list = [start]
    closed_list = []

    cost_to_come = {}
    back_trace = {}
    for v in graph.keys():
        cost_to_come[v] = float('inf')
        back_trace[v] = 0

    cost_to_come[start] = 0
    itr_count = 0
    while end not in closed_list:
        '''
        s = 'Open List  : '
        cst = 'Cost Of    : '
        for c in open_list:
            s += '{0: <5} '.format(c)
            cst += '%.3f ' % cost_to_come[c]
        print s
        print cst
        print 'Closed List: ', closed_list'''
        # Get the lowest cost to come from open list
        if heuristic is None:
            selected_node = min(open_list, key=lambda x: cost_to_come[x])
        else:
            selected_node = min(open_list, key=lambda x: cost_to_come[x] + weight * heuristic(coordinates[x],
                                                                                                  coordinates[end]))
       # print 'Selected node', selected_node
        # Remove it from the open list and store it in the closed list

        if selected_node in graph:
            #print('Min:', selected_node, '->', graph[selected_node])
            for n in graph[selected_node]:
                if n[0] not in closed_list:
                    neighbor = n[0]
                    cost = n[1]
                #    print '     neighbor =', neighbor, ' with cost', cost
                    vnew = cost + cost_to_come[selected_node]
                #    print '     new cost to come =', vnew
                    if neighbor not in cost_to_come:
                        cost_to_come[neighbor] = vnew
                        back_trace[neighbor] = selected_node
                    elif vnew < cost_to_come[neighbor]:
                        #print('Replacing the current entry for', neighbor, 'with old cost', cost_to_come[neighbor - 1],
                        #      'with new cost', vnew)
                        cost_to_come[neighbor] = vnew
                        back_trace[neighbor] = selected_node
                    if neighbor not in open_list:
                        open_list.append(neighbor)
            open_list.remove(selected_node)
            closed_list.append(selected_node)
        else:
     #       print 'Back Trace Table'
      #      pp.pprint(back_trace)
      #      rospy.loginfo('Node to visit ' + str(selected_node))
            path = [selected_node]
            itr = back_trace[selected_node]
            while itr != start:
                path.append(itr)
                itr = back_trace[itr]
            path.reverse()
            rospy.loginfo('Path to take: ' + str(path))
            show_graph(graph, closed_list, open_list, selected_node, coordinates, start, end)
            return path, [], itr_count
        itr_count += 1
#    print 'Back Trace Tabled'
 #   pp.pprint(back_trace)
    path = [end]
    itr = back_trace[end]
    while itr != start and itr != 0:
        path.append(itr)
        itr = back_trace[itr]
    path.reverse()
  #  rospy.loginfo('Path to take: ' + str(path))
    return path, [], itr_count

import networkx as nx
import numpy as np
import matplotlib.pyplot as plt

def show_graph(graph, closed, openlist, selected, coordinates, start, end):
    plt.ion()
    plt.clf()

    G = nx.DiGraph()


    G.add_node(start, pos=coordinates[start])
    G.add_node(end, pos=coordinates[end])

    for k in graph.keys():
        if not G.has_node(k):
            G.add_node(k, pos=coordinates[k])
        for e in graph[k]:
            if not G.has_node(e[0]):
                G.add_node(e[0], pos=coordinates[e[0]])
            G.add_edge(k, e[0], weight=e[1])
    pos = nx.get_node_attributes(G, 'pos')
    nodes_set = set(G.nodes())
    for n in closed:
        nodes_set.remove(n)
    for n in openlist:
        nodes_set.remove(n)
    if start in nodes_set:
        nodes_set.remove(start)
    if end in nodes_set:
        nodes_set.remove(end)
    not_considered = list(nodes_set)
    nx.draw_networkx_nodes(G, pos, nodelist=not_considered, node_size=100, alpha=1, node_color='gray')
    nx.draw_networkx_nodes(G, pos, nodelist=openlist, node_size=100, alpha=1, node_color='blue')
    nx.draw_networkx_nodes(G, pos, nodelist=closed, node_size=100, alpha=1, node_color='white')
    nx.draw_networkx_nodes(G, pos, nodelist=[selected], node_size=100, alpha=1, node_color='yellow')
    nx.draw_networkx_nodes(G, pos, nodelist=[start], node_size=100, alpha=1, node_color='red')
    nx.draw_networkx_nodes(G, pos, nodelist=[end], node_size=100, alpha=1, node_color='green')
    elarge = [(u, v) for (u, v, d) in G.edges(data=True) if d['weight'] == float('inf')]
    esmall = [(u, v) for (u, v, d) in G.edges(data=True) if d['weight'] != float('inf')]
    nx.draw_networkx_edges(G, pos, edgelist=elarge, width=2, edge_color='black', style='dashed')
    nx.draw_networkx_edges(G, pos, edgelist=esmall, width=2, edge_color='green')
    plt.axis('equal')
    plt.axis('off')
    plt.pause(2)
if __name__ == "__main__":
    main()