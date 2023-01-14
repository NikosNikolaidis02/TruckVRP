import random
import math

class Node:
    def __init__(self, idd, xx, yy, dem=0, st=0):
        self.x = xx
        self.y = yy
        self.ID = idd
        self.isRouted = False
        self.demand = dem
        self.serv_time = st


def load_model(file_name):
    all_nodes = []
    all_lines = list(open(file_name, "r"))

    separator = ','

    line_counter = 0
    ln = all_lines[line_counter]
    no_spaces = ln.split(sep=separator)
    vehicles = int(no_spaces[1])

    line_counter += 1
    ln = all_lines[line_counter]
    no_spaces = ln.split(sep=separator)
    capacity = int(no_spaces[1])

    line_counter += 1
    ln = all_lines[line_counter]
    no_spaces = ln.split(sep=separator)
    tot_customers = int(no_spaces[1])

    line_counter += 3
    ln = all_lines[line_counter]

    no_spaces = ln.split(sep=separator)
    x = float(no_spaces[1])
    y = float(no_spaces[2])
    depot = Node(0, x, y)
    all_nodes.append(depot)

    for i in range(tot_customers):
        line_counter += 1
        ln = all_lines[line_counter]
        no_spaces = ln.split(sep=separator)
        idd = int(no_spaces[0])
        x = float(no_spaces[1])
        y = float(no_spaces[2])
        demand = int(no_spaces[3])
        serv_time = int(no_spaces[4])
        customer = Node(idd, x, y, demand, serv_time)
        all_nodes.append(customer)

    return all_nodes, vehicles, capacity


def distance(from_node, to_node):
    dx = from_node.x - to_node.x
    dy = from_node.y - to_node.y
    dist = math.sqrt(dx ** 2 + dy ** 2)
    return dist


def calculate_route_details(nodes_sequence):
    rt_load = 0
    rt_cumulative_cost = 0
    tot_time = 0
    for i in range(len(nodes_sequence) - 1):
        from_node = nodes_sequence[i]
        to_node = nodes_sequence[i+1]
        tot_time += distance(from_node, to_node)
        rt_cumulative_cost += tot_time
        tot_time += to_node.serv_time
        rt_load += from_node.demand
    return rt_cumulative_cost, rt_load


def test_solution(file_name, all_nodes, vehicles, capacity):
    all_lines = list(open(file_name, "r"))
    line = all_lines[1]
    cost_reported = float(line.strip())
    cost_calculated = 0

    times_visited = {}
    for i in range(1, len(all_nodes)):
        times_visited[i] = 0

    line = all_lines[3]
    vehs_used = int(line.strip())

    if vehs_used > vehicles:
        print('More than', vehicles, 'used in the solution')
        return

    separator = ','
    line_counter = 4
    for i in range(vehs_used):
        ln = all_lines[line_counter]
        ln = ln.strip()
        no_commas = ln.split(sep=separator)
        ids = [int(no_commas[i]) for i in range(len(no_commas))]
        nodes_sequence = [all_nodes[idd] for idd in ids]
        rt_cumulative_time, rt_load = calculate_route_details(nodes_sequence)
        for nn in range(1, len(nodes_sequence)):
            n_in = nodes_sequence[nn].ID
            times_visited[n_in] = times_visited[n_in] + 1
        # check capacity constraints
        if rt_load > capacity:
            print('Capacity violation. Route', i, 'total load is', rt_load)
            return
        cost_calculated += rt_cumulative_time
        line_counter += 1
    # check solution objective
    if abs(cost_calculated - cost_reported) > 0.001:
        print('Cost Inconsistency. Cost Reported', cost_reported, '--- Cost Calculated', cost_calculated)
        return

    # Check number of times each customer is visited
    for t in times_visited:
        if times_visited[t] != 1:
            print('Error: customer', t, 'not present once in the solution')
            return

    # everything is ok
    print('Solution is ΟΚ. Total Cost:', cost_calculated)

all_nodes, vehicles, capacity = load_model('Instance.txt')
test_solution('solution.txt', all_nodes, vehicles, capacity)
