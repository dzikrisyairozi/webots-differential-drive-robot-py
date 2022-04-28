from collections import defaultdict
from math import sqrt, pow
from queue import PriorityQueue

maps = [[0, 0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0]]

graph = defaultdict(list)
dict = {}

def set_map_value(x, y, value):
  maps[y][x] = value

def a_star_search(source, target):
  visited = []
  traced = {}
  route = []
  p_queue = PriorityQueue()
  p_queue.put((dict[source], source, 0, None))
  
  while p_queue.empty() == False:
    current_cost, current_node, prev_cost, prev_node = p_queue.get()

    if current_node not in visited:
      visited.append(current_node)
      traced[current_node] = prev_node
      if current_node == target:
        route.append(current_node)
        # print("cost: ", current_cost)
        while prev_node != None:
          route.append(prev_node)
          prev_node = traced[prev_node]
        break

      for node, cost in graph[current_node]:
        total_cost = cost + prev_cost
        p_queue.put((dict[node] + total_cost, node, total_cost, current_node))

  route.reverse()
  return visited, traced, route

def addedge(x, y, cost):
  graph[x].append((y, cost))
  graph[y].append((x, cost))

def create_edge():
  n = len(maps)
  for i in range(n):
    for j in range(n):
      if maps[i][j] == 0:
        if i + 1 < n and maps[i+1][j] == 0:
          addedge((i, j), (i+1, j), 1)
        if j + 1 < n and maps[i][j+1] == 0:
          addedge((i, j), (i, j+1), 1)
        if j - 1 >= 0 and maps[i][j-1] == 0:
          addedge((i, j), (i, j-1), 1)

def set_heuristic():
  n = len(maps)
  for i in range(n):
    for j in range(n):
      if maps[i][j] == 0:
        dict[(i, j)] = round(sqrt(pow(n-i-1, 2) + pow(n-j-1, 2)), 2)

def get_route(start, finish):
  create_edge()
  set_heuristic()
  visited, traced, route = a_star_search(start, finish)
  return route

# print(get_route((1, 1), (8, 8)))
