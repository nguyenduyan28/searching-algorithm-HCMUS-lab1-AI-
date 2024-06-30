# TODO: Import libraries
import heapq 
import time
import tracemalloc
from collections import deque, defaultdict
# 1. Search Strategies Implementation
# 1.1. Breadth-first search (BFS)
def myfunc(a):
    return a & (a & 1)
def outputPath(source, visited, goal):
  path = []
  while (visited[goal] != source):
    path.append(goal)
    goal = visited[goal]
  path.append(source)
  path.reverse()
  return path


def bfs(arr, source, destination):
  """
  BFS algorithm:
  Parameters:
  ---------------------------
  arr: list / numpy array 
      The graph's adjacency matrix
  source: integer
      Starting node
  destination: integer
      Ending node
  
  Returns
  ---------------------
  visited: dictionary
      The dictionary contains visited nodes, each key is a visited node,
      each value is the adjacent node visited before it.
  path: list
      Founded path
  """
  # TODO
  'arr[i][j] = path from i -- j'
  path = []
  visited = {}
  queue = []
  queue.append(source)
  while (queue):
    curNode = queue.pop(0)
    path.append(curNode)
    if (curNode == destination):
      return True, outputPath(source, visited, destination)
    for i in range(len(arr[curNode])):
      if (i not in visited and arr[curNode][i] !=  0):
        queue.append(i)
        visited[i] = curNode
  return -1, []


# 1.2. Depth-first search (DFS)
def dfs(arr, source, destination):
  """
  DFS algorithm:
  Parameters:
  ---------------------------
  arr: list / numpy array 
      The graph's adjacency matrix
  source: integer
      Starting node
  destination: integer
      Ending node
  
  Returns
  ---------------------
  visited: dictionary
      The dictionary contains visited nodes, each key is a visited node,
      each value is the adjacent node visited before it.
  path: list
      Founded path
  """
  # TODO

  path = []
  visited = {}
  stack = []
  stack.append(source)
  while (stack):
    curNode = stack.pop()
    path.append(curNode)
    if (curNode == destination):
      return True, outputPath(source, visited, destination)
    for i in range(len(arr[curNode])):
      if (i not in visited and arr[curNode][i] !=  0):
        stack.append(i)
        visited[i] = curNode
  return -1, []



# 1.3. Uniform-cost search (UCS)
def ucs(arr, source, destination):
  """
  UCS algorithm:
  Parameters:
  ---------------------------
  arr: list / numpy array 
      The graph's adjacency matrix
  source: integer
      Starting node
  destination: integer
      Ending node
  
  Returns
  ---------------------
  visited: dictionary
      The dictionary contains visited nodes, each key is a visited node,
      each value is the adjacent node visited before it.
  path: list
      Founded path
  """
  # TODO
  pqueue = []
  path = []
  visited = {}
  pqueue.append((0, source))
  cost_min = {source : 0}
  while(pqueue):
    pqueue = sorted(pqueue)
    value, curNode = pqueue.pop(0)
    if (curNode == destination):
      return True, outputPath(source, visited, destination)
    for neighbor in range(len((arr[curNode]))):
      curCost = arr[curNode][neighbor]
      if (curCost > 0 and neighbor not in visited):
        newCost = curCost + value
        if (neighbor not in cost_min or newCost < cost_min[neighbor]):
          # update
          cost_min[neighbor] = newCost
          pqueue.append((newCost, neighbor))
          visited[neighbor] = curNode

  return -1, []


# 1.4. Iterative deepening search (IDS)
# 1.4.a. Depth-limited search
def dls(arr, source, destination, depth_limit):
  """
  DLS algorithm:
  Parameters:
  ---------------------------
  arr: list / numpy array 
      The graph's adjacency matrix
  source: integer
      Starting node
  destination: integer
      Ending node
  depth_limit: integer
      Maximum depth for search
  
  Returns
  ---------------------
  visited: dictionary
      The dictionary contains visited nodes, each key is a visited node,
      each value is the adjacent node visited before it.
  path: list
      Founded path
  """
  # TODO

  path = []
  visited = {}
  stack = []
  stack.append((0, source))
  depth = 0
  while (stack):
    if (depth <= depth_limit):
      value,  curNode = stack.pop()
      if (curNode == destination):
       return True,  outputPath(source, visited, destination)
      for neighbor in range(len(arr[curNode])):
        if (arr[curNode][neighbor] != 0 and neighbor not in visited):
          stack.append((arr[curNode][neighbor], neighbor))
          visited[neighbor] = curNode
      depth += 1
    else:
      return False, []
  
  return False, []


# 1.4.b. IDS
def ids(arr, source, destination):
  """
  IDS algorithm:
  Parameters:
  ---------------------------
  arr: list / numpy array 
      The graph's adjacency matrix
  source: integer
      Starting node
  destination: integer
      Ending node
  
  Returns
  ---------------------
  visited: dictionary
      The dictionary contains visited nodes, each key is a visited node,
      each value is the adjacent node visited before it.
  path: list
      Founded path
  """
  # TODO

  path = []
  visited = {}

  return visited, path


# 1.5. Greedy best first search (GBFS)
def gbfs(arr, source, destination, heuristic):
  """
  GBFS algorithm:
  Parameters:
  ---------------------------
  arr: list / numpy array 
      The graph's adjacency matrix
  source: integer
      Starting node
  destination: integer
      Ending node
  heuristic: list / numpy array
      The heuristic value from the current node to the goal
  
  Returns
  ---------------------
  visited: dictionary
      The dictionary contains visited nodes, each key is a visited node,
      each value is the adjacent node visited before it.
  path: list
      Founded path
  """
  # TODO

  path = []
  visited = {}

  return visited, path


# 1.6. Graph-search A* (AStar)
def astar(arr, source, destination, heuristic):
  """
  A* algorithm:
  Parameters:
  ---------------------------
  arr: list / numpy array 
      The graph's adjacency matrix
  source: integer
      Starting node
  destination: integer
      Ending node
  heuristic: list / numpy array
      The heuristic value from the current node to the goal
  
  Returns
  ---------------------
  visited: dictionary
      The dictionary contains visited nodes, each key is a visited node,
      each value is the adjacent node visited before it.
  path: list
      Founded path
  """
  # TODO

  path = []
  visited = {}

  return visited, path


# 1.7. Hill-climbing First-choice (HC)
def hc(arr, source, destination, heuristic):
  """
  HC algorithm:
  Parameters:
  ---------------------------
  arr: list / numpy array 
      The graph's adjacency matrix
  source: integer
      Starting node
  destination: integer
      Ending node
  heuristic: list / numpy array
      The heuristic value from the current node to the goal
  
  Returns
  ---------------------
  visited: dictionary
      The dictionary contains visited nodes, each key is a visited node,
      each value is the adjacent node visited before it.
  path: list
      Founded path
  """
  # TODO

  path = []
  visited = {}

  return visited, path

def readInput(filename):
  f = open(filename, 'r')
  rawData = f.read().split('\n')
  num = int(rawData[0])
  rawData[ : ] = [rawData[i].split() for i in range(0, num + 2)]
  source, goal = rawData[1]
  source = int(source)
  goal = int(goal)
  arr = [[]]
  # arr = list(map(list, rawData[2 : 2 + num]))
  arr = list(map(list,  rawData[2: 2 + num]))
  for i in range(0,len(arr)):
    arr[i] = list(map(lambda x: int(x), arr[i]))

  heuristic = [] 
  heuristic[:] = [int(h) for h in rawData[num]]
  return num, source, goal, arr, heuristic
  



# 2. Main function
if __name__ == "__main__":
  # TODO: Read the input data
  num, source, goal,  arr, heuristic = readInput("input.txt")
  # TODO: Start measuring
  # TODO: Call a function to execute the path finding process
  print(bfs(arr, source, goal))
  print(dfs(arr, source, goal))
  print(ucs(arr, source, goal))
  print(dls(arr, source, goal, 2))
  # TODO: Stop measuring 

  # TODO: Show the output data

  pass