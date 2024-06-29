# TODO: Import libraries
import heapq 
import time
import tracemalloc
from collections import deque, defaultdict
# 1. Search Strategies Implementation
# 1.1. Breadth-first search (BFS)
def myfunc(a):
    return a & (a & 1)
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
  visited = set()
  queue = []
  queue.append(source)
  visited.add(source)
  while (queue):
    curNode = queue.pop(0)
    path.append(curNode)
    if (curNode == destination):
      return visited, path
    for i in range(len(arr[curNode])):
      if (i not in visited and arr[curNode][i] !=  0):
        queue.append(i)
        visited.add(i)
  return -1


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
  visited = set()
  stack = []
  stack.append(source)
  visited.add(source)
  while (stack):
    curNode = stack.pop()
    path.append(curNode)
    if (curNode == destination):
      return visited, path
    for i in range(len(arr[curNode])):
      if (i not in visited and arr[curNode][i] !=  0):
        stack.append(i)
        visited.add(i)
  return -1



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

  path = []
  visited = {}

  return visited, path


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

  return visited, path


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
  # TODO: Stop measuring 

  # TODO: Show the output data

  pass