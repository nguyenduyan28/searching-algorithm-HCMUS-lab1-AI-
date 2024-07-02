import tracemalloc
import time
import heapq 
import random
from collections import deque, defaultdict

# Helper function to output the path
def outputPath(source, visited, goal):
  path = []
  while goal != source:
    path.append(goal)
    # path.append(" -> ")
    goal = visited[goal]
  path.append(source)
  path.reverse()

  return ' -> '.join(map(str, path))

# 1.1. Breadth-first search (BFS)
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
  path = []
  visited = {}
  queue = deque([source])
  visited[source] = None  # Mark the source as visited
  while queue:
    curNode = queue.popleft()
    path.append(curNode)
    if curNode == destination:
      return  outputPath(source, visited, destination)
    for i in range(len(arr[curNode])):
      if arr[curNode][i] != 0 and i not in visited:
        queue.append(i)
        visited[i] = curNode
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
  path = []
  visited = {}
  stack = [source]
  visited[source] = None  # Mark the source as visited
  while stack:
    curNode = stack.pop()
    path.append(curNode)
    if curNode == destination:
      return  outputPath(source, visited, destination)
    for i in range(len(arr[curNode])):
      if arr[curNode][i] != 0 and i not in visited:
        stack.append(i)
        visited[i] = curNode
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
  pqueue = []
  visited = {}
  heapq.heappush(pqueue, (0, source))
  cost_min = {source: 0}
  close = set()
  while pqueue:
    value, curNode = heapq.heappop(pqueue)
    if curNode in close:
      continue
    close.add(curNode)
    if curNode == destination:
      return outputPath(source, visited, destination)
    for neighbor in range(len(arr[curNode])):
      curCost = arr[curNode][neighbor]
      if curCost > 0 and neighbor not in close:
        newCost = curCost + value
        if neighbor not in cost_min or newCost < cost_min[neighbor]:
          cost_min[neighbor] = newCost
          heapq.heappush(pqueue, (newCost, neighbor))
          visited[neighbor] = curNode
  return -1
# 1.4.a Depth-limited search (DLS)
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
  path = []
  visited = {source : 0}
  stack = []
  stack.append((source, 0))
  while (stack):
    curNode, curDepth = stack.pop()
    if (curNode == destination):
      return 1, outputPath(source, visited, destination)
    if (curDepth >= depth_limit):
      return 0, []
    for neighbor in range(len(arr[curNode])):
      if (arr[curNode][neighbor] != 0 and neighbor not in visited):
        stack.append((neighbor, curDepth + 1))
        visited[neighbor] = curNode

  return -1, []

# 1.4.b. IDS
def ids(arr, source, destination, depth_limit):
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
  for limit in range(depth_limit):
    found, path = dls(arr, source, destination, limit)
    if found == 0:
      continue
    elif found == -1:
      return -1
    elif found == 1:
      return path
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
  pqueue = []
  visited = {}
  heapq.heappush(pqueue, (heuristic[source], source))
  close = set()
  while pqueue:
    _, curNode = heapq.heappop(pqueue)
    if curNode in close:
      continue
    close.add(curNode)
    if curNode == destination:
      return  outputPath(source, visited, destination)
    for neighbor in range(len(arr[curNode])):
      if arr[curNode][neighbor] != 0 and neighbor not in close:
        heapq.heappush(pqueue, (heuristic[neighbor], neighbor))
        visited[neighbor] = curNode
  return -1
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
  pqueue = []
  visited = {source: 0}
  cost_min = {source: 0}
  heapq.heappush(pqueue, (heuristic[source], source))
  close = set()
  while pqueue:
    curCost, curNode = heapq.heappop(pqueue)
    if curNode in close:
      continue
    close.add(curNode)
    if curNode == destination:
      return  outputPath(source, visited, destination)
    for neighbor in range(len(arr[curNode])):
      if arr[curNode][neighbor] > 0 and neighbor not in close:
        newCost = cost_min[curNode] + arr[curNode][neighbor]
        if neighbor not in cost_min or newCost < cost_min[neighbor]:
          cost_min[neighbor] = newCost
          heapq.heappush(pqueue, (newCost + heuristic[neighbor], neighbor))
          visited[neighbor] = curNode
  return -1

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
  visited = {source: 0}
  curNode = source
  while True:
    if curNode == destination:
      return  outputPath(source, visited, destination)
    getBetterNode = False
    neighbors = [i for i, values in enumerate(arr[curNode])]
    random.shuffle(neighbors)
    for neighbor in neighbors:
      if arr[curNode][neighbor] != 0 and heuristic[neighbor] < heuristic[curNode] and neighbor not in visited:
        visited[neighbor] = curNode
        curNode = neighbor
        getBetterNode = True
        break
    if not getBetterNode:
      break
  return -1

def readInput(filename):
  f = open(filename, 'r')
  rawData = f.read().split('\n')
  num = int(rawData[0])
  source, goal = map(int, rawData[1].split())
  arr = [list(map(int, rawData[i].split())) for i in range(2, 2 + num)]
  heuristic = list(map(int, rawData[2 + num].split()))
  return num, source, goal, arr, heuristic



def measure_performance(algorithm, arr, source, goal, heuristic=None, depth_limit=None):
  tracemalloc.start()
  start_time = time.time()

  if algorithm == 'bfs':
    path = bfs(arr, source, goal)
  elif algorithm == 'dfs':
    path = dfs(arr, source, goal)
  elif algorithm == 'ucs':
    path = ucs(arr, source, goal)
  elif algorithm == 'dls':
    found, path = dls(arr, source, goal, 2)
    if (found is False):
      path = -1
  elif algorithm == 'ids':
    path = ids(arr, source, goal, depth_limit)
  elif algorithm == 'gbfs':
    path = gbfs(arr, source, goal, heuristic)
  elif algorithm == 'astar':
    path = astar(arr, source, goal, heuristic)
  elif algorithm == 'hc':
    path = hc(arr, source, goal, heuristic)
  else:
    path = None

  end_time = time.time()
  current, peak = tracemalloc.get_traced_memory()
  tracemalloc.stop()

  runtime = end_time - start_time
  memory_usage = peak / 1024  # Convert to KB

  return path, runtime, memory_usage



# 2. Main function
if __name__ == "__main__":
  # TODO: Read the input data
  num, source, goal,  arr, heuristic = readInput("input.txt")
  # TODO: Start measuring
  # TODO: Call a function to execute the path finding process

  # Execute the path finding process
  algorithms = ['bfs', 'dfs', 'ucs', 'dls', 'ids', 'gbfs', 'astar', 'hc']
  for algorithm in algorithms:
    path, runtime, memory = measure_performance(algorithm, arr, source, goal, heuristic, depth_limit=99)
    print(f"{algorithm.upper()}")
    print("Path: ", path)
    print("Runtime: ", runtime)
    print("Memory usage: ", memory, "KB")
  # print("BFS:", bfs(arr, source, goal))
  # print("DFS:", dfs(arr, source, goal))
  # print("UCS:", ucs(arr, source, goal))
  # print("DLS:", dls(arr, source, goal, 2))
  # print("IDS:", ids(arr, source, goal, 10))
  # print("GBFS:", gbfs(arr, source, goal, heuristic))
  # print("AStar:", astar(arr, source, goal, heuristic))
  # print("HC:", hc(arr, source, goal, heuristic))
  # TODO: Stop measuring 

  # TODO: Show the output data