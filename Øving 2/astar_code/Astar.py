import Map

class Node:

    #Node class to keep track of defined properties
    def __init__(self, position):
        self.position = position

        self.g = 0      #Distance between the start node and the current node
        self.h  = 0     #Heuristic - estimated distance from the current node to the end node.
        self.f = 0      #g+h - Total cost of node

        self.parent = None #In order to backtrack movement, the parent node must be tracked
        self.children = []

    def __str__(self):
        return "x: " + str(self.position[0]) + ", y: " + str(self.position[1]) + ", cost: " + str(self.f)
    
    def generate_all_successors(self, mapObject):
        """
        Generates all successors of the node
        :param mapObject: 
        """
        #Adjacent coordinates in the cardinal directions
        northPos = [self.position[0]-1, self.position[1]]
        southPos = [self.position[0]+1, self.position[1]]
        westPos = [(self.position[0]), self.position[1]-1]
        eastPos = [self.position[0], self.position[1]+1]
        
        adjCoordinates = [northPos, southPos, westPos, eastPos]
        children = []
        #Ads the adjacent node to children if value is valid
        for cor in adjCoordinates:
            try: 
                cell_value = mapObject.get_cell_value(cor)
                if cell_value >= 0:
                    children.append(Node(cor))
            except IndexError:
                continue
        return children

def manhattan_distance(nodePosition, goalPosition):
    """
    :param nodePosition: The position of the node in focus
    :param goalPosition: The position of the goal node
    :return: Returns the Manhattan distance between the two nodes
    """
    return abs(nodePosition[0]-goalPosition[0]) + abs(nodePosition[1]-goalPosition[1])

def best_first_search(mapObject):
    """
    :param mapObject: Map object to be considered
    :return: Returns a node if the node is the goal node. If goal node not found, it returns "No path is found".
    """
    closed, opened = [], []

    goalPosition = mapObject.get_goal_pos()
    startPosition = mapObject.get_start_pos()

    startNode = Node(startPosition)
    opened.append(startNode)

    targetNode = startNode
    while targetNode.position != goalPosition:
        if len(opened) == 0: 
            # If no valid path is found when all nodes are checked, the program failed
            return "No path is found"

        targetNode = opened.pop()
        closed.append(targetNode)

        if targetNode.position == goalPosition:
            return targetNode

        succ = targetNode.generate_all_successors(mapObject)

        for s in succ:
            #checks if node is in opened or closed, to avoid duplication the node
            s_prime = check_for_node_in_list(s, opened, closed)

            if s_prime not in opened and s_prime not in closed:
                attach_and_eval(s_prime, targetNode, mapObject)
                opened.append(s_prime)
                #Sort list by f value in ascending order
                opened.sort(key=lambda node: node.f, reverse=True)
            #Checks if the old g value is larger than the new
            elif (targetNode.g + mapObject.get_cell_value(s_prime.position)) < s_prime.g:
                attach_and_eval(s_prime, targetNode, mapObject)
                if s_prime in closed:
                    propagate_path_improvements(s_prime, mapObject)

def check_for_node_in_list(node, opened, closed):
    """
    :param node: Node to be checked
    :param opened: Checks if node is in the opened list 
    :param closed: Checks if node is in the closed list 
    :return: If already in one of the lists, the node is replaced by the node in the list. If not, it returns node. 
    """
    for value in opened:
        # Checks on position as duplicate positions = duplicate node
        if value.position == node.position:
            return value
    for value in closed:
        if value.position == node.position:
            return value
    return node

def attach_and_eval(child, parent, mapObject):
    """
    Updates the cost values of a node
    :param child: The child in focus
    :param parent: The parent of the child
    :param map_obj: Map object to be considered
    """
    child.parent = parent
    child.g = parent.g + mapObject.get_cell_value(child.position)
    child.h = manhattan_distance(child.position, mapObject.get_end_goal_pos())
    child.f = child.g + child.h

def propagate_path_improvements(node, mapObject):
    """
    Updates path improvements
    :param node: Node to be considered
    :param mapObject: Map object to be considered
    """
    for child in node.children: 
        if node.g + mapObject.get_cell_value(child.position) < child.g:
            child.parent = node
            child.g = node.g + mapObject.get_cell_value(child.position)
            child.f = child.g + child.h
            propagate_path_improvements(child, mapObject)

def backtrack(node, mapObject):
    """
    Backtracks the nodes from end to start by tracing the parents of each node. Changes the path color to yellow.
    :param Node: Node to be considered
    :param map_obj: Map object to be considered
    """
    x = node
    while x.parent is not None:
        #Want to avoid coloring the blue end node
        if (x.position != mapObject.get_goal_pos()):
            mapObject.set_cell_value(x.position, 5) # Set to 5 to get yellow coloring of path
        x = x.parent
    mapObject.show_map()

def main(taskNumber):
    """
    Main function to run the program. Returns an image of the map from the chosen task, with the shortest path colored in yellow. 
    :param taskNumber: The task number to be executed
    """
    mapObject = Map.Map_Obj(task=taskNumber)
    executedTask = best_first_search(mapObject)

    try:
        backtrack(executedTask, mapObject)
    except AttributeError:
        print("failed")