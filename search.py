# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]




def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    # initialize the already visited nodes
    nodes_Already_Visited = []
    path_Points = []
    # print(problem.getStartState())
    # util.raiseNotDefined()
    # for DFS firstly initialize stack
    dfs_Stack = util.Stack()
    #putting the first element from stack
    dfs_Stack.push([(problem.getStartState(), "Stop", 0)])
    #Checking the conditions till last element
    while dfs_Stack.isEmpty() == False:
        #fetching from the stack
        future_traced_path = dfs_Stack.pop()
        # print(future_traced_path);
        current_pac_location = future_traced_path[-1][0]
        for path_point in future_traced_path:
            path_Points.append(path_point)
         #if goal state reached
        if problem.isGoalState(current_pac_location):
            temp_list=[path_point_one[1] for path_point_one in future_traced_path][1:]
        # as per configuration of project method should return list
            return temp_list

        # check the visited states:
        if current_pac_location not in nodes_Already_Visited:
            nodes_Already_Visited.append(current_pac_location)
        #putting unvisited nodes
            for next_move in problem.getSuccessors(current_pac_location):
                if next_move[0] not in nodes_Already_Visited:
                    parent_node_path = future_traced_path[:]
                    parent_node_path.append(next_move)
                    dfs_Stack.push(parent_node_path)
     #output must be list for all these search statments
    return []


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    #util.raiseNotDefined()
    nodes_Already_Visited = []
    path_Points = []
    # print(problem.getStartState())
    # util.raiseNotDefined()
    # for BFS firstly initialize stack
    bfs_Queue = util.Queue()
    #Initiating the first values
    bfs_Queue.push([(problem.getStartState(), "Stop", 0)])
    #cheking the positions till queue is empty
    while bfs_Queue.isEmpty() == False:
        #fetching the element from queue
        future_traced_path = bfs_Queue.pop()
        # print(future_traced_path);
        current_pac_location = future_traced_path[-1][0]
        for path_point in future_traced_path:
            path_Points.append(path_point)
        #if we reach the final position
        if problem.isGoalState(current_pac_location):
            temp_list= [path_point_one[1] for path_point_one in future_traced_path][1:]
            #output must be list for all these search statments
            return temp_list
        # check the visited states:
        if current_pac_location not in nodes_Already_Visited:
            nodes_Already_Visited.append(current_pac_location)
            for next_move in problem.getSuccessors(current_pac_location):
                if next_move[0] not in nodes_Already_Visited:
                    parent_node_path = future_traced_path[:]
                    parent_node_path.append(next_move)
                    bfs_Queue.push(parent_node_path)
    # output must be list for all these search statments
    return []



def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    #Using lambda function to traverse
    traversal_val = lambda travel_direction: problem.getCostOfActions([cost_points[1] for cost_points in travel_direction][1:])

    #util.raiseNotDefined()
    # util.raiseNotDefined()
    nodes_Already_Visited = []
    path_Points = []
    # print(problem.getStartState())
    # util.raiseNotDefined()
    # for uniform cost search firstly initialize priority queue with function
    uni_cost_find = util.PriorityQueueWithFunction(traversal_val)
    #Initiate the start location
    uni_cost_find.push([(problem.getStartState(), "Stop", 0)])
    #moving till last
    while uni_cost_find.isEmpty() == False:
        future_traced_path = uni_cost_find.pop()
        # print(future_traced_path);
        current_pac_location = future_traced_path[-1][0]
        for path_point in future_traced_path:
            path_Points.append(path_point)
        if problem.isGoalState(current_pac_location):
            temp_list= [path_point_one[1] for path_point_one in future_traced_path][1:]
            #as per configuration return state must be list
            return temp_list
        # check the visited states:
        if current_pac_location not in nodes_Already_Visited:
            #appending visited nodes
            nodes_Already_Visited.append(current_pac_location)
            for next_move in problem.getSuccessors(current_pac_location):
                if next_move[0] not in nodes_Already_Visited:
                    parent_node_path = future_traced_path[:]
                    parent_node_path.append(next_move)
                    #Puhing the values
                    uni_cost_find.push(parent_node_path)
    # as per configuration return state must be list
    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    # We need list to put all the visited nodes
    pac_node_traversed = {};
    #Take traversed cost as
    travel_movement_estimate = 0
    #Take one more list to push the actions already performed
    performed_actions = []
    #Firstly create the object of priority queue
    required_queue = util.PriorityQueue()
    #find out the start node in pacman game:
    pac_start_point= problem.getStartState()

    #Pushing the starting node in the priority queue
    required_queue.push((pac_start_point,performed_actions, travel_movement_estimate) ,travel_movement_estimate)

    #Check the priority queue is emplty or not
    while required_queue.isEmpty()==False:
        #Till the queue is not empty always pop out the elements from it till we reach goal state
        pop_out_next_location= required_queue.pop();
        #Check whether required popped out location is final or not
        if(problem.isGoalState(pop_out_next_location[0])==True):
            temp_list=pop_out_next_location[1]
            #output must be list as per configuration
            return temp_list
        elif(pop_out_next_location[0] not in pac_node_traversed):
            pac_node_traversed[pop_out_next_location[0]] = True
            for suucess_val,action_performed, travel_estimate in problem.getSuccessors(pop_out_next_location[0]):
                if (suucess_val and suucess_val not in performed_actions):
                    #add the values in priority queue
                    sec_val=pop_out_next_location[1] + [action_performed]
                    third_val=pop_out_next_location[2] + travel_estimate
                    total_cost_with_heuristic=pop_out_next_location[2] + travel_estimate+heuristic(suucess_val, problem)
                    #Pushing the values in queue by adding heuristic value
                    required_queue.push((suucess_val,sec_val,third_val),total_cost_with_heuristic)
    # output must be list as per configuration
    return []

             #if pacman is not reached


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
