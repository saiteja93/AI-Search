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
    start = problem.getStartState()
    # We use a stack here as, the adding and popping of nodes for DFS happens at one side of the datastructure (Top of the stack in this case)
    OpenL = util.Stack()
    # Pushing the startnode on the stack to start the problem, along with the action it took to reach that node, in the case of a rootnode, the list of actions is [].
    OpenL.push((start, []))
    # We maintain a closed list, which is the list of nodes explored
    ClosedL= []
    while not OpenL.isEmpty():
        node, action = OpenL.pop()
        # we'll check if the node is itself a goal State or not
        if problem.isGoalState(node):
            return action
        #If its not a Goal State, we have to check if the node has already been explored, if no, then explore it
        if node not in ClosedL:
            #appending it to the closed list
            ClosedL.append(node)
            successor_List = problem.getSuccessors(node)
            for i in range(len(successor_List)):
                #If the child is already in the closedL means, its already been explored.
                if successor_List[i][0] not in ClosedL:
                    #If its not, we update the action and push it back in the stack
                    OpenL.push((successor_List[i][0], action + [successor_List[i][1]]))




def breadthFirstSearch(problem):
    start = problem.getStartState()
    # We use a Queue here as, the adding and popping of nodes for DFS happens at opposite ends of the datastructure, by the property of the BFS
    OpenL = util.Queue()
    # Pushing the startnode on the Queue to start the problem, along with the action it took to reach that node, in the case of a rootnode, the list of actions is [].
    OpenL.push((start, []))
    # We maintain a closed list, which is the list of nodes explored
    ClosedL= []
    while not OpenL.isEmpty():
        node, action = OpenL.pop()
        # we'll check if the node is itself the goal State or not
        if problem.isGoalState(node):
            return action
        #If its not a Goal State, we have to check if the node has already been explored, if no, then explore it.
        if node not in ClosedL:
            ClosedL.append(node)
            successor_List = problem.getSuccessors(node)
            for i in range(len(successor_List)):
                #If the child is already in the closedL means, its already been explored.
                if successor_List[i][0] not in ClosedL:
                    #If its not, we update the action and push it back in the stack
                    OpenL.push((successor_List[i][0], action + [successor_List[i][1]]))


def uniformCostSearch(problem):
    start = problem.getStartState()
    # We use a Priority Queue here as, the adding and popping of nodes is based on the cost. From the Priority Queue, you pop the node with highest priority, that is the node which has the least amount of cost.
    OpenL = util.PriorityQueue()
    # Pushing the startnode on the Priority Queue to start the problem, along with the action it took to reach that node, in the case of a rootnode, the list of actions is [] and also the g value in getting to that node, for the case of the root node, we are assuming to be 0.
    OpenL.push((start, []), 0)
    # We maintain a closed list, which is the list of nodes explored
    ClosedL = []
    #For the while condition
    while not OpenL.isEmpty():
        node, action = OpenL.pop()
        # we'll check if the node is itself the goal State or not
        if problem.isGoalState(node):
            return action
        # If its not a Goal State, we have to check if the node has already been explored, if no, then explore it.
        if node not in ClosedL:
            ClosedL.append(node)
            successor_List = problem.getSuccessors(node)
            for i in range(len(successor_List)):
                # If the child is already in the closedL means, its already been explored.
                if successor_List[i][0] not in ClosedL:
                    # If its not, we update the action, compute the g value for getting to that node and push it back in the stack
                    cost = problem.getCostOfActions(action + [successor_List[i][1]])
                    OpenL.push((successor_List[i][0], action + [successor_List[i][1]]), cost)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    start = problem.getStartState()
    # We use a Priority Queue here as, the adding and popping of nodes is based on the cost (f value). From the Priority Queue, you pop the node with highest priority, that is the node which has the least amount of f value.
    OpenL = util.PriorityQueue()
    # Pushing the startnode on the Priority Queue to start the problem, with the action it took to reach that node, in the case of a rootnode, the list of actions is [] and also the (g+h) value in getting to that node, for the case of the root node, we are assuming g=0 and h is given by nullHeuristic function.
    OpenL.push((start, []), heuristic(start, problem))
    # We maintain a closed list, which is the list of nodes explored
    ClosedL = []
    # For the while condition
    while not OpenL.isEmpty():
        node, action = OpenL.pop()
        # we'll check if the node is itself the goal State or not
        if problem.isGoalState(node):
            return action
        # If its not a Goal State, we have to check if the node has already been explored, if no, then explore the successors.
        if node not in ClosedL:
            ClosedL.append(node)
            successor_List = problem.getSuccessors(node)
            for i in range(len(successor_List)):
                # If the child is already in the closedL means, its already been explored.
                if successor_List[i][0] not in ClosedL:
                    # If its not, we update the action, compute the cost function ((g+h) value) and push it back in the stack
                    #print "heuristic", heuristic(successor_List[i][0], problem)
                    cost = problem.getCostOfActions(action + [successor_List[i][1]]) + heuristic(successor_List[i][0], problem)
                    OpenL.push((successor_List[i][0], action + [successor_List[i][1]]), cost)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
