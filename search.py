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
    print problem.getStartState()
    print problem.getSuccessors(problem.getStartState())
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
    startState = problem.getStartState()
    actionList = depthFirstSearchVisit(problem, startState, [startState])
    return actionList[::-1]

def depthFirstSearchVisit(problem, targetState, pathList):
    for successor in problem.getSuccessors(targetState):
        nextState = successor[0]
        nextAction = successor[1]
        if nextState not in pathList:
            pathList.append(nextState)
            if not problem.isGoalState(nextState):           
                actionList = depthFirstSearchVisit(problem, nextState, pathList)
                if actionList != None:
                    actionList.append(nextAction)
                    return actionList               
            else:
                return [nextAction]
    """If no available path, give up this path"""           
    return None

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    path = []
    if hasattr(problem, 'finishSearch'):
        while len(problem.visitedCorners) < 4:
            path += breadthFirstSearchNormal(problem)
    else:
        path = breadthFirstSearchNormal(problem)
    return path

def breadthFirstSearchNormal(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue
    visitedList = {}
    startState = problem.getStartState()
    q = Queue()
    q.push(startState)
    visitedList[startState] = [None, None]
    while not q.isEmpty():
        targetState = q.pop()
        isGoal = problem.isGoalState(targetState)
        if isGoal:
                return buildActionListFromBFSResult(visitedList,startState, targetState)
        else:
            for successor in problem.getSuccessors(targetState):
                state = successor[0]
                action = successor[1]
                if state not in visitedList.keys():
                    visitedList[state] = [targetState, action]
                    q.push(state)
    
def buildActionListFromBFSResult(visitedList, startState, endState):
    actionList = []
    state = endState
    while state != startState:               
        nextAction = visitedList[state][1]
        state = visitedList[state][0]
        actionList.append(nextAction)
    return actionList[::-1]

def uniformCostSearch(problem):
    "*** YOUR CODE HERE ***"
    from util import PriorityQueueWithFunction
    visitedList = {}
    startState = problem.getStartState()
    q = PriorityQueueWithFunction(lambda a: a[1])#proposition to priority
    q.push((startState, 0))
    visitedList[startState] = [None, None, 0]
    while not q.isEmpty():
        targetStateAndCost = q.pop()
        print targetStateAndCost
        targetState = targetStateAndCost[0]
        targetPathCost = targetStateAndCost[1]
        if problem.isGoalState(targetState):
                return buildActionListFromBFSResult(visitedList,startState, targetState)
        else:
            for successor in problem.getSuccessors(targetState):
                state = successor[0]
                action = successor[1]
                weight = successor[2]
                cost =  targetPathCost + weight
                if state not in visitedList.keys()  or visitedList[state][2] > cost: #why?
                    visitedList[state] = [targetState, action, cost]
                    q.push((state, cost))


def uniformCostSearch2(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueueWithFunction
    visitedList = {}
    startState = problem.getStartState()
    q = PriorityQueueWithFunction(lambda a: -a.pathCost)
    q.push(StateWithPathCost(startState, 0))
    visitedList[startState] = [None, None]
    while not q.isEmpty():
        swp = q.pop()
        targetState = swp.state
        for successor in problem.getSuccessors(targetState):
            state = successor[0]
            action = successor[1]
            weight = successor[2]
            if state not in visitedList.keys():
                visitedList[state] = [targetState, action]
                q.push(StateWithPathCost(state, weight + swp.pathCost))
            if problem.isGoalState(state):
                return buildActionListFromBFSResult(visitedList,startState, state)

class StateWithPathCost:
    def __init__(self, state, pathCost):
        self.state = state
        self.pathCost = pathCost
    def __str__(self):
        return str(self.state) +": "+str(self.pathCost)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "*** YOUR CODE HERE ***"
    path = []
    if hasattr(problem, 'finishSearch'):
        while len(problem.visitedCorners) < 4:
            path += aStarSearchNormal(problem, heuristic)
            # from searchAgents import cornersHeuristic
            # for y in (range(5)):
            #     print cornersHeuristic((problem.maxAxis[1] -1, y+5), problem)
    else:
        path = aStarSearchNormal(problem, heuristic)

    print "Actual Cost", problem.getCostOfActions(path)


    return path


def aStarSearchNormal(problem, heuristic=nullHeuristic):
    from util import PriorityQueueWithFunction
    visitedList = {}
    startState = problem.getStartState()
    q = PriorityQueueWithFunction(lambda a: a.cost + a.heuristic)#proposition to priority
    q.push(aStarPriorityQueueMember(startState, None, None, 0, heuristic(startState, problem)))
    targetState = None
    while not q.isEmpty():
        targetStateAndCost = q.pop()
        targetState = targetStateAndCost.state
        targetPrevState = targetStateAndCost.prevState
        targetAction = targetStateAndCost.action
        targetPathCost = targetStateAndCost.cost
        if targetState not in visitedList.keys(): #why?
                visitedList[targetState] = [targetPrevState, targetAction, targetPathCost]
                if problem.isGoalState(targetState):
                    actionList = buildActionListFromAStarResult(visitedList,startState, targetState)
                    return actionList

                else:
                    for successor in problem.getSuccessors(targetState):
                        state = successor[0]
                        action = successor[1]
                        weight = successor[2]
                        cost =  targetPathCost + weight
                        q.push(aStarPriorityQueueMember(state, targetState, action, cost, heuristic(state, problem)))

class aStarPriorityQueueMember:
    def __init__(self, state, prevState, action, cost, heuristic):
        self.state = state
        self.prevState = prevState
        self.action = action
        self.cost = cost
        self.heuristic = heuristic

def buildActionListFromAStarResult(visitedList, startState, endState):
    actionList = []
    state = endState
    while state != startState:
        nextAction = visitedList[state][1]
        state = visitedList[state][0]
        actionList.append(nextAction)
    return actionList[::-1]

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
