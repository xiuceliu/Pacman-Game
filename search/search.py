# coding=utf-8
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
from util import Stack
from util import Queue
from util import PriorityQueue
from game import Directions


class Node:
    """
    构造搜索树节点，由父节点的动作导出
    """
    def __init__(self, state, action=None, pathCost=0, parent=None):
        """
        :param state: 节点状态，坐标
        :param action: 动作
        :param pathCost: 代价
        :param parent:
        """
        self.state = state
        self.action = action
        self.pathCost = pathCost
        self.parent = parent

        self.depth = 0      # 深度
        self.length = 0     # 路径长度

    def __repr__(self):
        return "<Node %s>" % (self.state,)

    def path(self):
        """
        构造从根节点到当前节点的路径
        """
        x, result = self, [self]
        while x.parent:     # 没有到达根节点
            result.append(x.parent)
            x = x.parent
        return result

    def move(self):
        """
        构造移动序列
        """
        result = self.path()
        directions = []
        for each in result:
            directions.append(each.action)
        directions.reverse()
        return directions[1:]

    def expand(self, problem):
        """
        :param problem: 搜索问题
        :return: 从当前节点出发可达的其它结点的列表
        """
        result = []
        for (next, action, cost) in problem.getSuccessors(self.state):      # getSuccessors()方法返回从当前状态出发可达的下一状态，及需要的操作和代价组成的元组
            result.append(Node(next, action, cost, self))
        return result

    def expandForUsc(self, problem):
        """
        :param problem: 搜索问题
        :return: 一致费用算法中当前节点的扩展节点列表
        """
        result = []
        for (next, action, cost) in problem.getSuccessors(self.state):
            result.append(Node(next, action, self.pathCost + cost, self))
        return result

    def expandForAStar(self, problem, closed, fringe, gScore):
        """
        :param problem:
        :param closed:
        :param fringe:
        :param gScore:
        :param hScore:
        :param fScore:
        :param heuristic:
        :return:
        """
        result = []
        for (next, action, cost) in problem.getSuccessors(self.state):
            if next in closed:
                continue
            gScore[next] = self.pathCost + cost
            if not is_Inside_PriorityQueue(fringe, next):
                result.append(Node(next, action, gScore[next], self))
        return result


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
    return [s, s, w, s, w, w, s, w]


def graphSearch(problem, fringe):
    """
    通用搜索算法
    :param problem: 搜索问题
    :param fringe: Open表结构
    :return:
    """
    closed = {}     # 存入还未访问过的结点
    startState = problem.getStartState()
    fringe.push(Node(startState, Directions.STOP, 1, None))
    while fringe:
        node = fringe.pop()
        if problem.isGoalState(node.state):
            return node     # 目标结点
        if node.state not in closed:
            closed[node.state] = True
            nodes = node.expand(problem)
            for each in nodes:
                fringe.push(each)
    return None     # 无解


def treeSearchForUsc(problem, fringe):
    closed = {}
    startState = problem.getStartState()
    fringe.push(Node(startState, Directions.STOP, 1, None), 0)
    while fringe:
        node = fringe.pop()
        if problem.isGoalState(node.state):
            return node
        if node.state not in closed:
            closed[node.state] = True
            for each in node.expandForUsc(problem):
                fringe.push(each, each.pathCost)
    return None


def treeSearchForAStar(problem, fringe, heuristic):
    startState = problem.getStartState()
    closed = {}
    gScore = {}
    hScore = {}
    fScore = {}
    gScore[startState] = 0
    hScore[startState] = heuristic(startState, problem)
    fScore[startState] = hScore[startState]
    fringe.push(Node(startState, Directions.STOP, gScore[startState], None), fScore[startState])
    while fringe:
        node = fringe.pop()
        if problem.isGoalState(node.state):
            return node
        if node.state not in closed:
            closed[node.state] = True
            for each in node.expandForAStar(problem, closed, fringe, gScore):
                fringe.push(each, each.pathCost + heuristic(each.state, problem))
    return None


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
    node = graphSearch(problem, Stack())
    return node.move()
    # util.raiseNotDefined()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    node = graphSearch(problem, Queue())
    return node.move()
    # util.raiseNotDefined()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    node = treeSearchForUsc(problem, PriorityQueue())
    return node.move()
    # util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    node = treeSearchForAStar(problem, PriorityQueue(), heuristic)
    return node.move()
    # util.raiseNotDefined()


def is_Inside_PriorityQueue(fringe, aimState):
    for item in fringe.heap:
        if item[0] == aimState:
            return True
    return False


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
