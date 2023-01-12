#from TSP_Model import Model
from Solver import *
import os


def printSolutionForChecker(sol: Solution):
    relativePath = "../solution.txt"
    if os.path.exists(relativePath):
        os.remove(relativePath)

    f = open(relativePath, "a+")
    f.write("Cost:\n")
    f.write(str(sol.cost) + "\n")
    f.write("Routes:\n")
    f.write(str(len(sol.routes)) + "\n")

    route: Route
    node: Node
    for i in range(len(sol.routes)):
        route = sol.routes[i]
        for j in range(len(route.sequenceOfNodes)):
            node = route.sequenceOfNodes[j]
            ending = ","
            if j == len(route.sequenceOfNodes) - 1:
                ending = ""
            f.write(str(node.ID) + ending)
        f.write("\n")


m = Model()
m.BuildModel()
s = Solver(m)
sol = s.solve()
printSolutionForChecker(sol)