import random
import math

class Model:

# instance variables
    def __init__(self):
        self.allNodes = []
        self.customers = []
        self.matrix = []
        self.capacity = -1
        self.vehicle_number = -1

    """def BuildModel(self):
        random.seed(1)
        depot = Node(0, 50, 50, 0)
        self.allNodes.append(depot)

        self.capacity = 50
        totalCustomers = 120

        for i in range (0, totalCustomers):
            x = random.randint(0, 100)
            y = random.randint(0, 100)
            dem = random.randint(1, 4)
            cust = Node(i + 1, x, y, dem)
            self.allNodes.append(cust)
            self.customers.append(cust)

        rows = len(self.allNodes)
        self.matrix = [[0.0 for x in range(rows)] for y in range(rows)]

        for i in range(0, len(self.allNodes)):
            for j in range(0, len(self.allNodes)):
                a = self.allNodes[i]
                b = self.allNodes[j]
                dist = math.sqrt(math.pow(a.x - b.x, 2) + math.pow(a.y - b.y, 2))
                self.matrix[i][j] = dist"""

    def BuildModel(self):
        f = open("../Instance.txt", "r")
        readfile = f.read().splitlines()
        self.vehicle_number = int(readfile[0].split(",")[1])
        self.capacity = int(readfile[1].split(",")[1])
        number_of_customers = int(readfile[2].split(",")[1])

        for lineIndex in range(5, len(readfile)):
            line = readfile[lineIndex].split(",")
            id = int(line[0])
            x = int(line[1])
            y = int(line[2])
            d = int(line[3])
            unloading_time = int(line[4])
            n = Node(id, x, y, d, unloading_time)
            self.allNodes.append(n)

        self.customers = self.allNodes[1:]

        rows = len(self.allNodes)
        self.matrix = [[0.0 for x in range(rows)] for y in range(rows)]

        for i in range(0, len(self.allNodes)):
            for j in range(0, len(self.allNodes)):
                a: Node = self.allNodes[i]
                b: Node = self.allNodes[j]
                dist = math.sqrt(math.pow(a.x - b.x, 2) + math.pow(a.y - b.y, 2))
                self.matrix[i][j] = dist  # + b.unloading_time


class Node:

    def __init__(self, idd, xx, yy, dem, unloading_time):
        self.x = xx
        self.y = yy
        self.ID = idd
        self.demand = dem
        self.unloading_time = unloading_time
        self.isRouted = False
        self.isTabuTillIterator = -1

    def __str__(self) -> str:
        return str(self.ID) + " " + str(self.x) + " " + str(self.y) + " " +\
               str(self.demand) + " " + str(self.unloading_time)


class Route:
    def __init__(self, dp, cap):
        self.sequenceOfNodes = []
        self.sequenceOfNodes.append(dp)
        # self.sequenceOfNodes.append(dp)
        self.cost = 0
        self.capacity = cap
        self.load = 0
