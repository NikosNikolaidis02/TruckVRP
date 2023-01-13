from VRP_Model import *
import random
from SolutionDrawer import *


class Solution:
    def __init__(self):
        self.cost = 0.0
        self.routes = []


# Class for Clark & Wright based construction algorithm.
class Saving:
    def __init__(self, n1, n2, sav):
        self.n1 = n1
        self.n2 = n2
        self.score = sav


class RelocationMove(object):
    def __init__(self):
        self.originRoutePosition = None
        self.targetRoutePosition = None
        self.originNodePosition = None
        self.targetNodePosition = None
        self.costChangeOriginRt = None
        self.costChangeTargetRt = None
        self.moveCost = None

    def Initialize(self):
        self.originRoutePosition = None
        self.targetRoutePosition = None
        self.originNodePosition = None
        self.targetNodePosition = None
        self.costChangeOriginRt = None
        self.costChangeTargetRt = None
        self.moveCost = 10 ** 9


class SwapMove(object):

    def __init__(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.costChangeFirstRt = None
        self.costChangeSecondRt = None
        self.moveCost = None

    def Initialize(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.costChangeFirstRt = None
        self.costChangeSecondRt = None
        self.moveCost = 10 ** 9


class TwoOptMove(object):
    def __init__(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.moveCost = None

    def Initialize(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.moveCost = 10 ** 9


class CustomerInsertion(object):
    def __init__(self):
        self.customer = None
        self.route = None
        self.cost = 10 ** 9


class CustomerInsertionAllPositions(object):
    def __init__(self):
        self.customer = None
        self.route = None
        self.insertionPosition = None
        self.cost = 10 ** 9


class Solver:
    def __init__(self, m):
        self.allNodes = m.allNodes
        self.customers = m.customers
        self.depot = m.allNodes[0]
        self.distanceMatrix = m.matrix
        self.capacity = m.capacity
        self.vehicle_number = m.vehicle_number
        self.list_load = [[]]
        self.sol = None
        self.bestSolution = None
        self.minTabuTenure = 50
        self.maxTabuTenure = 60
        self.tabuTenure = 20

    def solve(self):
        self.SetRoutedFlagToFalseForAllCustomers()
        # self.MinimumInsertions()
        self.Clarke_n_Wright()
        self.calculate_initial_demands()
        #self.read_solution()
        # print("Solution of Clark & Wright:")
        self.ReportSolution(self.sol)
        # print("Entering TabuSearch")
        self.TabuSearch(0)
        # print("Solution After Search:")
        self.ReportSolution(self.sol)
        print('Total Cost is:', self.CalculateTotalCost(self.sol))

        return self.sol

    def SetRoutedFlagToFalseForAllCustomers(self):
        for i in range(0, len(self.customers)):
            self.customers[i].isRouted = False

    # =============================== Methods for Clark & Wright based construction algorithm ==============================
    def Clarke_n_Wright(self):

        self.sol = self.create_initial_routes()
        savings: list = self.calculate_savings()
        savings.sort(key=lambda s: s.score, reverse=True)
        for i in range(0, len(savings)):
            sav = savings[i]
            n1 = sav.n1
            n2 = sav.n2
            rt1 = n1.route
            rt2 = n2.route

            if n1.route == n2.route:
                continue
            if self.not_first_or_last(rt1, n1) or self.not_first_or_last(rt2, n2):
                continue
            if rt1.load + rt2.load > self.capacity:
                continue

            self.merge_routes(n1, n2)

            # cst -= sav.score
            self.sol.cost = self.CalculateTotalCost(self.sol)

            if len(self.sol.routes) == self.vehicle_number:
                break

        routes = self.sol.routes[::]
        routeNumber = len(routes)
        while self.UnevenRoutes(routes):  # might be endless loop.
            routes.sort(key=lambda r: len(r.sequenceOfNodes), reverse=True)
            for i in range(routeNumber - 1, -1, -1):
                relocatedNode = routes[0].sequenceOfNodes[len(routes[0].sequenceOfNodes) - 1]  # Selecting the last node
                # from the route with the longest sequence of nodes.
                # potentialRoutes = list()
                potentialRoute: Route = routes[i]  # The route that the relocatedNode will potentially be appended.
                if potentialRoute.load + relocatedNode.demand > self.capacity:
                    continue
                potentialRoute.sequenceOfNodes.append(routes[0].sequenceOfNodes.pop())
                potentialRoute.load += relocatedNode.demand
                routes[0].load -= relocatedNode.demand
                break

        for route in self.sol.routes:
            route.cost, route.load = self.calculate_route_details(route.sequenceOfNodes)
        self.sol.cost = self.CalculateTotalCost(self.sol)

        # print(cst, self.sol.cost)

    def calculate_savings(self):
        savings = []
        for i in range(0, len(self.customers)):
            n1 = self.customers[i]
            for j in range(i + 1, len(self.customers)):
                n2 = self.customers[j]

                score = self.distanceMatrix[self.depot.ID][n2.ID] - self.distanceMatrix[n1.ID][n2.ID]

                sav = Saving(n1, n2, score)
                savings.append(sav)

        return savings

    def create_initial_routes(self):
        s = Solution()
        for i in range(0, len(self.customers)):
            n = self.customers[i]
            rt = Route(self.depot, self.capacity)
            n.route = rt
            n.position_in_route = 1
            rt.sequenceOfNodes.insert(1, n)
            rt.load = n.demand
            rt.cost = self.distanceMatrix[self.depot.ID][n.ID]
            s.routes.append(rt)
            s.cost += rt.cost
        return s

    def not_first_or_last(self, rt, n):
        if n.position_in_route != 1 and n.position_in_route != len(rt.sequenceOfNodes) - 1:  # -1 because there
            return True  # is no depot in the end
        return False

    def merge_routes(self, n1, n2):
        rt1 = n1.route
        rt2 = n2.route

        if n1.position_in_route == 1 and n2.position_in_route == len(rt2.sequenceOfNodes) - 1:
            rt1.sequenceOfNodes[1:1] = rt2.sequenceOfNodes[1:len(rt2.sequenceOfNodes)]
        elif n1.position_in_route == 1 and n2.position_in_route == 1:
            rt1.sequenceOfNodes[1:1] = rt2.sequenceOfNodes[len(rt2.sequenceOfNodes) - 1:0:-1]
        elif n1.position_in_route == len(rt1.sequenceOfNodes) - 1 and n2.position_in_route == 1:
            rt1.sequenceOfNodes[len(rt1.sequenceOfNodes):len(rt1.sequenceOfNodes)] = rt2.sequenceOfNodes[1:len(
                rt2.sequenceOfNodes)]
        elif n1.position_in_route == len(rt1.sequenceOfNodes) - 1 and n2.position_in_route == len(
                rt2.sequenceOfNodes) - 1:
            rt1.sequenceOfNodes[len(rt1.sequenceOfNodes):len(rt1.sequenceOfNodes)] = rt2.sequenceOfNodes[
                                                                                     len(rt2.sequenceOfNodes) - 1:0:-1]
        rt1.load += rt2.load
        self.sol.routes.remove(rt2)
        self.update_route_customers(rt1)

    def update_route_customers(self, rt):
        for i in range(1, len(rt.sequenceOfNodes)):
            n = rt.sequenceOfNodes[i]
            n.route = rt
            n.position_in_route = i

    def UnevenRoutes(self, routes):
        meanNodes = int(len(self.customers) / self.vehicle_number + 1)
        for route in routes:
            routeLength = len(route.sequenceOfNodes)
            if not (routeLength == meanNodes or routeLength == meanNodes + 1):
                return True

# =============================== Methods for Minimum Insertions based construction algorithm ==========================

    def Always_keep_an_empty_route(self):
        if len(self.sol.routes) == 0:
            rt = Route(self.depot, self.capacity)
            self.sol.routes.append(rt)
        else:
            rt = self.sol.routes[-1]
            if len(rt.sequenceOfNodes) > 2:
                rt = Route(self.depot, self.capacity)
                self.sol.routes.append(rt)

    def IdentifyMinimumCostInsertion(self, best_insertion):
        for i in range(0, len(self.customers)):
            candidateCust: Node = self.customers[i]
            if candidateCust.isRouted is False:
                for rt in self.sol.routes:
                    if rt.load + candidateCust.demand <= rt.capacity:
                        for j in range(0, len(rt.sequenceOfNodes) - 1):
                            A = rt.sequenceOfNodes[j]
                            B = rt.sequenceOfNodes[j + 1]
                            costAdded = self.distanceMatrix[A.ID][candidateCust.ID] +\
                                self.distanceMatrix[candidateCust.ID][B.ID]
                            costRemoved = self.distanceMatrix[A.ID][B.ID]
                            trialCost = costAdded - costRemoved
                            if trialCost < best_insertion.cost:
                                best_insertion.customer = candidateCust
                                best_insertion.route = rt
                                best_insertion.insertionPosition = j
                                best_insertion.cost = trialCost
                    else:
                        continue

    def MinimumInsertions(self):
        model_is_feasible = True
        self.sol = Solution()
        insertions = 0

        while insertions < len(self.customers):
            best_insertion = CustomerInsertionAllPositions()
            self.Always_keep_an_empty_route()
            self.IdentifyMinimumCostInsertion(best_insertion)

            if best_insertion.customer is not None:
                self.ApplyCustomerInsertionAllPositions(best_insertion)
                insertions += 1
            else:
                print('FeasibilityIssue')
                model_is_feasible = False
                break

        if model_is_feasible:
            self.TestSolution()

    def ApplyCustomerInsertionAllPositions(self, insertion):
        insCustomer = insertion.customer
        rt = insertion.route
        # before the second depot occurrence
        insIndex = insertion.insertionPosition
        rt.sequenceOfNodes.insert(insIndex + 1, insCustomer)
        rt.cost += insertion.cost
        self.sol.cost += insertion.cost
        rt.load += insCustomer.demand
        insCustomer.isRouted = True

    def IdentifyBestInsertionAllPositions(self, bestInsertion, rt):
        for i in range(0, len(self.customers)):
            candidateCust: Node = self.customers[i]
            if candidateCust.isRouted is False:
                if rt.load + candidateCust.demand <= rt.capacity:
                    lastNodePresentInTheRoute = rt.sequenceOfNodes[-2]
                    for j in range(0, len(rt.sequenceOfNodes) - 1):
                        A = rt.sequenceOfNodes[j]
                        B = rt.sequenceOfNodes[j + 1]
                        costAdded = self.distanceMatrix[A.ID][candidateCust.ID] + self.distanceMatrix[candidateCust.ID][
                            B.ID]
                        costRemoved = self.distanceMatrix[A.ID][B.ID]
                        trialCost = costAdded - costRemoved

                        if trialCost < bestInsertion.cost:
                            bestInsertion.customer = candidateCust
                            bestInsertion.route = rt
                            bestInsertion.cost = trialCost
                            bestInsertion.insertionPosition = j

# ===============================Methods that are used only in construction algorithms end here.=======================

    def read_solution(self):
        f = open("../example_solution.txt", "r")
        readfile = f.read().splitlines()
        number_of_routes = int(readfile[3])
        solution = [list() for _ in range(0, number_of_routes)]
        for lineIndex in range(4, len(readfile)):
            line = readfile[lineIndex].split(",")
            for number_char in line:
                solution[lineIndex - 4].append(int(number_char))

        self.sol = Solution()
        for route in solution:
            r: Route = Route(self.allNodes[0], self.capacity)
            for nodeID in route:
                n: Node = self.findNode(nodeID)
                if n.ID != 0:
                    r.sequenceOfNodes.append(n)
            routeCost, routeLoad = self.calculate_route_details(r.sequenceOfNodes)
            r.cost = routeCost
            r.load = routeLoad
            self.sol.routes.append(r)
        self.sol.cost = self.CalculateTotalCost(self.sol)

    def calculate_route_details(self, nodes_sequence):
        rt_load = 0
        rt_cumulative_cost = 0
        tot_time = 0
        for i in range(len(nodes_sequence) - 1):
            from_node = nodes_sequence[i]
            to_node = nodes_sequence[i + 1]
            tot_time += self.distanceMatrix[from_node.ID][to_node.ID]
            rt_cumulative_cost += tot_time
            tot_time += to_node.unloading_time
            rt_load += from_node.demand
        return rt_cumulative_cost, rt_load

    def findNode(self, id):
        allNodes = self.allNodes
        for n in allNodes:
            n: Node
            if id == n.ID:
                return n

    def TabuSearch(self, operator):
        solution_cost_trajectory = []
        random.seed(1)
        self.bestSolution = self.cloneSolution(self.sol)
        terminationCondition = False
        localSearchIterator = 0

        rm = RelocationMove()
        sm = SwapMove()
        top: TwoOptMove = TwoOptMove()

        # SolDrawer.draw(0, self.sol, self.allNodes)

        while terminationCondition is False:
            # operator = random.randint(0, 1)
            operator = 0

            rm.Initialize()
            sm.Initialize()
            top.Initialize()

            # Relocations
            if operator == 0:
                self.FindBestRelocationMove(rm, localSearchIterator)
                if rm.originRoutePosition is not None:
                    self.ApplyRelocationMove(rm, localSearchIterator)
            # Swaps
            elif operator == 1:
                self.FindBestSwapMove(sm, localSearchIterator)
                if sm.positionOfFirstRoute is not None:
                    self.ApplySwapMove(sm, localSearchIterator)
            elif operator == 2:
                self.FindBestTwoOptMove(top, localSearchIterator)
                if top.positionOfFirstRoute is not None:
                    self.ApplyTwoOptMove(top, localSearchIterator)

            # self.ReportSolution(self.sol)
            self.TestSolution()
            # solution_cost_trajectory.append(self.sol.cost)

            if self.sol.cost < self.bestSolution.cost:
                self.bestSolution = self.cloneSolution(self.sol)

            # SolDrawer.draw(localSearchIterator, self.sol, self.allNodes)

            localSearchIterator = localSearchIterator + 1

            # Το κάνω 30 για να τρέχει πιο γρήγορα στον έλεγχο
            if localSearchIterator > 10:
                terminationCondition = True

        # SolDrawer.draw('final_ts', self.bestSolution, self.allNodes)
        # SolDrawer.drawTrajectory(solution_cost_trajectory)

        self.sol = self.bestSolution

    def cloneSolution(self, sol: Solution):
        cloned = Solution()
        for i in range(0, len(sol.routes)):
            rt = sol.routes[i]
            clonedRoute = self.cloneRoute(rt)
            cloned.routes.append(clonedRoute)
        cloned.cost = self.sol.cost
        return cloned

    def cloneRoute(self, rt: Route):
        cloned = Route(self.depot, self.capacity)
        cloned.cost = rt.cost
        cloned.load = rt.load
        cloned.sequenceOfNodes = rt.sequenceOfNodes.copy()
        return cloned

    def TestSolution(self):
        totalSolCost = 0
        for r in range(0, len(self.sol.routes)):
            rt: Route = self.sol.routes[r]
            rt_load = 0
            rt_cumulative_cost = 0
            tot_time = 0
            for j in range(0, len(rt.sequenceOfNodes) - 1):
                from_node = rt.sequenceOfNodes[j]
                to_node = rt.sequenceOfNodes[j + 1]
                tot_time += self.distanceMatrix[from_node.ID][to_node.ID]
                rt_cumulative_cost += tot_time
                tot_time += to_node.unloading_time
                rt_load += from_node.demand
            if abs(rt_cumulative_cost - rt.cost) > 0.0001:
                print('Route Cost problem (first node of route:)', rt.sequenceOfNodes[1].ID)
            if rt_load != rt.load:
                print('Route Load problem')

            totalSolCost += rt.cost

        if abs(totalSolCost - self.sol.cost) > 0.0001:
            print('Solution Cost problem')
            print("Total Cost:", totalSolCost, "Solution Cost:", self.sol.cost)

    def FindBestRelocationMove(self, rm, iterator):
        for originRouteIndex in range(0, len(self.sol.routes)):
            rt1: Route = self.sol.routes[originRouteIndex]
            for targetRouteIndex in range(0, len(self.sol.routes)):
                rt2: Route = self.sol.routes[targetRouteIndex]
                for originNodeIndex in range(1, len(rt1.sequenceOfNodes) - 1):
                    for targetNodeIndex in range(0, len(rt2.sequenceOfNodes) - 1):

                        if originRouteIndex == targetRouteIndex and (
                                targetNodeIndex == originNodeIndex or targetNodeIndex == originNodeIndex - 1):
                            continue

                        A = rt1.sequenceOfNodes[originNodeIndex - 1]
                        B = rt1.sequenceOfNodes[originNodeIndex]
                        C = rt1.sequenceOfNodes[originNodeIndex + 1]

                        F = rt2.sequenceOfNodes[targetNodeIndex]
                        G = rt2.sequenceOfNodes[targetNodeIndex + 1]

                        if rt1 != rt2:
                            if rt2.load + B.demand > rt2.capacity:
                                break

                        rt1size = len(rt1.sequenceOfNodes)
                        rt2size = len(rt2.sequenceOfNodes)


                        costAdded = (rt1size - (originNodeIndex + 1)) * self.distanceMatrix[A.ID][C.ID] + \
                                    (rt2size - targetNodeIndex) * self.distanceMatrix[F.ID][B.ID] + \
                                    (rt2size - (targetNodeIndex + 1)) * self.distanceMatrix[B.ID][G.ID] +\
                                    (rt2size - targetNodeIndex) * rt2.sequenceOfNodes[targetNodeIndex + 1].unloading_time

                        costRemoved = (rt1size - originNodeIndex) * self.distanceMatrix[A.ID][B.ID] + \
                                      (rt1size - (originNodeIndex + 1)) * self.distanceMatrix[B.ID][C.ID] + \
                                      (rt2size - (targetNodeIndex + 1)) * self.distanceMatrix[F.ID][G.ID] + \
                                      (rt1size - originNodeIndex) * rt1.sequenceOfNodes[originNodeIndex].unloading_time

                        originRtCostChange = (rt1size - (originNodeIndex + 1)) * self.distanceMatrix[A.ID][C.ID] - \
                                             (rt1size - originNodeIndex) * self.distanceMatrix[A.ID][B.ID] - \
                                             (rt1size - (originNodeIndex + 1)) * self.distanceMatrix[B.ID][C.ID] - \
                                             (rt1size - originNodeIndex) * rt1.sequenceOfNodes[originNodeIndex].unloading_time

                        for i in range(0, originNodeIndex - 1):
                            originRtCostChange -= self.distanceMatrix[rt1.sequenceOfNodes[i].ID][
                                rt1.sequenceOfNodes[i + 1].ID]
                            originRtCostChange -= rt1.sequenceOfNodes[i+1].unloading_time
                            costRemoved += self.distanceMatrix[rt1.sequenceOfNodes[i].ID][rt1.sequenceOfNodes[i + 1].ID]
                            costRemoved += rt1.sequenceOfNodes[i+1].unloading_time

                        targetRtCostChange = (rt2size - targetNodeIndex) * self.distanceMatrix[F.ID][B.ID] + \
                                             (rt2size - (targetNodeIndex + 1)) * self.distanceMatrix[B.ID][G.ID] - \
                                             (rt2size - (targetNodeIndex + 1)) * self.distanceMatrix[F.ID][G.ID] + \
                                             (rt2size - targetNodeIndex) * rt2.sequenceOfNodes[targetNodeIndex + 1].unloading_time

                        for j in range(0, targetNodeIndex):
                            targetRtCostChange += self.distanceMatrix[rt2.sequenceOfNodes[j].ID][
                                rt2.sequenceOfNodes[j + 1].ID]
                            targetRtCostChange += rt2.sequenceOfNodes[j+1].unloading_time
                            costAdded += self.distanceMatrix[rt2.sequenceOfNodes[j].ID][rt2.sequenceOfNodes[j + 1].ID]
                            costAdded += rt2.sequenceOfNodes[j+1].unloading_time

                        moveCost = costAdded - costRemoved

                        if self.MoveIsTabu(B, iterator, moveCost):
                            continue

                        if moveCost < rm.moveCost:
                            self.StoreBestRelocationMove(originRouteIndex, targetRouteIndex, originNodeIndex,
                                                         targetNodeIndex, moveCost, originRtCostChange,
                                                         targetRtCostChange, rm)

    def FindBestSwapMove(self, sm, iterator):
        for firstRouteIndex in range(0, len(self.sol.routes)):
            rt1: Route = self.sol.routes[firstRouteIndex]
            for secondRouteIndex in range(firstRouteIndex, len(self.sol.routes)):
                rt2: Route = self.sol.routes[secondRouteIndex]
                for firstNodeIndex in range(1, len(rt1.sequenceOfNodes) - 1):
                    startOfSecondNodeIndex = 1
                    if rt1 == rt2:
                        startOfSecondNodeIndex = firstNodeIndex + 1
                    for secondNodeIndex in range(startOfSecondNodeIndex, len(rt2.sequenceOfNodes) - 1):

                        a1 = rt1.sequenceOfNodes[firstNodeIndex - 1]
                        b1 = rt1.sequenceOfNodes[firstNodeIndex]
                        c1 = rt1.sequenceOfNodes[firstNodeIndex + 1]

                        a2 = rt2.sequenceOfNodes[secondNodeIndex - 1]
                        b2 = rt2.sequenceOfNodes[secondNodeIndex]
                        c2 = rt2.sequenceOfNodes[secondNodeIndex + 1]

                        rt1size = len(rt1.sequenceOfNodes)
                        rt2size = len(rt2.sequenceOfNodes)

                        '''
                        print('---------------')
                        print('rt1:')
                        for i in rt1.sequenceOfNodes:
                            print(i.ID)
                        print('rt1:', rt1size, 'a1:', firstNodeIndex - 1, 'b1:', firstNodeIndex, 'c1:',
                              firstNodeIndex + 1)
                        print('rt2:')
                        for j in rt2.sequenceOfNodes:
                            print(j.ID)
                        print('rt2:', rt1size, 'a2:', secondNodeIndex - 1, 'b2:', secondNodeIndex, 'c2:',
                              secondNodeIndex + 1)
                        print('---------------')
                        '''
                        moveCost = None
                        costChangeFirstRoute = None
                        costChangeSecondRoute = None

                        if rt1 == rt2:
                            if firstNodeIndex == secondNodeIndex - 1:
                                costRemoved = (rt1size - firstNodeIndex) * self.distanceMatrix[a1.ID][b1.ID] + \
                                              (rt1size - secondNodeIndex) * self.distanceMatrix[b1.ID][b2.ID] + \
                                              (rt1size - (secondNodeIndex + 1)) * self.distanceMatrix[b2.ID][c2.ID]
                                costAdded = (rt1size - firstNodeIndex) * self.distanceMatrix[a1.ID][b2.ID] + \
                                            (rt1size - secondNodeIndex) * self.distanceMatrix[b2.ID][b1.ID] + \
                                            (rt1size - (secondNodeIndex + 1)) * self.distanceMatrix[b1.ID][c2.ID]
                                moveCost = costAdded - costRemoved
                            else:

                                costRemoved1 = (rt1size - firstNodeIndex) * self.distanceMatrix[a1.ID][b1.ID] + \
                                               (rt1size - (firstNodeIndex + 1)) * self.distanceMatrix[b1.ID][c1.ID]
                                costAdded1 = (rt1size - firstNodeIndex) * self.distanceMatrix[a1.ID][b2.ID] + \
                                             (rt1size - (firstNodeIndex + 1)) * self.distanceMatrix[b2.ID][c1.ID]
                                costRemoved2 = (rt1size - secondNodeIndex) * self.distanceMatrix[a2.ID][b2.ID] + \
                                               (rt1size - (secondNodeIndex + 1)) * self.distanceMatrix[b2.ID][c2.ID]
                                costAdded2 = (rt1size - secondNodeIndex) * self.distanceMatrix[a2.ID][b1.ID] + \
                                             (rt1size - (secondNodeIndex + 1)) * self.distanceMatrix[b1.ID][c2.ID]
                                moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)
                        else:
                            if rt1.load - b1.demand + b2.demand > self.capacity:
                                continue
                            if rt2.load - b2.demand + b1.demand > self.capacity:
                                continue

                            costRemoved1 = (rt1size - firstNodeIndex) * self.distanceMatrix[a1.ID][b1.ID] + \
                                           (rt1size - (firstNodeIndex + 1)) * self.distanceMatrix[b1.ID][c1.ID]
                            costAdded1 = (rt1size - firstNodeIndex) * self.distanceMatrix[a1.ID][b2.ID] + \
                                         (rt1size - (firstNodeIndex + 1)) * self.distanceMatrix[b2.ID][c1.ID]
                            costRemoved2 = (rt2size - secondNodeIndex) * self.distanceMatrix[a2.ID][b2.ID] + \
                                           (rt2size - (secondNodeIndex + 1)) * self.distanceMatrix[b2.ID][c2.ID]
                            costAdded2 = (rt2size - secondNodeIndex) * self.distanceMatrix[a2.ID][b1.ID] + \
                                         (rt2size - (secondNodeIndex + 1)) * self.distanceMatrix[b1.ID][c2.ID]

                            costChangeFirstRoute = costAdded1 - costRemoved1
                            costChangeSecondRoute = costAdded2 - costRemoved2

                            moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)

                        if self.MoveIsTabu(b1, iterator, moveCost) or self.MoveIsTabu(b2, iterator, moveCost):
                            continue

                        if moveCost < sm.moveCost:
                            self.StoreBestSwapMove(firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex,
                                                   moveCost, costChangeFirstRoute, costChangeSecondRoute, sm)

    def ApplyRelocationMove(self, rm: RelocationMove, iterator):
        oldCost = self.CalculateTotalCost(self.sol)

        originRt = self.sol.routes[rm.originRoutePosition]
        targetRt = self.sol.routes[rm.targetRoutePosition]

        B = originRt.sequenceOfNodes[rm.originNodePosition]

        if originRt == targetRt:
            del originRt.sequenceOfNodes[rm.originNodePosition]
            if rm.originNodePosition < rm.targetNodePosition:
                targetRt.sequenceOfNodes.insert(rm.targetNodePosition, B)
            else:
                targetRt.sequenceOfNodes.insert(rm.targetNodePosition + 1, B)
            originRt.cost += rm.moveCost
        else:
            self.calculate_demands(B, self.depot, rm.originRoutePosition, rm.targetRoutePosition)
            del originRt.sequenceOfNodes[rm.originNodePosition]
            targetRt.sequenceOfNodes.insert(rm.targetNodePosition + 1, B)
            originRt.cost += rm.costChangeOriginRt
            targetRt.cost += rm.costChangeTargetRt
            originRt.load -= B.demand
            targetRt.load += B.demand

        self.sol.cost += rm.moveCost

        newCost = self.CalculateTotalCost(self.sol)

        self.SetTabuIterator(B, iterator)
        # debuggingOnly
        temp = abs((newCost - oldCost) - rm.moveCost)
        if abs((newCost - oldCost) - rm.moveCost) > 0.0001:
            print('Cost Issue')

    def ApplySwapMove(self, sm, iterator):
        oldCost = self.CalculateTotalCost(self.sol)
        rt1 = self.sol.routes[sm.positionOfFirstRoute]
        rt2 = self.sol.routes[sm.positionOfSecondRoute]
        b1 = rt1.sequenceOfNodes[sm.positionOfFirstNode]
        b2 = rt2.sequenceOfNodes[sm.positionOfSecondNode]
        rt1.sequenceOfNodes[sm.positionOfFirstNode] = b2
        rt2.sequenceOfNodes[sm.positionOfSecondNode] = b1

        if rt1 == rt2:
            rt1.cost += sm.moveCost
        else:
            self.calculate_demands(b1, b2, sm.positionOfFirstRoute, sm.positionOfSecondRoute)
            rt1.cost += sm.costChangeFirstRt
            rt2.cost += sm.costChangeSecondRt
            rt1.load = rt1.load - b1.demand + b2.demand
            rt2.load = rt2.load + b1.demand - b2.demand

        self.sol.cost += sm.moveCost

        newCost = self.CalculateTotalCost(self.sol)

        self.SetTabuIterator(b1, iterator)
        self.SetTabuIterator(b2, iterator)
        # debuggingOnly
        if abs((newCost - oldCost) - sm.moveCost) > 0.0001:
            print('Cost Issue')

    def ReportSolution(self, sol):
        for i in range(0, len(sol.routes)):
            rt = sol.routes[i]
            for j in range(0, len(rt.sequenceOfNodes)):
                print(rt.sequenceOfNodes[j].ID, end=' ')
            print(rt.cost)
        print(self.sol.cost)

    def GetLastOpenRoute(self):
        if len(self.sol.routes) == 0:
            return None
        else:
            return self.sol.routes[-1]

    def IdentifyBestInsertion(self, bestInsertion, rt):
        for i in range(0, len(self.customers)):
            candidateCust: Node = self.customers[i]
            if candidateCust.isRouted is False:
                if rt.load + candidateCust.demand <= rt.capacity:
                    lastNodePresentInTheRoute = rt.sequenceOfNodes[-2]
                    trialCost = self.distanceMatrix[lastNodePresentInTheRoute.ID][candidateCust.ID]
                    if trialCost < bestInsertion.cost:
                        bestInsertion.customer = candidateCust
                        bestInsertion.route = rt
                        bestInsertion.cost = trialCost

    def ApplyCustomerInsertion(self, insertion):
        insCustomer = insertion.customer
        rt = insertion.route
        # before the second depot occurrence
        insIndex = len(rt.sequenceOfNodes) - 1
        rt.sequenceOfNodes.insert(insIndex, insCustomer)

        beforeInserted = rt.sequenceOfNodes[-3]

        costAdded = self.distanceMatrix[beforeInserted.ID][insCustomer.ID] + self.distanceMatrix[insCustomer.ID][
            self.depot.ID]
        costRemoved = self.distanceMatrix[beforeInserted.ID][self.depot.ID]

        rt.cost += costAdded - costRemoved
        self.sol.cost += costAdded - costRemoved

        rt.load += insCustomer.demand

        insCustomer.isRouted = True

    def StoreBestRelocationMove(self, originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost,
                                originRtCostChange, targetRtCostChange, rm: RelocationMove):
        rm.originRoutePosition = originRouteIndex
        rm.originNodePosition = originNodeIndex
        rm.targetRoutePosition = targetRouteIndex
        rm.targetNodePosition = targetNodeIndex
        rm.costChangeOriginRt = originRtCostChange
        rm.costChangeTargetRt = targetRtCostChange
        rm.moveCost = moveCost

    def StoreBestSwapMove(self, firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, moveCost,
                          costChangeFirstRoute, costChangeSecondRoute, sm):
        sm.positionOfFirstRoute = firstRouteIndex
        sm.positionOfSecondRoute = secondRouteIndex
        sm.positionOfFirstNode = firstNodeIndex
        sm.positionOfSecondNode = secondNodeIndex
        sm.costChangeFirstRt = costChangeFirstRoute
        sm.costChangeSecondRt = costChangeSecondRoute
        sm.moveCost = moveCost

    """def CalculateTotalCost(self, sol):
        c = 0
        for i in range (0, len(sol.routes)):
            rt = sol.routes[i]
            for j in range (0, len(rt.sequenceOfNodes) - 1):
                a = rt.sequenceOfNodes[j]
                b = rt.sequenceOfNodes[j + 1]
                c += self.distanceMatrix[a.ID][b.ID]
        return c"""

    def CalculateTotalCost(self, sol: Solution):
        c = 0
        for i in range(0, len(sol.routes)):
            nodes_sequence = sol.routes[i].sequenceOfNodes
            rt_cumulative_cost = 0
            tot_time = 0
            for j in range(len(nodes_sequence) - 1):
                from_node = nodes_sequence[j]
                to_node = nodes_sequence[j + 1]
                tot_time += self.distanceMatrix[from_node.ID][to_node.ID]
                rt_cumulative_cost += tot_time
                tot_time += to_node.unloading_time
            c += rt_cumulative_cost
        return c

    def MoveIsTabu(self, n: Node, iterator, moveCost):
        if moveCost + self.sol.cost < self.bestSolution.cost - 0.001:
            return False
        if iterator < n.isTabuTillIterator:
            return True
        return False

    def SetTabuIterator(self, n: Node, iterator):
        # n.isTabuTillIterator = iterator + self.tabuTenure
        n.isTabuTillIterator = iterator + random.randint(self.minTabuTenure, self.maxTabuTenure)

    def FindBestTwoOptMove(self, top, iterator):
        for rtInd1 in range(0, len(self.sol.routes)):
            rt1: Route = self.sol.routes[rtInd1]
            for rtInd2 in range(rtInd1, len(self.sol.routes)):
                rt2: Route = self.sol.routes[rtInd2]
                for nodeInd1 in range(0, len(rt1.sequenceOfNodes) - 1):
                    start2 = 0
                    if rt1 == rt2:
                        start2 = nodeInd1 + 2

                    for nodeInd2 in range(start2, len(rt2.sequenceOfNodes) - 1):
                        moveCost = 10 ** 9

                        rt1size = len(rt1.sequenceOfNodes)
                        rt2size = len(rt2.sequenceOfNodes)
                        '''
                        print('---------------')
                        print('rt1:')
                        for i in rt1.sequenceOfNodes:
                            print(i.ID)
                        print('rt1:', rt1size, 'A:', nodeInd1, 'B:', nodeInd1 + 1)
                        print('rt2:')
                        for j in rt2.sequenceOfNodes:
                            print(j.ID)
                        print('rt2:', rt1size, 'K:', nodeInd2, 'b2:', nodeInd2 + 1)
                        print('---------------')
                        '''

                        A = rt1.sequenceOfNodes[nodeInd1]
                        B = rt1.sequenceOfNodes[nodeInd1 + 1]
                        K = rt2.sequenceOfNodes[nodeInd2]
                        L = rt2.sequenceOfNodes[nodeInd2 + 1]

                        if rt1 == rt2:
                            costAdded = (rt1size - (nodeInd1 + 1)) * self.distanceMatrix[A.ID][K.ID] + \
                                        (rt1size - (nodeInd2 + 1)) * self.distanceMatrix[B.ID][L.ID]

                            costRemoved = (rt1size - (nodeInd1 + 1)) * self.distanceMatrix[A.ID][B.ID] + \
                                          (rt1size - (nodeInd2 + 1)) * self.distanceMatrix[K.ID][L.ID]
                            j = 1
                            for i in range(nodeInd1 + 1, nodeInd2):
                                costRemoved += (rt1size - (i + 1)) * self.distanceMatrix[rt1.sequenceOfNodes[i].ID][
                                    rt1.sequenceOfNodes[i + 1].ID]
                                costAdded += (rt1size - (nodeInd2 + 1) + j) * \
                                    self.distanceMatrix[rt1.sequenceOfNodes[i].ID][rt1.sequenceOfNodes[i + 1].ID]
                                j += 1

                            moveCost = costAdded - costRemoved
                        else:
                            if nodeInd1 == 0 and nodeInd2 == 0:
                                continue

                            if self.CapacityIsViolated(rt1, nodeInd1, rt2, nodeInd2):
                                continue
                            costAdded = (rt1size - (nodeInd1 + 1)) * self.distanceMatrix[A.ID][L.ID] + \
                                        (rt2size - (nodeInd2 + 1)) * self.distanceMatrix[B.ID][K.ID]

                            costRemoved = (rt1size - (nodeInd1 + 1)) * self.distanceMatrix[A.ID][B.ID] + \
                                          (rt2size - (nodeInd2 + 1)) * self.distanceMatrix[K.ID][L.ID]

                            if (nodeInd1 + 2) < rt1size:
                                costAdded += (rt1size - (nodeInd1 + 2)) * self.distanceMatrix[L.ID][
                                    rt1.sequenceOfNodes[nodeInd1 + 2].ID]
                                costRemoved += (rt1size - (nodeInd1 + 2)) * self.distanceMatrix[B.ID][
                                    rt1.sequenceOfNodes[nodeInd1 + 2].ID]

                            if (nodeInd2 + 2) < rt2size:
                                costAdded += (rt2size - (nodeInd2 + 2)) * self.distanceMatrix[K.ID][
                                    rt2.sequenceOfNodes[nodeInd2 + 2].ID]
                                costRemoved += (rt2size - (nodeInd2 + 2)) * self.distanceMatrix[B.ID][
                                    rt2.sequenceOfNodes[nodeInd2 + 2].ID]

                            moveCost = costAdded - costRemoved

                        if self.MoveIsTabu(A, iterator, moveCost) or self.MoveIsTabu(K, iterator, moveCost):
                            continue

                        if moveCost < top.moveCost:
                            self.StoreBestTwoOptMove(rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, top)

    def calculate_initial_demands(self):
        for rtInd in range(0, len(self.sol.routes)):
            rt: Route = self.sol.routes[rtInd]
            for nodeInd in range(1, len(rt.sequenceOfNodes)):
                if nodeInd == 1:
                    self.list_load[rtInd].append(0)
                    self.list_load[rtInd].append(rt.sequenceOfNodes[nodeInd].demand)

                else:
                    self.list_load[rtInd].append(self.list_load[rtInd][nodeInd-2] + rt.sequenceOfNodes[nodeInd].demand)
            self.list_load.append([])

    def calculate_demands(self, origin_node, target_node, origin_route_pos, target_route_pos):
        originRt = self.sol.routes[origin_route_pos]
        targetRt = self.sol.routes[target_route_pos]
        print('node', target_node)
        print('index', targetRt.sequenceOfNodes.index(target_node))
        print('length', len(targetRt.sequenceOfNodes))
        for nodeInd in range(originRt.sequenceOfNodes.index(origin_node), len(originRt.sequenceOfNodes)):
            self.list_load[origin_route_pos][nodeInd] = self.list_load[origin_route_pos][nodeInd] + target_node.demand - origin_node.demand

        for nodeInd in range(targetRt.sequenceOfNodes.index(target_node), len(targetRt.sequenceOfNodes)):
            print(nodeInd)
            print('route_pos', target_route_pos)
            self.list_load[target_route_pos][nodeInd] = self.list_load[target_route_pos][nodeInd] - target_node.demand + origin_node.demand
            print(self.list_load)


    def CapacityIsViolated(self, rt1, nodeInd1, rt2, nodeInd2):
        rt1FirstSegmentLoad = self.list_load[rt1][nodeInd1]
        rt1SecondSegmentLoad = rt1.load - rt1FirstSegmentLoad
        rt2FirstSegmentLoad = self.list_load[rt2][nodeInd2]
        rt2SecondSegmentLoad = rt2.load - rt2FirstSegmentLoad

        if rt1FirstSegmentLoad + rt2SecondSegmentLoad > rt1.capacity:
            return True
        if rt2FirstSegmentLoad + rt1SecondSegmentLoad > rt2.capacity:
            return True

        return False


    def ApplyTwoOptMove(self, top, iterator):
        rt1: Route = self.sol.routes[top.positionOfFirstRoute]
        rt2: Route = self.sol.routes[top.positionOfSecondRoute]

        if rt1 == rt2:
            # reverses the nodes in the segment [positionOfFirstNode + 1,  top.positionOfSecondNode]
            reversedSegment = reversed(rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1])
            # lst = list(reversedSegment)
            # lst2 = list(reversedSegment)
            rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1] = reversedSegment

            self.SetTabuIterator(rt1.sequenceOfNodes[top.positionOfFirstNode], iterator)
            self.SetTabuIterator(rt1.sequenceOfNodes[top.positionOfSecondNode], iterator)

            rt1.cost += top.moveCost

        else:
            # slice with the nodes from position top.positionOfFirstNode + 1 onwards
            relocatedSegmentOfRt1 = rt1.sequenceOfNodes[top.positionOfFirstNode + 1:]

            # slice with the nodes from position top.positionOfFirstNode + 1 onwards
            relocatedSegmentOfRt2 = rt2.sequenceOfNodes[top.positionOfSecondNode + 1:]

            del rt1.sequenceOfNodes[top.positionOfFirstNode + 1:]
            del rt2.sequenceOfNodes[top.positionOfSecondNode + 1:]

            rt1.sequenceOfNodes.extend(relocatedSegmentOfRt2)
            rt2.sequenceOfNodes.extend(relocatedSegmentOfRt1)

            self.SetTabuIterator(rt1.sequenceOfNodes[top.positionOfFirstNode], iterator)
            self.SetTabuIterator(rt2.sequenceOfNodes[top.positionOfSecondNode], iterator)

            self.UpdateRouteCostAndLoad(rt1)
            self.UpdateRouteCostAndLoad(rt2)
            self.calculate_demands(self, rt1.sequenceOfNodes[top.positionOfFirstNode] + 1,
                                   rt2.sequenceOfNodes[top.positionOfSecondNode] + 1, top.positionOfFirstRoute,
                                   top.positionOfSecondRoute)

        self.sol.cost += top.moveCost

    def UpdateRouteCostAndLoad(self, rt: Route):
        tc = 0
        tl = 0
        for i in range(0, len(rt.sequenceOfNodes) - 1):
            A = rt.sequenceOfNodes[i]
            B = rt.sequenceOfNodes[i + 1]
            tc += self.distanceMatrix[A.ID][B.ID]
            tl += A.demand
        rt.load = tl
        rt.cost = tc

    def StoreBestTwoOptMove(self, rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, top):
        top.positionOfFirstRoute = rtInd1
        top.positionOfSecondRoute = rtInd2
        top.positionOfFirstNode = nodeInd1
        top.positionOfSecondNode = nodeInd2
        top.moveCost = moveCost
