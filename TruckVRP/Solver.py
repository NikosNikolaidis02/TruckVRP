from VRP_Model import *
import random
from SolutionDrawer import *


class Solution:
    def __init__(self):
        self.cost = 0.0
        self.routes = []


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
        self.MinimumInsertions()
        self.calculate_initial_demands()
        print('kkkkkkkkkkkk', self.list_load[13])
        print('rrrrrrrrrr', len(self.sol.routes[13].sequenceOfNodes))

        # self.read_solution()
        self.ReportSolution(self.sol)
        self.TabuSearch(0)
        self.ReportSolution(self.sol)
        print('Total Cost is:', self.CalculateTotalCost(self.sol))

        return self.sol

    def SetRoutedFlagToFalseForAllCustomers(self):
        for i in range(0, len(self.customers)):
            self.customers[i].isRouted = False

    # =============================== Methods for Minimum Insertions based construction algorithm ======================

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

                        for j in range(0, len(rt.sequenceOfNodes)):
                            A = rt.sequenceOfNodes[j]
                            if len(rt.sequenceOfNodes) == 1:  # Route: 0 X    (Route has only the depot)

                                trialCost = self.distanceMatrix[A.ID][candidateCust.ID]

                            elif (len(rt.sequenceOfNodes) - 1) == j:  # Route: 0 x x x X (Last Node)

                                trialCost = self.distanceMatrix[A.ID][candidateCust.ID]

                                for temp in range(0, j):
                                    trialCost += self.distanceMatrix[rt.sequenceOfNodes[temp].ID][
                                        rt.sequenceOfNodes[temp + 1].ID]
                                    trialCost += rt.sequenceOfNodes[temp + 1].unloading_time
                            else:

                                B = rt.sequenceOfNodes[j + 1]
                                rtsize = len(rt.sequenceOfNodes)

                                trialCost = (rtsize - j) * self.distanceMatrix[A.ID][candidateCust.ID] + \
                                            (rtsize - (j + 1)) * self.distanceMatrix[candidateCust.ID][B.ID] - \
                                            (rtsize - (j + 1)) * self.distanceMatrix[A.ID][B.ID] + \
                                            (rtsize - (j + 1)) * candidateCust.unloading_time

                                for temp in range(0, j):
                                    trialCost += self.distanceMatrix[rt.sequenceOfNodes[temp].ID][
                                        rt.sequenceOfNodes[temp + 1].ID]
                                    trialCost += rt.sequenceOfNodes[temp + 1].unloading_time

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

            if len(self.sol.routes) < 14:
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
            self.TestSolution(0)

    def ApplyCustomerInsertionAllPositions(self, insertion):
        insCustomer = insertion.customer
        rt = insertion.route
        # before the second depot occurrence
        insIndex = insertion.insertionPosition
        rt.sequenceOfNodes.insert(insIndex + 1, insCustomer)
        # rt.cost += insertion.cost

        oldCost = rt.cost
        rt.cost, rt.load = self.calculate_route_details(rt.sequenceOfNodes)
        newCost = rt.cost - oldCost
        self.sol.cost += newCost

        # rt.load += insCustomer.demand
        insCustomer.isRouted = True

    # ===============================Methods that are used only in construction algorithms end here.====================

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
        random.seed(3)
        self.bestSolution = self.cloneSolution(self.sol)
        terminationCondition = False
        localSearchIterator = 0

        rm = RelocationMove()
        sm = SwapMove()
        top: TwoOptMove = TwoOptMove()

        # SolDrawer.draw(0, self.sol, self.allNodes)

        while terminationCondition is False:
            operator = random.randint(0, 2)

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
            # TwoOpt
            elif operator == 2:
                self.FindBestTwoOptMove(top, localSearchIterator)
                if top.positionOfFirstRoute is not None:
                    self.ApplyTwoOptMove(top, localSearchIterator)

            # self.ReportSolution(self.sol)
            self.TestSolution(localSearchIterator)
            # solution_cost_trajectory.append(self.sol.cost)

            print(localSearchIterator, self.sol.cost, self.bestSolution.cost)

            if self.sol.cost < self.bestSolution.cost:
                self.bestSolution = self.cloneSolution(self.sol)

            # SolDrawer.draw(localSearchIterator, self.sol, self.allNodes)

            localSearchIterator = localSearchIterator + 1

            if localSearchIterator > 50:
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

    def TestSolution(self, iterator):
        """if iterator == whatever:
            print("This is the whatever th iteration")"""
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

                        if originRouteIndex == targetRouteIndex:
                            rt: Route = self.cloneRoute(rt1)
                            rt.sequenceOfNodes.remove(B)
                            if originNodeIndex > targetNodeIndex:
                                rt.sequenceOfNodes.insert(targetNodeIndex + 1, B)
                            else:
                                rt.sequenceOfNodes.insert(targetNodeIndex, B)
                            rt.cost = self.calculate_route_details(rt.sequenceOfNodes)[0]
                            if rt.cost > rt1.cost:
                                continue
                            originRtCostChange = 0  # it doesn't matter what you put here because
                            targetRtCostChange = 0  # the target and origin are the same (check ApplyRelocationMove).
                            costAdded = 0
                            costRemoved = rt1.cost - rt.cost

                        else:
                            costAdded = (rt1size - (originNodeIndex + 1)) * self.distanceMatrix[A.ID][C.ID] + \
                                        (rt2size - targetNodeIndex) * self.distanceMatrix[F.ID][B.ID] + \
                                        (rt2size - (targetNodeIndex + 1)) * self.distanceMatrix[B.ID][G.ID] + \
                                        (rt2size - (targetNodeIndex + 1)) * B.unloading_time

                            costRemoved = (rt1size - originNodeIndex) * self.distanceMatrix[A.ID][B.ID] + \
                                          (rt1size - (originNodeIndex + 1)) * self.distanceMatrix[B.ID][C.ID] + \
                                          (rt2size - (targetNodeIndex + 1)) * self.distanceMatrix[F.ID][G.ID] + \
                                          (rt1size - (originNodeIndex + 1)) * B.unloading_time

                            originRtCostChange = (rt1size - (originNodeIndex + 1)) * self.distanceMatrix[A.ID][C.ID] - \
                                                 (rt1size - originNodeIndex) * self.distanceMatrix[A.ID][B.ID] - \
                                                 (rt1size - (originNodeIndex + 1)) * self.distanceMatrix[B.ID][C.ID] - \
                                                 (rt1size - (originNodeIndex + 1)) * B.unloading_time

                            for i in range(0, originNodeIndex - 1):
                                originRtCostChange -= self.distanceMatrix[rt1.sequenceOfNodes[i].ID][
                                    rt1.sequenceOfNodes[i + 1].ID]
                                originRtCostChange -= rt1.sequenceOfNodes[i + 1].unloading_time
                                costRemoved += self.distanceMatrix[rt1.sequenceOfNodes[i].ID][
                                    rt1.sequenceOfNodes[i + 1].ID]
                                costRemoved += rt1.sequenceOfNodes[i + 1].unloading_time

                            targetRtCostChange = (rt2size - targetNodeIndex) * self.distanceMatrix[F.ID][B.ID] + \
                                                 (rt2size - (targetNodeIndex + 1)) * self.distanceMatrix[B.ID][G.ID] - \
                                                 (rt2size - (targetNodeIndex + 1)) * self.distanceMatrix[F.ID][G.ID] + \
                                                 (rt2size - (targetNodeIndex + 1)) * B.unloading_time

                            for j in range(0, targetNodeIndex):
                                targetRtCostChange += self.distanceMatrix[rt2.sequenceOfNodes[j].ID][
                                    rt2.sequenceOfNodes[j + 1].ID]
                                targetRtCostChange += rt2.sequenceOfNodes[j + 1].unloading_time
                                costAdded += self.distanceMatrix[rt2.sequenceOfNodes[j].ID][
                                    rt2.sequenceOfNodes[j + 1].ID]
                                costAdded += rt2.sequenceOfNodes[j + 1].unloading_time

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

        self.update_demands_from_rm(rm.originNodePosition, rm.targetNodePosition, rm.originRoutePosition,
                                    rm.targetRoutePosition)

        if originRt == targetRt:
            del originRt.sequenceOfNodes[rm.originNodePosition]
            if rm.originNodePosition < rm.targetNodePosition:
                targetRt.sequenceOfNodes.insert(rm.targetNodePosition, B)
            else:
                targetRt.sequenceOfNodes.insert(rm.targetNodePosition + 1, B)
            originRt.cost += rm.moveCost
        else:
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

        self.update_demands_from_sm(sm.positionOfFirstNode, sm.positionOfSecondNode, sm.positionOfFirstRoute,
                                    sm.positionOfSecondRoute)

        if rt1 == rt2:
            rt1.cost += sm.moveCost
        else:
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

                        A = rt1.sequenceOfNodes[nodeInd1]
                        B = rt1.sequenceOfNodes[nodeInd1 + 1]
                        K = rt2.sequenceOfNodes[nodeInd2]
                        L = rt2.sequenceOfNodes[nodeInd2 + 1]

                        if rt1 == rt2:
                            rt: Route = self.cloneRoute(rt1)  # Depot until A and L until end of route
                            reversedSegment = reversed(rt1.sequenceOfNodes[nodeInd1 + 1: nodeInd2 + 1])  # K until B
                            rt.sequenceOfNodes[nodeInd1 + 1: nodeInd2 + 1] = reversedSegment
                            rt.cost = self.calculate_route_details(rt.sequenceOfNodes)[0]
                            moveCost = rt.cost - rt1.cost
                        else:
                            if nodeInd1 == 0 and nodeInd2 == 0:
                                continue

                            if self.CapacityIsViolated(rtInd1, nodeInd1, rtInd2, nodeInd2):
                                continue

                            rt1Cloned = self.cloneRoute(rt1)
                            rt2Cloned = self.cloneRoute(rt2)
                            relocatedSegmentOfRt1 = rt1.sequenceOfNodes[nodeInd1 + 1:]
                            relocatedSegmentOfRt2 = rt2.sequenceOfNodes[nodeInd2 + 1:]

                            rt1Cloned.sequenceOfNodes[nodeInd1 + 1:] = relocatedSegmentOfRt2
                            rt2Cloned.sequenceOfNodes[nodeInd2 + 1:] = relocatedSegmentOfRt1

                            rt1Cloned.cost = self.calculate_route_details(rt1Cloned.sequenceOfNodes)[0]
                            rt2Cloned.cost = self.calculate_route_details(rt2Cloned.sequenceOfNodes)[0]
                            moveCost = rt1Cloned.cost + rt2Cloned.cost - rt1.cost - rt2.cost

                        if self.MoveIsTabu(A, iterator, moveCost) or self.MoveIsTabu(K, iterator, moveCost):
                            continue

                        if moveCost < top.moveCost:
                            self.StoreBestTwoOptMove(rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, top)

    def calculate_initial_demands(self):
        for rtInd in range(0, len(self.sol.routes)):
            rt: Route = self.sol.routes[rtInd]
            for nodeInd in range(1, len(rt.sequenceOfNodes)):
                if nodeInd == 1:
                    self.list_load[rtInd].append(self.depot.demand)
                    self.list_load[rtInd].append(rt.sequenceOfNodes[nodeInd].demand)
                else:
                    self.list_load[rtInd].append(self.list_load[rtInd][nodeInd - 1] + rt.sequenceOfNodes[nodeInd].demand)
            if rtInd != len(self.sol.routes) - 1:
                self.list_load.append([])

    def update_demands_from_rm(self, origin_node_pos, target_node_pos, origin_route_pos, target_route_pos):
        originRt = self.sol.routes[origin_route_pos]
        targetRt = self.sol.routes[target_route_pos]
        origin_node = originRt.sequenceOfNodes[origin_node_pos]

        del self.list_load[origin_route_pos][origin_node_pos]
        for nodeInd in range(origin_node_pos, len(originRt.sequenceOfNodes) - 1):
            self.list_load[origin_route_pos][nodeInd] = self.list_load[origin_route_pos][nodeInd] - origin_node.demand

        if originRt == targetRt:
            end_node = len(targetRt.sequenceOfNodes) - 1
        else:
            end_node = len(targetRt.sequenceOfNodes)

        for nodeInd in range(target_node_pos, end_node):
            if target_node_pos == 0:
                continue
            self.list_load[target_route_pos][nodeInd] = self.list_load[target_route_pos][nodeInd] + origin_node.demand

        if originRt == targetRt and origin_node_pos < target_node_pos and target_node_pos != 0:
            self.list_load[target_route_pos].insert(target_node_pos, origin_node.demand + self.list_load[target_route_pos][target_node_pos - 1])
        else:
            self.list_load[target_route_pos].insert(target_node_pos + 1, origin_node.demand
                                                    + self.list_load[target_route_pos][target_node_pos - 1])

    def update_demands_from_sm(self, origin_node_pos, target_node_pos, origin_route_pos, target_route_pos):
        originRt = self.sol.routes[origin_route_pos]
        targetRt = self.sol.routes[target_route_pos]
        origin_node = originRt.sequenceOfNodes[origin_node_pos]
        target_node = targetRt.sequenceOfNodes[target_node_pos]
        '''print('node', target_node.ID)
        print('index', targetRt.sequenceOfNodes.index(target_node))
        print('length', len(targetRt.sequenceOfNodes))'''
        for nodeInd in range(origin_node_pos, len(originRt.sequenceOfNodes)):
            self.list_load[origin_route_pos][nodeInd] = self.list_load[origin_route_pos][nodeInd] + target_node.demand \
                                                        - origin_node.demand

        for nodeInd in range(target_node_pos, len(targetRt.sequenceOfNodes)):
            print(nodeInd)
            print('route_pos', target_route_pos)
            self.list_load[target_route_pos][nodeInd] = self.list_load[target_route_pos][nodeInd] \
                                                            - target_node.demand + origin_node.demand
            print()

    def update_demands_from_top(self, origin_node_pos, target_node_pos, origin_route_pos, target_route_pos):
        originRt = self.sol.routes[origin_route_pos]
        targetRt = self.sol.routes[target_route_pos]
        origin_node = originRt.sequenceOfNodes[origin_node_pos]
        target_node = targetRt.sequenceOfNodes[target_node_pos]
        '''print('node', target_node.ID)
        print('index', targetRt.sequenceOfNodes.index(target_node))
        print('length', len(targetRt.sequenceOfNodes))'''
        for nodeInd in range(origin_node_pos, len(originRt.sequenceOfNodes)):
            if nodeInd == origin_node_pos:
                self.list_load[origin_route_pos][nodeInd] = self.list_load[target_route_pos][target_node_pos - 1] + origin_node.demand
            else:
                self.list_load[origin_route_pos][nodeInd] = self.list_load[origin_route_pos][nodeInd - 1] + originRt.sequenceOfNodes[nodeInd].demand

        for nodeInd in range(target_node_pos, len(targetRt.sequenceOfNodes)):
            '''print(nodeInd)
            print('route_pos', target_route_pos)'''
            if nodeInd == target_node_pos:
                self.list_load[target_route_pos][nodeInd] = self.list_load[origin_route_pos][origin_node_pos - 1] + target_node.demand
            else:
                self.list_load[target_route_pos][nodeInd] = self.list_load[target_route_pos][nodeInd - 1] + targetRt.sequenceOfNodes[nodeInd].demand

        if originRt == targetRt:
            K = targetRt.sequenceOfNodes[target_node_pos - 1]

            self.list_load[target_route_pos][origin_node_pos] = self.list_load[target_route_pos][
                                                            origin_node_pos - 1] + K.demand

    def CapacityIsViolated(self, rt1_index, nodeInd1, rt2_index, nodeInd2):
        rt1 = self.sol.routes[rt1_index]
        rt2 = self.sol.routes[rt2_index]

        rt1FirstSegmentLoad = self.list_load[rt1_index][nodeInd1]
        rt1SecondSegmentLoad = rt1.load - rt1FirstSegmentLoad
        rt2FirstSegmentLoad = self.list_load[rt2_index][nodeInd2]
        rt2SecondSegmentLoad = rt2.load - rt2FirstSegmentLoad

        if rt1FirstSegmentLoad + rt2SecondSegmentLoad > rt1.capacity:
            return True
        if rt2FirstSegmentLoad + rt1SecondSegmentLoad > rt2.capacity:
            return True

        return False

    def ApplyTwoOptMove(self, top, iterator):
        rt1: Route = self.sol.routes[top.positionOfFirstRoute]
        rt2: Route = self.sol.routes[top.positionOfSecondRoute]

        self.update_demands_from_top(top.positionOfFirstNode + 1, top.positionOfSecondNode + 1,
                                     top.positionOfFirstRoute, top.positionOfSecondRoute)

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

        self.sol.cost += top.moveCost

    def UpdateRouteCostAndLoad(self, rt: Route):
        rt.cost, rt.load = self.calculate_route_details(rt.sequenceOfNodes)

    def StoreBestTwoOptMove(self, rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, top):
        top.positionOfFirstRoute = rtInd1
        top.positionOfSecondRoute = rtInd2
        top.positionOfFirstNode = nodeInd1
        top.positionOfSecondNode = nodeInd2
        top.moveCost = moveCost
