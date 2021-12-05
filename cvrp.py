#!/usr/bin/env python3.7
"""
    Constrained Vehicle Routing Problem

"""
import sys
import json
import os
import math
import random
from itertools import combinations
import gurobipy as gp
from gurobipy import GRB

def renumber(tuples):
    numbers = []
    for t in tuples:
        numbers.append(t[0])
        numbers.append(t[1])
    numbers = list(set(numbers))
    numbers = sorted(numbers)
    renum, renum2 = dict(), dict()
    k = 0
    for n in numbers:
        renum[k] = n  # sorted keys
        renum2[n] = k # original keys
        k += 1
    new_tuples = [(renum2[x], renum2[y]) for x,y in tuples]
    return renum, renum2, new_tuples

# cardinality of a route - excluding Depot
def get_route_nodes(route):
    current_nodes = []
    for i, j in route:
        if i != 0:
            current_nodes.append(i)
        if j != 0:
            current_nodes.append(j)
    return list(set(current_nodes))

# Callback - use lazy constraints to eliminate sub-tours
def subtourelim(model, where):
    if where == GRB.Callback.MIPSOL:
        # make a list of edges selected in the solution
        vals = model.cbGetSolution(model._vars)
        for t in range(nbTrucks):
            # print(t, nbTrucks)
            tuples = gp.tuplelist((i, j) for i, j, k in model._vars.keys()
                                    if vals[i, j, k] > 0.5 and k == t)
            d1, _, newtuples = renumber(tuples)
            selected =gp.tuplelist(newtuples)
            n = len(d1)
            # find the shortest cycle in the selected edge list
            # print('Routes: ', tuples)
            tour = subtour(selected, n)
            # print(tour, len(tour), n)
            if len(tour) < n:
                new_tour = [d1[x] for x in tour]  # map back to original node number
                # print('Cycles:', new_tour,'\n')
                # add subtour elimination constr. for every pair of cities in tour
                for t in range(nbTrucks):
                    if len(new_tour) == 2:
                        model.cbLazy(gp.quicksum(model._vars[i, j, t] + model._vars[j, i, t] for i, j in combinations(new_tour, 2)) <= len(tour)-1)
                    else:
                        model.cbLazy(gp.quicksum(model._vars[i, j, t] for i, j in combinations(new_tour, 2)) <= len(tour)-1)


# Given a tuplelist of edges, find the shortest subtour
def subtour(edges, n):
    unvisited = list(range(n))
    cycle = range(n+1)  # initial length has 1 more city
    while unvisited:  # true if list is non-empty
        thiscycle = []
        neighbors = unvisited
        while neighbors:
            current = neighbors[0]
            thiscycle.append(current)
            unvisited.remove(current)
            neighbors = [j for i, j in edges.select(current, '*')
                         if j in unvisited]
        # assuming 0 will be there for all vehicles and 0 means Depot
        if len(cycle) > len(thiscycle) and 0 not in thiscycle:
            cycle = thiscycle
    return cycle

problem = 'ORToolsCVRP_16_0_v2'
path_base = "/home/AzureUser/notebooks/agci-logistics/Routing/NewRoutingCore/"
path_to_distance = os.path.join(path_base,"data/Distances/Distances." + problem + ".json")
path_to_travel_time = os.path.join (path_base,"data/TravelTimes/TravelTimes." + problem + ".json")
path_to_fleet = os.path.join (path_base,"data/Fleet/Fleet." + problem + ".json")
path_to_requests = os.path.join (path_base,"data/Requests/Requests." + problem + ".json")

# Read the distance file
with open(path_to_distance) as fp:
    A = json.load(fp)
    distList = A['values']

with open(path_to_travel_time) as fp:
    tdata = json.load(fp)

with open(path_to_fleet) as fp:
    A = json.load(fp)
    vmodel = A['models']

nbTrucks = 0
for model in vmodel:
    nbTrucks += model['number']
capacity = model['capacity']

with open(path_to_requests) as fp:
    A = json.load(fp)
    request = list(A)

# customer demands
Q = [r['quantity'] for r in request]
Q.insert(0, 0)
print(Q)

n = len(distList)
dist = {(i, j): distList[i][j] for i in range(n) for j in range(n)}
print(f"Total {n-1} customers, {nbTrucks} trucks with {capacity} capacity")

arcs, Dist = gp.multidict(dist)  # arcs - tuplelist, Dist = tupledict
# print(Dist)
# sys.exit()

m = gp.Model()

#VARIABLES
"""
    In a more complex version, you can specify arbitrary lists of immutable objects, and this 
    method will create variables for each member of the cross product of these lists. 

    In the following, X = arcs X # Trucks
"""
# X=m.addVars(arcs, [k for k in range(nbTrucks)], vtype=GRB.BINARY, name="routes")

# Create variables
X = gp.tupledict()
for k in range(nbTrucks):
    for i,j in dist.keys():
        X[i,j,k] = m.addVar(vtype=GRB.BINARY, name='e[%d,%d,%d]'%(i,j,k))  # obj=dist[i,j], 

# print(X.keys())
# set the objective function
totaldist = gp.quicksum(X[i,j,k]*Dist[i,j] for i,j in arcs for k in range(nbTrucks))
m.setObjective(totaldist, GRB.MINIMIZE)
# m.setObjectiveN(totaldist, GRB.MINIMIZE, 0)

# total Trucks used
trucksUsed = {k: gp.quicksum(X[i,j,k] for i,j in arcs if i!=0 and j!=0) for k in range(nbTrucks)}
nbTrucksUsed = gp.quicksum(trucksUsed)
# m.setObjectiveN(nbTrucksUsed, GRB.MINIMIZE, 1)

# Add the constraints
# no more than one vehicle for each arc
# for i in range(1, n):
#     for j in range(1, n):
#         if i != j:
#             m.addConstr(sum(X[i,j,k] for k in range(nbTrucks)) == 1)

# each customer is visited exactly once
AllNodesVisited={i: gp.quicksum(X[j,i,k] for j in range(n) if i!=j for k in range(nbTrucks)) for i in range(1,n)} # (2.2)
m.addConstrs(AllNodesVisited[i]==1 for i in range(1,n))

TruckCapacity={k: gp.quicksum(Q[i]*X[i,j,k] for i,j in arcs) for k in range(nbTrucks)} #(2.3)
# capacity constraints
# for k in range(nbTrucks):
#     for i in range(1, n):
#         m.addConstr(sum(X[i,j,k]*Q[i] for j in range(1,n)) <= capacity)
m.addConstrs(TruckCapacity[k]<=capacity for k in range(nbTrucks)) #(2.3)

InFlow = {(j,k): gp.quicksum(X[i,j,k] for i in range(n) if i!=j) for j in range(n) for k in range(nbTrucks)}
OutFlow = {(j,k): gp.quicksum(X[j,i,k] for i in range(n)) for j in range(n) for k in range(nbTrucks)}
m.addConstrs(InFlow[j,k]==OutFlow[j,k] for j in range(n) for k in range(nbTrucks)) #(4)

# InFlow={(h,k):gp.quicksum(X[i,h,k] for i in range(n) if h!=i) for h in range(1,n) for k in range(nbTrucks)} #(2.5)
# OutFlow={(h,k):gp.quicksum(X[h,j,k] for j in range(n) if h!=j) for h in range(1,n) for k in range(nbTrucks)} #(2.5)
# m.addConstrs(InFlow[h,k]==OutFlow[h,k] for h in range(1,n) for k in range(nbTrucks)) #(2.5)
# m.addConstrs(X[i,p,k]==X[p,j,k] for i in range(n) for j in range(n) if j != i for p in range(1,n) if p != i and p != j for k in range(nbTrucks)) #(2.5)

DepositOut={k:gp.quicksum(X[0,j,k] for j in range(1,n)) for k in range(nbTrucks)} #(2.4)
m.addConstrs(DepositOut[k]==1 for k in range(nbTrucks)) #(2.4)

DepositIn={k:gp.quicksum(X[i,0,k] for i in range(1,n)) for k in range(nbTrucks)} #(2.6)
m.addConstrs(DepositIn[k]==1 for k in range(nbTrucks)) #(2.6)

# Sub-tour elimination constraint
# cycle = {k: gp.quicksum(X[i,j,k] for i in range(1,n) for j in range(1,n) if i!=j) for k in range(nbTrucks)}
# m.addConstrs(cycle[k])

# Optimize model
m._vars = X
m.Params.lazyConstraints = 1
m.optimize(subtourelim)

# Ensure status is optimal
assert m.Status == GRB.Status.OPTIMAL
m.printAttr('X')

nSolutions  = m.SolCount
nObjectives = m.NumObj
print('Problem has', nObjectives, 'objectives')
print('Gurobi found', nSolutions, 'solutions')

# solutions = []
# for s in range(nSolutions):
#   # Set which solution we will query from now on
#   m.params.SolutionNumber = s

#   # Print objective value of this solution in each objective
#   print('Solution', s, ':', end='')
#   for o in range(nObjectives):
#     # Set which objective we will query
#     m.params.ObjNumber = o
#     # Query the o-th objective value
#     # print(' ',m.ObjNVal, end='')

vals = m.getAttr('X', X)
# selected_routes = list()
# for t in range(nbTrucks):
#     tuples = gp.tuplelist((i, j) for i, j, k in vals.keys() if vals[i, j, k] > 0.5 and k == t)
#     d1, d2, newtuples = renumber(tuples)
#     n = len(d1)
#     selected =gp.tuplelist(newtuples)
#     tour = subtour(selected, n)
#     new_tour = [d1[x] for x in tour]  # map back to original node number
#     print(new_tour)
#     assert len(tour) == n

optimal_routes = [[] for _ in range(nbTrucks)]
distance_traversed = [0 for _ in range(nbTrucks)]
for tup in vals:
    if vals[tup] > 0:
        i, j, k = tup
        optimal_routes[k].append((i,j))
        distance_traversed[k] += Dist[(i,j)]
        # print(tup, vals[tup], Dist[(i,j)])
for v in range(nbTrucks):
    print(v)
    print(optimal_routes[v])
    print(distance_traversed[v])
print("Total distance traversed:", sum(distance_traversed))
