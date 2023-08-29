# Sample-based HLP For Kinova

## Start
 - Sample NUM_NODES nodes and save them
 - Run PrecomputeJointsPositions.m
 - Run ./BuildAdajacency
 - Compile by ./compile_collision_checker.sh

## Parameters
 - FastCollisionChecking.h NUM_NODES: number of nodes in total
 - collision_checker.cu NUM_EDGES: number of edges in total, should be equal to the number of lines in the result of BuildAdajacency
 - collision_checker.cu COLLISION_THRESHOLD: node within this range with respect to obstacles will be discarded
 - collision_checker.cu EDGE_THRESHOLD: edge larger than this will be discarded

