# Graph Algorithm Application

This is a C++ program that implements graph algorithms to find the shortest and longest paths in a graph. It reads input data from a file containing information about vertices, edges, and their weights, then computes and outputs the shortest and longest paths between specified vertices.

## Features

- **Graph Representation**: The program represents the graph using adjacency matrix and adjacency list data structures.
- **Dijkstra's Algorithm**: It uses Dijkstra's algorithm to find the shortest path between two vertices.
- **Depth-First Search (DFS)**: It utilizes DFS to find the longest path between two vertices.
- **Input File Handling**: The program reads input data from a file, including the total number of vertices, edges, vertex coordinates, edge connections, and weights.
- **Output**: It displays the total vertices and edges, the starting and ending vertices, the Euclidean distance between start and end vertices, the shortest path, its length, and the maximum path length with its corresponding path.

## How to Use

1. **Compile**: Compile the program using a C++ compiler. For example, using g++:

```bash
g++ main.cpp -o graph_app

1. Run: Execute the compiled binary file and provide the input file as an argument:
./graph_app input.txt
Replace input.txt with the name of your input file.


Path finder using dijkstra algorithm to find the shortest path, creating a graph
and using Depth first Search to brute force find the longest path using adjacent lists.
