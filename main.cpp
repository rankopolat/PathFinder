#include <iostream>
#include <fstream>
#include <cmath>

using namespace std;

struct Vertices{
    double x;
    double y;
    int ID;
};

struct Edges{
    int start;
    int end;
    double weight;
};

struct adjList{

    int vertex;
    double distance;
    adjList *next;

    adjList(int _v, double _w, adjList& i)
        :vertex(_v),distance(_w),next(&i) {};

};

class Graph{

private:

    //Main Global Variables
    int totalVertex;
    int totalEdge;
    double** graph;

    //Edges and Vertices Holders
    Edges *_edges;
    Vertices *_vertices;
    adjList *_adjacent[100]{};

    //Shortest path variables
    int *_spath;
    int sCounter;
    double sDistance;

    //Longest Path Variables
    int *_mpath;
    int lCounter;
    double mDistance;

    //Starting Vertex and Goal/Ending Vertex
    int startVertex;
    int endVertex;


public:

    //Create the lists/arrays than destruct at end of program
    Graph(int, int);
    ~Graph();

    void readFile(ifstream &input);
    double Euclidean(); //Calculate Euclidean distance with Vertex objects
    void output();

    //Functions Used to find the shortest Path
    void dijkstraAlgorithm();
    int Distance(const double distance[],const bool visited[]) const;

    //Functions used to find the largest Path
    void DepthFirstSearch();
    void DFS(int current, int goal, int index, double length, bool *visited, int *path);

};

Graph::Graph(int tv,int te)
        :totalVertex(tv), totalEdge(te),startVertex(0),endVertex(0),sCounter(0),sDistance(0),lCounter(0),mDistance(0){

    _edges = new Edges[te];
    _vertices = new Vertices[tv];
    _spath = new int[te];
    _mpath = new int[te];

    //Initialise graph totalVertices x totalVertices
    graph = new double*[totalVertex];
    for (int i = 0; i < totalVertex; i++) {
        graph[i] = new double[totalVertex];
    }

    //Initialise Adjacent list all to null
    for(int i = 0; i < te; i++){
        _adjacent[i] = nullptr;
    }

    //set values in graph to 9999 as default
    for(int i = 0; i < totalVertex; i++){
        for(int j = 0; j < totalVertex; j++){
            graph[i][j] = 9999;
        }
    }
}

//Destructor Clean up memory on completion
Graph::~Graph(){

    delete[] _edges;
    delete[] _vertices;
    delete[] _spath;
    delete[] _mpath;

    //Deallocate dynamic 2D Graph
    for (int i = 0; i < totalVertex; i++) {
        delete[] graph[i];
    }
    delete[] graph;

    //Deallocate Adjacent list working downwards
    for (int i = 0; i < totalEdge; i++) {
        adjList* current = _adjacent[i];
        while (current != nullptr) {
            adjList* next = current->next;
            delete current;
            current = next;
        }
    }
}


void Graph::readFile(ifstream &input){

    for(int i = 0; i < totalVertex; i++){
        //Set Vertex Record
        input >> _vertices[i].ID >> _vertices[i].x >> _vertices[i].y;
    }

    for(int i = 0; i < totalEdge; i++){

        //Set Edge Record
        input >> _edges[i].start >> _edges[i].end >> _edges[i].weight;

        //Setting Weight values on graph
        graph[_edges[i].start-1][_edges[i].end-1] = _edges[i].weight;

        //Setting Adjacent list values to find the longest path
        _adjacent[_edges[i].start] = new adjList(_edges[i].end,_edges[i].weight,*_adjacent[_edges[i].start]);

    }

    //Getting starting Vertex and Ending Vertex
    input >> startVertex >> endVertex;

    //Setting the Graph starting vectors distance to 0
    graph[startVertex-1][startVertex-1] = 0;

}

int Graph::Distance(const double distance[], const bool visited[]) const {

    double minDistance = 9999;
    int smallest;

    for(int vertex = 0; vertex < totalVertex; vertex++){
        if(!visited[vertex] && distance[vertex] < minDistance){
            minDistance = distance[vertex];
            smallest = vertex;
        }
    }

    return smallest;
}


double Graph::Euclidean() {

    return std::sqrt(pow(_vertices[startVertex-1].x - _vertices[endVertex-1].x, 2)
                             + pow(_vertices[startVertex-1].y - _vertices[endVertex-1].y,2));
}


void Graph::dijkstraAlgorithm() {

    double D[totalVertex]; //Distances per vertex array
    bool V[totalVertex]; // Visited array
    int P[totalVertex]; // Parent/Predecessor array
    int holder = endVertex-1;

    //Set default values for Arrays
    for(int i = 0; i < totalVertex; i++){
        if(i == startVertex-1)
            V[i] = true;
        else{
            D[i] = graph[startVertex-1][i];
            V[i] = false;
            P[i] = startVertex - 1;
        }
    }

    //Find the smallest vertex than connect to current vertex with the lowest weight
    for(int a = 0; a < totalVertex; a++) {
        int minVertex = Distance(D, V);
        V[minVertex] = true; //Set the lowest weight vertex as visited

        for(int v = 0; v < totalVertex; v++) {
            double distance = D[minVertex] + graph[minVertex][v];

            //Check if it hasn't been visited and the total distance is less than the Current
            if(!V[v] && distance < D[v]) {
                D[v] = D[minVertex] + graph[minVertex][v];
                P[v] = minVertex;
            }
        }
    }

    //Track backwards adding the predecessor to the path until it reaches the start
    while(holder != startVertex-1){
        _spath[sCounter] = holder + 1;
        holder = P[holder];
        sCounter++;
    }

    _spath[sCounter] = holder + 1;
    sCounter++;
    sDistance = D[endVertex-1];
}


void Graph::output() {

    cout << "Total Vertices: " << totalVertex << endl;
    cout << "Total Edges: " << totalEdge << endl;
    cout << "Starting Vertex: " << startVertex << endl;
    cout << "Ending Vertex: " << endVertex << endl;
    cout << "====================" << endl;
    cout << "The Euclidean distance between the start and the goal vertices is: " << Euclidean() << endl;
    cout << "The Shortest Path being: ";

    for(int i = sCounter-1; i >= 0; i--){
        if(i == 0)
            cout << _spath[i];
        else
            cout << _spath[i] << " -> ";
    }

    cout << endl << "Length of the shortest path: " << sDistance << endl;
    cout << "Maximum Path Length: " << mDistance << endl;
    cout << "The Longest path being: ";

    for(int i = 0; i < lCounter; i++){
        if(i == lCounter-1)
            cout << _mpath[i];
        else
            cout << _mpath[i] << " -> ";
    }
}


void Graph::DepthFirstSearch() {

    double length = 0;
    int index = 0;
    int *path = new int[totalEdge];
    bool visited[totalVertex];

    for (int i = 0; i < totalVertex; i++) {
        visited[i] = false; // Initialise beginning vertexes as not visited
    }

    DFS(startVertex, endVertex, index, length, visited, path); //Start DFS from starting vertex

}

void Graph::DFS(int current, int goal, int index, double length, bool visited[], int path[]) {

    visited[current] = true;
    path[index] = current;
    index++;

    //This will only run once being when the path loop back to itself
    if(current == goal){
        if(length > mDistance){
            mDistance = length;
            lCounter = index;
            for(int i = 0; i < index; i++)
                _mpath[i] = path[i];
        }
        visited[current] = false;
        return; //Quick return to avoid checking other paths once correct path is found
    }

    for (adjList *a = _adjacent[current]; a != nullptr; a = a->next) {
        if (!visited[a->vertex]) {
            DFS(a->vertex, goal, index, length + a->distance, visited, path);
        }
    }

    //Reset Visited to continue checking each vertex
    visited[current] = false;
}


int main() {

    //main variables of total edges and vertices
    int totalVertex, totalEdge;

    //Input file Checker return error code -1 if fail to open
    char file[30];
    cout << "Please Enter File Name: ";
    cin >> file;

    ifstream input(file);
    if(!input){
        cerr << "File Failed to open" << endl;
        return -1;
    }

    input >> totalVertex >> totalEdge;

    //Create a Graph Object from total edges and vertices
    Graph graph(totalVertex,totalEdge);
    graph.readFile(input);

    graph.dijkstraAlgorithm();//Find minimum Path
    graph.DepthFirstSearch();//Find Maximum Path

    graph.output();

    return 0;
}
