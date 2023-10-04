#include <iostream>
#include <vector>
#include <queue>
#include <limits>

using namespace std;

const int INF = numeric_limits<int>::max();

// Structure to represent an edge
struct Edge {
    int to;
    int weight;
};

// Function to add a directed edge to the graph
void addEdge(vector<vector<Edge>>& graph, int from, int to, int weight) {
    graph[from].push_back({to, weight});
}

// Bellman-Ford algorithm to find the minimum distance from a source vertex to all other vertices
bool bellmanFord(vector<vector<Edge>>& graph, vector<int>& distance, int source) {
    int numVertices = graph.size();
    distance.assign(numVertices, INF);
    distance[source] = 0;

    for (int i = 0; i < numVertices - 1; ++i) {
        for (int u = 0; u < numVertices; ++u) {
            for (const Edge& edge : graph[u]) {
                int v = edge.to;
                int weight = edge.weight;
                if (distance[u] != INF && distance[u] + weight < distance[v]) {
                    distance[v] = distance[u] + weight;
                }
            }
        }
    }

    // Check for negative weight cycles
    for (int u = 0; u < numVertices; ++u) {
        for (const Edge& edge : graph[u]) {
            int v = edge.to;
            int weight = edge.weight;
            if (distance[u] != INF && distance[u] + weight < distance[v]) {
                return false; // Negative weight cycle found
            }
        }
    }

    return true;
}

// Dijkstra's algorithm to find the shortest paths from a source vertex to all other vertices
vector<int> dijkstra(vector<vector<Edge>>& graph, int source) {
    int numVertices = graph.size();
    vector<int> distance(numVertices, INF);
    distance[source] = 0;

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, source});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        for (const Edge& edge : graph[u]) {
            int v = edge.to;
            int weight = edge.weight;
            if (distance[u] != INF && distance[u] + weight < distance[v]) {
                distance[v] = distance[u] + weight;
                pq.push({distance[v], v});
            }
        }
    }

    return distance;
}

// Johnson's algorithm to find all pairs shortest paths
vector<vector<int>> johnsonsAlgorithm(vector<vector<Edge>>& graph) {
    int numVertices = graph.size();

    // Create a new vertex and connect it to all existing vertices with weight 0
    for (int i = 0; i < numVertices; ++i) {
        addEdge(graph, numVertices, i, 0);
    }

    // Step 1: Run Bellman-Ford algorithm from the new vertex
    vector<int> h(numVertices + 1, 0);
    if (!bellmanFord(graph, h, numVertices)) {
        cout << "Negative weight cycle detected. Algorithm cannot continue." << endl;
        return {};
    }

    // Step 2: Reweight the edges
    for (int u = 0; u < numVertices; ++u) {
        for (Edge& edge : graph[u]) {
            edge.weight += h[u] - h[edge.to];
        }
    }

    // Step 3: Run Dijkstra's algorithm from each vertex
    vector<vector<int>> distances(numVertices, vector<int>(numVertices, INF));
    for (int u = 0; u < numVertices; ++u) {
        vector<int> dist = dijkstra(graph, u);
        for (int v = 0; v < numVertices; ++v) {
            distances[u][v] = dist[v] - h[u] + h[v];
        }
    }

    return distances;
}

int main() {
    int numVertices, numEdges;
    cout << "Enter the number of vertices and edges: ";
    cin >> numVertices >> numEdges;

    vector<vector<Edge>> graph(numVertices);

    cout << "Enter edges in the format (from, to, weight):" << endl;
    for (int i = 0; i < numEdges; ++i) {
        int from, to, weight;
        cin >> from >> to >> weight;
        addEdge(graph, from, to, weight);
    }

    vector<vector<int>> shortestPaths = johnsonsAlgorithm(graph);

    if (!shortestPaths.empty()) {
        cout << "Shortest paths between all pairs of vertices:" << endl;
        for (int u = 0; u < numVertices; ++u) {
            for (int v = 0; v < numVertices; ++v) {
                if (shortestPaths[u][v] == INF) {
                    cout << "INF ";
                } else {
                    cout << shortestPaths[u][v] << " ";
                }
            }
            cout << endl;
        }
    }

    return 0;
}
