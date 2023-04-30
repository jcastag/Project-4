#ifndef GRAPH_H
#define GRAPH_H

#include <map>
#include <set>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

class Graph
{
public:
    struct Edge
    {
        int sID, eID;
        double weight; // pheremone
        Edge(int id1, int id2, double w)
        {
            this->sID = id1;
            this->eID = id2;
            this->weight = w;
        }
        Edge()
        {
            this->sID = 0; // start ID
            this->eID = 0; // end ID
            this->weight = 0;
        }
    };

    Graph(const std::string &filename);
    Graph(const std::string &filename, bool ant);
    const std::set<int> &getNeighbors(int node) const;
    const std::vector<int> &getNodes() const;
    std::vector<Edge> getEdges(int node);
    void setEdgeWeight(int fn, int tn, double v);
    double getEdgeWeight(int fn, int tn);

private:
    bool isAnt;
    std::map<int, std::set<int>> adjacency_list;
    std::map<int, std::vector<Edge>> edges_list;
    std::vector<int> nodes;

    void addEdges();
    void readFile(const std::string &filename);
};

#endif
