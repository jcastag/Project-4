#include "graph.h"

#include <iostream>

Graph::Graph(const std::string &filename)
{
    std::string fn = "graph/" + filename;
    readFile(fn);
}

Graph::Graph(const std::string &filename, bool ant)
{
    std::string fn = "graph/" + filename;
    if (ant)
        this->isAnt = true;
    readFile(fn);
    addEdges();
}

// values to add here
// starting edge value

void Graph::addEdges()
{
    double base = 1.5; // BASE STARTING PHEREMONE

    for (int i = 0; i < this->nodes.size(); i++)
    {
        std::set<int> neighbors = this->getNeighbors(i);
        std::set<int>::iterator itr;

        for (itr = neighbors.begin();
             itr != neighbors.end(); itr++)
        {
            Edge e;
            e.eID = i;
            e.sID = *itr;

            if (isAnt)
                e.weight = base;
            else
                e.weight = 0; // gives empty weight if using hot-potato

            this->edges_list[i].push_back(e);
        }
    }
}

std::vector<Graph::Edge> Graph::getEdges(int node)
{
    std::vector<Edge> nodeEdges;

    nodeEdges = this->edges_list[node];

    return nodeEdges;
}

void Graph::setEdgeWeight(int fromNode, int ToNode, double val)
{
    std::vector<Edge> edges = getEdges(fromNode);
    for (auto &e : edges)
    {
        if (e.sID == fromNode && e.sID == ToNode)
            e.weight = val;
    }
}

double Graph::getEdgeWeight(int fromNode, int ToNode)
{
    std::vector<Edge> edges = getEdges(fromNode);
    for (auto &e : edges)
    {
        if (e.sID == fromNode && e.sID == ToNode)
            return e.weight;
    }
    return 0;
}

const std::set<int> &Graph::getNeighbors(int node) const
{
    return adjacency_list.at(node);
}

const std::vector<int> &Graph::getNodes() const
{
    return nodes;
}

void Graph::readFile(const std::string &filename)
{
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        exit(1);
    }

    std::string line;
    while (std::getline(file, line))
    {
        if (line[0] == '#')
        {
            continue;
        }

        std::istringstream iss(line);
        int node, neighbor;
        char colon;

        iss >> node >> colon;

        nodes.push_back(node);

        while (iss >> neighbor)
        {
            adjacency_list[node].insert(neighbor);
        }
    }
    file.close();
}
