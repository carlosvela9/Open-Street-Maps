// graph.h <Starter Code>
// Carlos Velasquez
//
// Basic graph class using adjacency matrix representation.  Currently
// limited to a graph with at most 100 vertices.
//
//
// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Fall 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//

/*-------------------------------------------
Program 5: Open Maps
Course: CS 251, Fall 2023, UIC
System: VSCode, Advanced ZyLabs
Author: Carlos Velasquez, starter code provided by Adam Koehler
------------------------------------------- */

#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>
#include <map>

using namespace std;

template<typename VertexT, typename WeightT>
class graph 
{
  private:
    map<VertexT, map<VertexT, WeightT>> adjacencyList;

  public:
    //Default constructor
    graph() {}

    //Destructor
    ~graph() {}

    // Copy constructor
    graph(const graph& other) : adjacencyList(other.adjacencyList) {}

    // Assignment operator
    graph& operator=(const graph& other) 
    {
      if (this != &other) 
      {
        adjacencyList = other.adjacencyList;
      }
      
      return *this;
    }

    //Returns number of vertices
    int NumVertices() const 
    {
      return adjacencyList.size();
    }

    //Returns number of edges
    int NumEdges() const 
    {
      int count = 0;

      for (const auto& vertexEdges : adjacencyList) 
      {
        count += vertexEdges.second.size();
      }

      return count;
    }

    //Checks if the given vertex already exists in the adjacency list.
    //If the vertex does not exist, it is added to the graph with an empty map of edges
    //and weights
    bool addVertex(VertexT v) 
    {
      //Check if the vertex already exists
      if (adjacencyList.find(v) != adjacencyList.end()) 
      {
        return false;
      }

      //Add the vertex with an empty map of edges and weights
      adjacencyList[v] = map<VertexT, WeightT>();

      return true;
    }

    //Checks if the given vertices exist in the adjacency list,
    //and if they do, it adds an edge with the specified weight from the source
    //vertex to the destination vertex
    bool addEdge(VertexT from, VertexT to, WeightT weight) 
    {
      //Check if the vertices exist
      auto itFrom = adjacencyList.find(from);
      auto itTo = adjacencyList.find(to);

      if (itFrom == adjacencyList.end() || itTo == adjacencyList.end()) 
      {
        return false;
      }

      //Add the edge with its weight
      itFrom->second[to] = weight;

      return true;
    }

    //Checks if the specified source vertex and destination vertex exist in
    //the graph. If both vertices exist and there is an edge between them, the function sets
    //the weight of the edge
    bool getWeight(VertexT from, VertexT to, WeightT& weight) const 
    {
      //Check if the vertices exist
      auto itFrom = adjacencyList.find(from);

      if (itFrom == adjacencyList.end()) 
      {
        return false;
      }

      //Check if the edge exists
      auto itTo = itFrom->second.find(to);

      if (itTo == itFrom->second.end()) 
      {
        return false;
      }

      //Return the weight
      weight = itTo->second;

      return true;
    }

    //Checks if the specified vertex exists in the graph. If the vertex exists,
    //it retrieves the set of neighbors (vertices connected by edges) and returns it. If the
    //vertex does not exist, an empty set is returned.
    set<VertexT> neighbors(VertexT v) const 
    {
      set<VertexT> S;

      //Check if the vertex exists
      auto found = adjacencyList.find(v);

      if (found != adjacencyList.end()) 
      {
          //Add neighbors to the set
          for (const auto& edge : found->second) 
          {
            S.insert(edge.first);
          }
      }

      return S;
    }

    //Iterates through the vertices stored in the adjacency list and
    //returns a vector containing all vertices present in the graph
    vector<VertexT> getVertices() const 
    {
      vector<VertexT> vertices;

      // Collect vertices from the map
      for (const auto& vertexEdges : adjacencyList) 
      {
        vertices.push_back(vertexEdges.first);
      }

      return vertices;
    }

    void dump(ostream& output) const 
    {
      output << "***************************************************" << endl;
      output << "********************* GRAPH ***********************" << endl;

      output << "**Num vertices: " << NumVertices() << endl;
      output << "**Num edges: " << NumEdges() << endl;

      output << endl;
      output << "**Vertices:" << endl;

      for (const auto& vertexEdges : adjacencyList) 
      {
          output << " " << vertexEdges.first << endl;
      }

      output << endl;
      output << "**Edges:" << endl;

      for (const auto& vertexEdges : adjacencyList) 
      {
          output << vertexEdges.first << ": ";

          for (const auto& edge : vertexEdges.second) 
          {
              output << "(" << edge.first << "," << edge.second << ") ";
          }

          output << endl;
      }
      
      output << "**************************************************" << endl;
    }
};