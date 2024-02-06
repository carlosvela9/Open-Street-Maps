// application.cpp <Starter Code>
// Carlos Velasquez
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
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

/*-------------------------------------------
Program 5: Open Maps
Course: CS 251, Fall 2023, UIC
System: VSCode, Advanced ZyLabs
Author: Carlos Velasquez, starter code provided by Adam Koehler
------------------------------------------- */

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <queue>
#include <stack>

#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"

using namespace std;
using namespace tinyxml2;

const double INF = numeric_limits<double>::max();

//This function searches for a building in a vector of building information based on either its abbreviation or a partial match in its full name
BuildingInfo searchBuilding(const vector<BuildingInfo>& Buildings, const string& query) 
{
  //Search by abbreviation first
  for (const BuildingInfo& building : Buildings) 
  {
    if (building.Abbrev == query) 
    {
      // Abbreviation match found
      return building;
    }
  }

  //If not found, search for a partial match in full names
  for (const BuildingInfo& building : Buildings) 
  {
    if (building.Fullname.find(query) != string::npos) 
    {
      //Partial name match found
      return building;
    }
  }

  //If no match is found return an empty BuildingInfo
  return BuildingInfo();
}

//This function finds the closest building to a specified center point among a given vector of buildings
BuildingInfo findClosestBuilding(const vector<BuildingInfo>& Buildings, const Coordinates& center) 
{
  double minDistance = INF;
  BuildingInfo closestBuilding;

  for (const BuildingInfo& building : Buildings) 
  {
    //Calculate the distance between the center point and the current building
    double distance = distBetween2Points(center.Lat, center.Lon, building.Coords.Lat, building.Coords.Lon);

    if (distance < minDistance) 
    {
      minDistance = distance;
      closestBuilding = building;
    }
  }

  return closestBuilding;
}

//Function to find the nearest footway node to a given building
long long findNearestFootwayNode(const vector<FootwayInfo>& Footways, const map<long long, Coordinates>& Nodes, const BuildingInfo& building) 
{
  double minDistance = INF;
  long long nearestNode = -1;

  for (const FootwayInfo& footway : Footways) 
  {
    for (const long long& node : footway.Nodes) 
    {
      //Calculate the distance between the building and the current node on the footway
      double distance = distBetween2Points(building.Coords.Lat, building.Coords.Lon, Nodes.at(node).Lat, Nodes.at(node).Lon);

      //Update the minimum distance and the nearest node
      if (distance < minDistance) 
      {
        minDistance = distance;
        nearestNode = node;
      }
    }
  }

  return nearestNode;
}

//Define the prioritize class
class prioritize 
{
public:
  bool operator()(const pair<long long, double>& p1, const pair<long long, double>& p2) const 
  {
    return p1.second > p2.second; 
  }
};

//Dijkstra's Shortest Path Algorithm
void DijkstraShortestPath(const graph<long long, double>& G, long long startV, map<long long, double>& distances, map<long long, long long>& predecessors) 
{
  priority_queue<pair<long long, double>, vector<pair<long long, double>>, prioritize> unvisitedQueue;

  //Initialize distances and predecessors
  for (const auto& vertex : G.getVertices()) 
  {
    distances[vertex] = INF;
    predecessors[vertex] = 0;
    unvisitedQueue.emplace(vertex, INF);
  }

  //Set the distance of the start vertex to 0
  distances[startV] = 0;
  unvisitedQueue.emplace(startV, 0);

  while (!unvisitedQueue.empty()) 
  {
    //Visit vertex with minimum distance from startV
    long long currentV = unvisitedQueue.top().first;
    unvisitedQueue.pop();

    for (const auto& neighbor : G.neighbors(currentV)) 
    {
      double edgeWeight;
      if (G.getWeight(currentV, neighbor, edgeWeight)) 
      {
        double alternativePathDistance = distances[currentV] + edgeWeight;

        //If shorter path from startV to neighbor is found, update neighbor's distance and predecessor
        if (alternativePathDistance < distances[neighbor]) 
        {
          distances[neighbor] = alternativePathDistance;
          predecessors[neighbor] = currentV;
          unvisitedQueue.emplace(neighbor, alternativePathDistance);
        }
      }
    }
  }
}

//Helper function to print the path from start to end using predecessors
void printPath(const map<long long, long long>& predecessors, long long destination) 
{
  stack<long long> path;
  long long current = destination;

  //Push nodes onto the stack until reaching the source node
  while (predecessors.find(current) != predecessors.end()) 
  {
    path.push(current);
    current = predecessors.at(current);
  }

  //Print the path 
  cout << "Path: ";
  while (!path.empty()) 
  {
    cout << path.top();
    path.pop();
    if (!path.empty())
    {
      cout << "->";
    }
  }
  cout << endl;
}

void application(map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways, vector<BuildingInfo>& Buildings, graph<long long, double>& G) 
{
  string person1Building, person2Building;
  bool pathFound = false;

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") 
  {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    //Search Buildings 1 and 2
    BuildingInfo building1 = searchBuilding(Buildings, person1Building);
    BuildingInfo building2 = searchBuilding(Buildings, person2Building);

    //Check if buildings are found
    if (building1.Fullname.empty() || building2.Fullname.empty()) 
    {
      if (building1.Fullname.empty()) 
      {
        cout << "Person 1's building not found" << endl;
      }

      if (building2.Fullname.empty()) 
      {
        cout << "Person 2's building not found" << endl;
      }

      //Skip steps 8-11 and get another pair of inputs
      cout << endl;
      cout << "Enter person 1's building (partial name or abbreviation), or #> ";
      getline(cin, person1Building);
      continue;
    }
    while (!pathFound) 
    {
      //Locate Center Building
      Coordinates center = centerBetween2Points(building1.Coords.Lat, building1.Coords.Lon, building2.Coords.Lat, building2.Coords.Lon);
      BuildingInfo closestBuilding = findClosestBuilding(Buildings, center);

      cout << endl;
      cout << "Person 1's point:" << endl;
      cout << " " << building1.Fullname << endl;
      cout << " (" << building1.Coords.Lat << ", " << building1.Coords.Lon << ")" << endl;

      cout << "Person 2's point:" << endl;
      cout << " " << building2.Fullname << endl;
      cout << " (" << building2.Coords.Lat << ", " << building2.Coords.Lon << ")" << endl;

      cout << "Destination Building:" << endl;
      cout << " " << closestBuilding.Fullname << endl;
      cout << " (" << closestBuilding.Coords.Lat << ", " << closestBuilding.Coords.Lon << ")" << endl << endl;
     
      //Find Nearest Nodes from buildings 1, 2 & Center
      long long nearestNode1 = findNearestFootwayNode(Footways, Nodes, building1);
      long long nearestNode2 = findNearestFootwayNode(Footways, Nodes, building2);
      long long nearestNodeCenter = findNearestFootwayNode(Footways, Nodes, closestBuilding);

      cout << "Nearest P1 node:" << endl;
      cout << " " << nearestNode1 << endl;
      cout << " (" << Nodes.at(nearestNode1).Lat << ", " << Nodes.at(nearestNode1).Lon << ")" << endl;
      cout << "Nearest P2 node:" << endl;
      cout << " " << nearestNode2 << endl;
      cout << " (" << Nodes.at(nearestNode2).Lat << ", " << Nodes.at(nearestNode2).Lon << ")" << endl;
      cout << "Nearest destination node:" << endl;
      cout << " " << nearestNodeCenter << endl;
      cout << " (" << Nodes.at(nearestNodeCenter).Lat << ", " << Nodes.at(nearestNodeCenter).Lon << ")" << endl << endl;

      //Run Dijkstra’s Algorithm 
      map<long long, double> distances1;
      map<long long, long long> predecessors1;
      DijkstraShortestPath(G, nearestNode1, distances1, predecessors1);

      //No path from building 1 to building 2
      if (distances1[nearestNode2] >= INF) 
      {
        cout << "Sorry, destination unreachable." << endl;
        pathFound = true;
        continue;
      }

      //Run Dijkstra’s Algorithm 
      map<long long, double> distances2;
      map<long long, long long> predecessors2;
      DijkstraShortestPath(G, nearestNode2, distances2, predecessors2);

      cout << "Person 1's distance to dest: " << distances1[nearestNodeCenter] << " miles" << endl;
      printPath(predecessors1, nearestNodeCenter);
      cout << endl;

      cout << "Person 2's distance to dest: " << distances2[nearestNodeCenter] << " miles" << endl;
      printPath(predecessors2, nearestNodeCenter);

      //Reset pathFound for the next iteration
      pathFound = true;
    }

    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
    pathFound = false;
  }
}

int main() 
{
  graph<long long, double> G;

  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates>  Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo>          Footways;
  // info about each building, in no particular order
  vector<BuildingInfo>         Buildings;
  XMLDocument                  xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") 
  {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) 
  {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;

  for (const auto& node : Nodes) 
  {
    G.addVertex(node.first);
  }

  for (const auto& footway : Footways) 
  {
    const vector<long long>& nodes = footway.Nodes;

    //Loop through nodes (up to second to last element)
    for (size_t i = 0; i < nodes.size() - 1; ++i) 
    {
      long long node1 = nodes[i];
      long long node2 = nodes[i + 1];

      //Get coordinates for the two nodes
      const Coordinates& coord1 = Nodes[node1];
      const Coordinates& coord2 = Nodes[node2];

      //Calculate the distance between the two nodes using distBetween2Points
      double weight = distBetween2Points(coord1.Lat, coord1.Lon, coord2.Lat, coord2.Lon);

      //Add bidirectional edges using the provided addEdge function
      G.addEdge(node1, node2, weight);
      G.addEdge(node2, node1, weight);
    }
  }

  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  //Execute Application
  application(Nodes, Footways, Buildings, G);

  cout << "** Done **" << endl;
  return 0;
}
