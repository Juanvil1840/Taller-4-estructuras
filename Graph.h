#ifndef __GRAPH_H__
#define __GRAPH_H__

#include "Pair.h"
#include <vector>
#include <list>

template < class T, class P >
class Graph {
    protected:
	std::vector<T> vertices;
	std::list<std::list<Pair<P>>> edges;
    public:
	Graph();
	Graph(std::vector<T> &vert, std::list<std::list<Pair<P>>> &edges);
	~Graph();

	std::vector<T>& getVertices();
	std::list<std::list<Pair<P>>>& getEdges();
	void setVertices(std::vector<T>& newVertices);
	bool setEdges(std::list<std::list<Pair<P>>>& newEdges);
	
	// Vertex operations
	long vertexCount();
	bool insertVertex(T data);
	long findVertex(T data);
	bool removeVertex(T data);
	T getVertex(long index);

	
	// Edge operations
	long edgeCount();
	bool insertDirectedEdge(T source, T destination, P weight);
	bool insertUndirectedEdge(T source, T destination, P weight);
	P* findEdge(T source, T destination); 
	bool removeEdge(T source, T destination); 

	//Traversals
	void flatTraversal(); 
	bool DFS(T vertex, std::vector<T>& visited);
	bool BFS(T vertex, std::vector<T>& visited); 
};

#include "Graph.hxx"

#endif