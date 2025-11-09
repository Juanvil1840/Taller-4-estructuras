#include "Graph.h"
#include <algorithm>
#include <queue>

template < class T, class P >
Graph<T,P>::Graph() {
    vertices.clear();
    edges.clear();
}

template < class T, class P >
Graph<T,P>::Graph(std::vector<T> &vert, std::list<std::list<Pair<P>>> &edg){
    vertices = vert; 

    //Como el grafo es un par ordenado, es importante verificar que exista el conjunto de vertices para poder crear el conjunto de aristas
    //Se asume que si el tamaño de la lista es diferente al del vector, la lista de adyacencia no conteine los mismos vertices que el vector
    if(!vertices.empty() && (vertices.size() == edg.size())){
	edges = edg;
    }else{
	edges.clear();
    }
}

template < class T, class P >
Graph<T,P>::~Graph() {
    vertices.clear();
    edges.clear();
}

template < class T, class P >
std::vector<T>& Graph<T,P>::getVertices(){
    return vertices;
}

template < class T, class P >
std::list<std::list<Pair<P>>>& Graph<T,P>::getEdges(){
    return edges;
}

template < class T, class P >
void Graph<T,P>::setVertices(std::vector<T>& newVertices){
    vertices = newVertices;
}

template < class T, class P >
bool Graph<T,P>::setEdges(std::list<std::list<Pair<P>>>& newEdges){
    if(!vertices.empty() && (vertices.size() == newEdges.size())){
	edges = newEdges;
	return true;
    }
	
    return false;
}

// Vertex operations
template < class T, class P >
long Graph<T,P>::vertexCount(){
    return (long)vertices.size();
}

template < class T, class P >
bool Graph<T,P>::insertVertex(T data){
    if(findVertex(data) != -1) return false;

    vertices.push_back(data);
    edges.push_back(std::list<Pair<P>>{});

    return true;    
}

template < class T, class P >
long Graph<T,P>::findVertex(T data){
    for(std::size_t i = 0; i < vertices.size(); i++){
	if(vertices[i] == data) return (long)i;
    }   

    return -1; 
}

template < class T, class P >
bool Graph<T,P>::removeVertex(T data){
    long index = findVertex(data);

    if(index == -1) return false;

    typename std::list<std::list<Pair<P>>>::iterator ita = edges.begin();
    std::advance(ita, index);

    typename std::list<Pair<P>>::iterator itp;
    for(itp = ita->begin();itp != ita->end(); itp ++){
	delete *itp;
    }
    edges.erase(ita);

    std::vector<typename std::list<Pair<P>>::iterator> removedConnections;
    long pairIndex;
    for(ita = edges.begin();ita != edges.end(); ita ++){
	for(itp = ita->begin();itp != ita->end(); itp ++){
	    pairIndex = itp->getIndex();
	    if(pairIndex == index) removedConnections.push_back(itp);

	    if(pairIndex > index) itp->setIndex(pairIndex - 1);
        }
	
	for(std::size_t i= 0; i <removedConnections.size(); i++){
	    ita->erase(removedConnections[i]);
	}

	removedConnections.clear();
    }

    typename std::vector<T>::iterator itv = vertices.begin() + index;
    vertices.erase(itv);

    return true;
}

template < class T, class P >
T Graph<T,P>::getVertex(long index){
    return vertices[index];
}

// Edge operations
template < class T, class P >
long Graph<T,P>::edgeCount(){
    long count = 0;
    typename std::list<std::list<Pair<P>>>::iterator it;
    
    for(it= edges.begin(); it!= edges.end(); it ++){
	count += (long)(it->size());
    }

    return count;
}

template < class T, class P >
bool Graph<T,P>::insertDirectedEdge(long sourceIndex, long destinationIndex, P weight){    
    if((sourceIndex < 0) || (sourceIndex >= (long)vertices.size()) || (destinationIndex < 0) || (destinationIndex >= (long)vertices.size())) return false;

    typename std::list<std::list<Pair<P>>>::iterator ita = edges.begin();
    std::advance(ita, sourceIndex);

    typename std::list<Pair<P>>::iterator itp;
    for(itp = ita->begin();itp != ita->end(); itp ++){
	if(itp->getIndex() == destinationIndex) return false;
    }

    Pair<P> newPair(destinationIndex, weight);
    ita->push_back(newPair);        
    return true;
}

template < class T, class P >
bool Graph<T,P>::insertUndirectedEdge(long sourceIndex, long destinationIndex, P weight){
    if((sourceIndex < 0) || (sourceIndex >= (long)vertices.size()) || (destinationIndex < 0) || (destinationIndex >= (long)vertices.size())) return false;

    typename std::list<std::list<Pair<P>>>::iterator ita = edges.begin();
    std::advance(ita, sourceIndex);

    typename std::list<Pair<P>>::iterator itp;
    for(itp = ita->begin();itp != ita->end(); itp ++){
	if(itp->getIndex() == destinationIndex) return false;
    }

    Pair<P> pair1(destinationIndex, weight);
    ita->push_back(pair1);

    Pair<P> pair2(sourceIndex, weight);
    ita = edges.begin();    
    std::advance(ita, destinationIndex);  
    ita->push_back(pair2); 
    return true;
}

template < class T, class P >
P* Graph<T,P>::findEdge(T source, T destination){
    long sourceIndex = findVertex(source);
    long destinationIndex = findVertex(destination);
    
    if((sourceIndex == -1) || (destinationIndex == -1)) return false;

    typename std::list<std::list<Pair<P>>>::iterator ita = edges.begin();
    std::advance(ita, sourceIndex);

    typename std::list<Pair<P>>::iterator itp;
    for(itp = ita->begin();itp != ita->end(); itp ++){
	if(itp->getIndex() == destinationIndex){
	    return & itp->getWeight();
	} 
    }

    return nullptr;
}

template < class T, class P >
bool Graph<T,P>::removeEdge(T source, T destination){
    long sourceIndex = findVertex(source);
    long destinationIndex = findVertex(destination);
    
    if((sourceIndex == -1) || (destinationIndex == -1)) return false;

    typename std::list<std::list<Pair<P>>>::iterator ita = edges.begin();
    std::advance(ita, sourceIndex);

    typename std::list<Pair<P>>::iterator itp;
    for(itp = ita->begin();itp != ita->end(); itp ++){
	if(itp->getIndex() == destinationIndex){
	    ita->erase(itp);  
	    return true;  
	} 
    }

    return false;
}

//Traversals
template < class T, class P >
void Graph<T,P>::flatTraversal(){
    for(std::size_t i = 0; i < vertices.size(); i++){
	std::cout << vertices[i] << " ";
    }

    std::cout << std::endl;
}

template < class T, class P >
bool Graph<T,P>::DFS(T vertex, std::vector<T>& visited){
    long index = findVertex(vertex);
    if(index == -1) return false;

    if(std::find(visited.begin(),visited.end(), vertex) != visited.end()) return true;

    visited.push_back(vertices[index]);
    typename std::list<std::list<Pair<P>>>::iterator ita = edges.begin();
    std::advance(ita, index);

    //Ordenar la lista de vecinos inmediatos de menor a mayor
    ita->sort([this](const Pair<P>& a, const Pair<P>& b){
    return vertices[a.getIndex()] < vertices[b.getIndex()];
    });

    typename std::list<Pair<P>>::iterator itp;
    for(itp = ita->begin();itp != ita->end(); itp ++){
	T neighbor = vertices[itp->getIndex()];
	if(std::find(visited.begin(),visited.end(), vertices[itp->getIndex()]) == visited.end()){
	    if(!DFS(neighbor,visited)) return false;
	}
    }

    return true;
}

template < class T, class P >
bool Graph<T,P>::BFS(T vertex, std::vector<T>& visited){
    long index = findVertex(vertex);
    if(index == -1) return false;

    visited.push_back(vertices[index]);

    //Empujar el nodo en la cola
    std::queue<long> verticesQueue;
    verticesQueue.push(index);

    while(!verticesQueue.empty()){
	long neighbor = verticesQueue.front();
	verticesQueue.pop();

	typename std::list<std::list<Pair<P>>>::iterator ita = edges.begin();
        std::advance(ita,neighbor);

	//Ordenar la lista de vecinos inmediatos de menor a mayor
        ita->sort([this](const Pair<P>& a, const Pair<P>& b){
        return vertices[a.getIndex()] < vertices[b.getIndex()];
        });

	typename std::list<Pair<P>>::iterator itp;
	for(itp = ita->begin();itp != ita->end(); itp ++){
	    if(std::find(visited.begin(),visited.end(), vertices[itp->getIndex()]) == visited.end()){
		visited.push_back(vertices[itp->getIndex()]);
	        verticesQueue.push(itp->getIndex());
	    }
	}	
    }

    return true;
} 

template <class T, class P>
std::vector<long> Graph<T,P>::Dijkstra(long startIndex, long endIndex){

    long n = (long)vertices.size(); // Obtiene cuantos vértices hay en el grafo 

    std::vector<P> dist; // vector dist de tipo plantilla
    dist.resize(n); 

    std::vector<long> pred; // vector de predecesores   <-- corregido (era long0)
    pred.resize(n);

    std::vector<bool> S; //vector de nodos ya visitados
    S.resize(n);

    //inicializar
    long i;
    for(i = 0; i<n; i++){
        dist[i] = (P)9999999999999999; // representa un infinito
        pred[i]= -1; //ningun predecesor
        S[i] = false; // nadie ha sido visitado
    }

    //nodo inicial
    dist[startIndex] = (P)0; // la distancia del inicio es 0
    pred[startIndex] = startIndex; // El predecesor del inicio es él mismo

    // Se busca el vértice no visitado que tenga la menor distancia desde el nodo origen
    for(long k=0; k<n; k++){
        long ind = -1; //índice del vertice con menor distancia 
        P minDist = (P)9999999999999999; //minima distancia encontrada hasta ahroa
        for(long j = 0; j<n; j++){
            if(!S[j] && dist[j] < minDist){ // si visitado es falso y distancia es menor a la minima distancia encontrada hasta ahora
                minDist = dist[j];// minDist guarda la menor distancia encontrada hasta esta iteración
                ind = j; // ind es el índice del vértice que tiene esa menor distancia
            }
        }
        if(ind == -1){ break;} // no hay más alcanzables

        S[ind] = true;   // <-- corregido (antes decía visited)

        
        if(ind == endIndex){break;} //Si se llega al final, termin
        

        // despues de escojer la distancia minima, se debe revisar si pasar por ind mejora la distancia hacia cada vecino

        // obtener lista de vecino de ind
        typename std::list<std::list<Pair<P>>>::iterator ita = edges.begin();
        std::advance(ita, ind);

        
        typename std::list<Pair<P>>::iterator itp;
        for(itp = ita->begin(); itp != ita->end(); itp++){
            long v = itp->getIndex();
            P weight = itp->getWeight();
            if(!S[v] && dist[ind] + weight < dist[v]){ // si el nodo no ha sido visitade y la distancia más el peso de la conexión es menor a la distancia actual para ese nodo 
                dist[v] = dist[ind] + weight;
                pred[v] = ind;   // <-- corregido (antes decía u, variable inexistente)
            }
        }

    }

    //reconstruir el camino desde endIndex hacía atrás 
    std::vector<long> path;
    long actual = endIndex;

    if(pred[actual] == -1){return path;}

    while(actual != startIndex){
        path.insert(path.begin(), actual);
        actual = pred[actual];
    }

    path.insert(path.begin(), startIndex);

    return path;
}
