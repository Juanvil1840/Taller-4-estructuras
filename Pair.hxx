#include "Pair.h"

template< class P >
Pair<P>::Pair() {
    index = -1;
    weight = P{};
}

template< class P >
Pair<P>::Pair(int ind, P cost){
    index = ind;
    weight = cost;
}

template< class P >
Pair<P>::~Pair() = default;

template< class P >
int Pair<P>::getIndex() const{
    return index;
}

template< class P >
P Pair<P>::getWeight() const{
    return weight;
}

template< class P >
void Pair<P>::setIndex(int ind){
    index = ind;
}

template< class P >
void Pair<P>::setweight(P wei){
    weight = wei;
}