#ifndef __PAIR_H__
#define __PAIR_H__

template< class P >
class Pair{
  protected:
    int index;
    P weight;
  public:
    Pair();
    Pair(int ind, P wei);
    ~Pair();
    int getIndex() const;
    P getWeight() const;
    void setIndex(int ind);
    void setweight(P wei);
};

#include "Pair.hxx"

#endif

