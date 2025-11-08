#ifndef __PAIR_H__
#define __PAIR_H__

template< class P >
class Pair{
  protected:
    long index;
    P weight;
  public:
    Pair();
    Pair(long ind, P wei);
    ~Pair();
    long getIndex() const;
    P getWeight() const;
    void setIndex(long ind);
    void setweight(P wei);
};

#include "Pair.hxx"

#endif

