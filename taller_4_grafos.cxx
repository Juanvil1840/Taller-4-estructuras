#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include "Graph.h"

struct Point
{
  float X, Y, Z;
  float distanceTo( const Point& p ) const
    {
      float x = X - p.X;
      float y = Y - p.Y;
      float z = Z - p.Z;
      return( std::sqrt( ( x * x ) + ( y * y ) + ( z * z ) ) );
    }

   //Es necesario sobrecargar el operador = para poder hacer los recorridos en el grafo ya que se utiliza std::find
    bool operator==(const Point& p) const {
        return X == p.X && Y == p.Y && Z == p.Z;
    }

   //Es necesario sobrecargar el operador < para poder hacer los recorridos en el grafo ya que se utiliza sort()
    bool operator<(const Point& p) const {
        return (X < p.X) || (X == p.X && (Y < p.Y || (Y == p.Y && Z < p.Z)));
    }
};

// TODO 1
typedef Graph< Point, float > TGraph; 

int main( int argc, char* argv[] )
{
  if( argc < 4 )
  {
    std::cerr
      << "Usage: " << argv[ 0 ] << " input_mesh start end"
      << std::endl;
    return( 1 );

  } 
  long start_id = std::atoi( argv[ 2 ] ); //
  long end_id = std::atoi( argv[ 3 ] );

  // TODO 2 
  TGraph g;

  // Load file in a buffer
  std::ifstream in_mesh_stream( argv[ 1 ], std::ifstream::binary );
  if( !in_mesh_stream )
  {
    std::cerr << "Error reading \"" << argv[ 1 ] << "\"" << std::endl;
    return( 1 );

  }

  in_mesh_stream.seekg( 0, in_mesh_stream.end );
  unsigned long in_mesh_file_length = in_mesh_stream.tellg( );
  in_mesh_stream.seekg( 0, in_mesh_stream.beg );
  char* in_mesh_file_buffer = new char[ in_mesh_file_length ];
  in_mesh_stream.read( in_mesh_file_buffer, in_mesh_file_length );
  in_mesh_stream.close( );
  std::istringstream in_mesh( in_mesh_file_buffer );

  // Read vertices
  long nPoints;
  in_mesh >> nPoints;
  for( long pId = 0; pId < nPoints; pId++ )
  {
    Point pnt;
    in_mesh >> pnt.X >> pnt.Y >> pnt.Z;

    // TODO 3
    g.insertVertex(pnt);

  }

  // Read edges
  long nEdges;
  in_mesh >> nEdges;
  for( long eId = 0; eId < nEdges; eId++ )
  {
    long start, end;
    in_mesh >> start >> end;

    //TODO 4
    float cost = g.getVertex(start).distanceTo(g.getVertex(end));
    g.insertUndirectedEdge(g.getVertices()[start],g.getVertices()[end],cost);
  }
  delete [] in_mesh_file_buffer;

  if(
    start_id < 0 || start_id >= g.vertexCount() ||
    end_id < 0 || end_id >= g.vertexCount()
    )
  {
    std::cerr << "Invalid path endpoints." << std::endl;
    return( 1 );
  }

  /*
    TODO 5:
    std::vector< long > path = g.Dijkstra( start_id, end_id );
    std::cout << path.size( ) << std::endl;
    for( unsigned int i = 0; i < path.size( ); ++i )
    std::cout
    << vertices[ path[ i ] ].X << " "
    << vertices[ path[ i ] ].Y << " "
    << vertices[ path[ i ] ].Z << std::endl;
  */

  return( 0 );
}
