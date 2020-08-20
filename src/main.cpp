/*
STL2Particle

Yuan Chiang (20th/Aug/2020)

Adapted from CGAL 5.0.3 example - AABB Tree
Author(s) : Camille Wormser, Pierre Alliez
*/

#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <string>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

#include <CGAL/Vector_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Exact_predicates_exact_constructions_kernel EK;
// typedef CGAL::Simple_cartesian<double> K;

typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line;
typedef K::Point_3 Point;
typedef K::Vector_3 Vector;
typedef K::Triangle_3 Triangle;
typedef CGAL::Polyhedron_3<K> Polyhedron;

typedef CGAL::Surface_mesh<Point> Mesh;

typedef std::vector<Point> Points;
typedef std::list<Triangle>::iterator Iterator;
typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
typedef CGAL::AABB_traits<K, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;

typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;



int main(int argc, char* argv[])
{
    Point a(1.0, 0.0, 0.0);
    Point b(0.0, 1.0, 0.0);
    Point c(0.0, 0.0, 1.0);
    Point d(0.0, 0.0, 0.0);
    Point e(-1.0, 0.0, 0);
    Point f(1, 1, 1);



    const char* filename = (argc > 1) ? argv[1] : "data/tetrahedron.off";
    std::ifstream input(filename);
    Mesh mesh;
    input >> mesh;
    Tree tree(faces(mesh).first, faces(mesh).second, mesh);

    // define polyhedron to hold convex hull
    Polyhedron poly;

    // compute convex hull of non-collinear points
    Points points, result;

    std::cout << mesh.points() << std::endl;
    std::cout << mesh << std::endl;

    double xc, yc, zc;
    Mesh::Property_map<vertex_descriptor, Point> coord = mesh.points();
    for(vertex_descriptor vd : mesh.vertices()) {
      std::cout << coord[vd] << std::endl;
      xc = xc + coord[vd][0];
      yc = yc + coord[vd][1];
      zc = zc + coord[vd][2];
    }
    xc/=3;
    yc/=3;
    zc/=3;
    Point centroid(xc, yc, zc);

    Vector epsilon(1.0,1.0,1.0);

    std::cout << "The centroid is located at " << centroid <<  std::endl;

    std::cout << "The convex hull contains " << poly.size_of_vertices() << " vertices" << std::endl;



    Ray ray_query(centroid,epsilon);
    std::cout << tree.number_of_intersected_primitives(ray_query)
        << " intersections(s) with ray query" << std::endl;


    // Mesh m;
    // vertex_descriptor v0 = m.add_vertex(K::Point_3(0,2,0));
    // vertex_descriptor v1 = m.add_vertex(K::Point_3(2,2,0));
    // vertex_descriptor v2 = m.add_vertex(K::Point_3(0,0,0));
    // vertex_descriptor v3 = m.add_vertex(K::Point_3(2,0,0));
    // vertex_descriptor v4 = m.add_vertex(K::Point_3(1,1,0));
    // m.add_face(v3, v1, v4);
    // m.add_face(v0, v4, v1);
    // m.add_face(v0, v2, v4);
    // m.add_face(v2, v3, v4);
    // // give each vertex a name, the default is empty
    // Mesh::Property_map<vertex_descriptor,std::string> name;
    // bool created;
    // boost::tie(name, created) = m.add_property_map<vertex_descriptor,std::string>("v:name","");
    // assert(created);
    // // add some names to the vertices
    // name[v0] = "hello";
    // name[v2] = "world";
    // {
    //   // You get an existing property, and created will be false
    //   Mesh::Property_map<vertex_descriptor,std::string> name;
    //   bool created;
    //   boost::tie(name, created) = m.add_property_map<vertex_descriptor,std::string>("v:name", "");
    //   assert(! created);
    // }
    // //  You can't get a property that does not exist
    // Mesh::Property_map<face_descriptor,std::string> gnus;
    // bool found;
    // boost::tie(gnus, found) = m.property_map<face_descriptor,std::string>("v:gnus");
    // assert(! found);
    // // retrieve the point property for which exists a convenience function
    // Mesh::Property_map<vertex_descriptor, K::Point_3> location = m.points();
    // for(vertex_descriptor vd : m.vertices()) {
    //   std::cout << name[vd] << " @ " << location[vd] << std::endl;
    // }
    // std::vector<std::string> props = m.properties<vertex_descriptor>();
    // for(std::string p : props){
    //   std::cout << p << std::endl;
    // }
    // // delete the string property again
    // m.remove_property_map(name);
    // std::vector<std::string> props_del = m.properties<vertex_descriptor>();
    // for(std::string p : props_del){
    //   std::cout << p << std::endl;
    // }


    return EXIT_SUCCESS;
}
