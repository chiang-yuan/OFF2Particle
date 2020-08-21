/*
STL2Particle

Yuan Chiang (20th/Aug/2020)
*/

#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <string>
#include <stdio.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

#include <CGAL/Extreme_points_traits_adapter_3.h>

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
    const char* filename = (argc > 1) ? argv[1] : "data/tetrahedron.off";
    std::ifstream input(filename);

    printf("Reading mesh from %s...\n", filename);
    Mesh mesh;
    if(! input || !(input >> mesh) || !mesh.is_valid()){
      std::cerr << "Not a valid off file." << std::endl;
    }

    printf("\tNumber of vertices %10d\n", mesh.number_of_vertices());
    printf("\tNumber of edges    %10d\n", mesh.number_of_edges());
    printf("\tNumber of faces    %10d\n", mesh.number_of_faces());

    printf("Computing convex hull...\n");
    // define polyhedron to hold convex hull
    Mesh convex_hull;

    // access the vertices of
    Points vertices;
    double xc, yc, zc;
    Mesh::Property_map<vertex_descriptor, Point> coord = mesh.points();
    for(vertex_descriptor vd : mesh.vertices()) {
      vertices.push_back(coord[vd]);
      xc = xc + coord[vd][0];
      yc = yc + coord[vd][1];
      zc = zc + coord[vd][2];
    }
    xc/=mesh.number_of_vertices();
    yc/=mesh.number_of_vertices();
    zc/=mesh.number_of_vertices();
    Point centroid(xc, yc, zc);

    // std::cout << "The centroid of vertices is located at " << centroid <<  std::endl;

    // compute convex hull of non-collinear points
    CGAL::convex_hull_3(vertices.begin(), vertices.end(), convex_hull);
    std::cout << "\tThe convex hull contains " << convex_hull.number_of_vertices() << " vertices" << std::endl;

    // export convex hull
    std::ofstream output("output/convex_hull.off");
    output << convex_hull;
    
    // //This will contain the extreme vertices
    // std::vector<Mesh::Vertex_index> extreme_vertices;
    // //call the function with the traits adapter for vertices
    //
    // CGAL::extreme_points_3(vertices(convex_hull),std::back_inserter(extreme_vertices), CGAL::make_extreme_points_traits_adapter(convex_hull.points()));
    // //print the number of extreme vertices
    // std::cout << "There are  " << extreme_vertices.size() << " extreme vertices in this mesh." << std::endl;

    double lc = 1.0;
    Vector fcc_a1(0.0, 0.5, 0.5);
    Vector fcc_a2(0.5, 0.0, 0.5);
    Vector fcc_a3(0.5, 0.5, 0.0);

    Vector epsilon(1.0,1.0,1.0);

    Ray ray_query(centroid,epsilon);

    printf("Establishing AABB tree...\n");
    Tree tree(faces(mesh).first, faces(mesh).second, mesh);

    std::cout << tree.number_of_intersected_primitives(ray_query)
        << " intersections(s) with ray query" << std::endl;



    Point a(1.0, 0.0, 0.0);
    Point b(0.0, 1.0, 0.0);
    Point c(0.0, 0.0, 1.0);
    Point d(0.0, 0.0, 0.0);
    Point e(-1.0, 0.0, 0);
    Point f(1, 1, 1);
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
