/*
OFF2Particle

Yuan Chiang (20th/Aug/2020)
*/

#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

#include <CGAL/Extreme_points_traits_adapter_3.h>

#include <CGAL/Vector_3.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
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

typedef CGAL::Point_set_3<Point> Point_set;
typedef Point_set::Property_map<int> Type_map;

typedef std::vector<Point> Points;
typedef std::list<Triangle>::iterator Iterator;
typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
typedef CGAL::AABB_traits<K, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;

typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;

int main(int argc, char* argv[])
{
    time_t start_main = time(NULL);
    printf("OFF2Particle starts at %s\n", ctime(&start_main));
    const clock_t start_time = clock();

    const char* filename = (argc > 1) ? argv[1] : "data/tetrahedron.off";
    std::ifstream input(filename);

    printf("Reading mesh from %s...\n", filename);
    Mesh mesh;
    if(! input || !(input >> mesh) || !mesh.is_valid()){
      printf("\tNot a valid off file.\n");
      return EXIT_FAILURE;
    }

    printf("\tNumber of vertices %10d\n", mesh.number_of_vertices());
    printf("\tNumber of edges    %10d\n", mesh.number_of_edges());
    printf("\tNumber of faces    %10d\n", mesh.number_of_faces());

    // Vector phase()

    // Point origin(0.0, 0.0, 0.0);

    double lc = (argc > 2) ? atof(argv[2]) : 1.0;
    Vector fcc_a1(0.5, 0.5, 0.0);
    Vector fcc_a2(0.0, 0.5, 0.5);
    Vector fcc_a3(0.5, 0.0, 0.5);

    printf("Computing lattices...\n");
    // define polyhedron to hold convex hull
    // Mesh convex_hull;

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
    Point centroid(xc/=mesh.number_of_vertices(),
                   yc/=mesh.number_of_vertices(),
                   zc/=mesh.number_of_vertices());

    int nx = 0;
    int ny = 0;
    int nz = 0;
    for(vertex_descriptor vd : mesh.vertices()) {
      Vector vertex(centroid, coord[vd]);
      if (abs((vertex[0])/(lc)) > nx)
        nx = (int)ceil(abs((vertex[0])/(lc)));
      if (abs((vertex[1])/(lc)) > ny)
        ny = (int)ceil(abs((vertex[1])/(lc)));
      if (abs((vertex[2])/(lc)) > nz)
        nz = (int)ceil(abs((vertex[2])/(lc)));
      // if (abs((vertex*fcc_a1)/(fcc_a1*fcc_a1)) > n_a1)
      //   n_a1 = (int)ceil(abs((vertex*fcc_a1)/(fcc_a1*fcc_a1)));
      // if (abs((vertex*fcc_a2)/(fcc_a2*fcc_a2)) > n_a2)
      //   n_a2 = (int)ceil(abs((vertex*fcc_a2)/(fcc_a2*fcc_a2)));
      // if (abs((vertex*fcc_a3)/(fcc_a3*fcc_a3)) > n_a3)
      //   n_a3 = (int)ceil(abs((vertex*fcc_a3)/(fcc_a3*fcc_a3)));
    }


    // compute convex hull of non-collinear points
    // CGAL::convex_hull_3(vertices.begin(), vertices.end(), convex_hull);

    // printf("\tNumber of vertices %10d\n", convex_hull.number_of_vertices());
    // printf("\tNumber of edges    %10d\n", convex_hull.number_of_edges());
    // printf("\tNumber of faces    %10d\n", convex_hull.number_of_faces());

    std::cout << "\tCentroid of vertices is located at " << centroid << std::endl;
    printf("\tNumber of lattices %3d %3d %3d\n", 2*nx+1, 2*ny+1, 2*nz+1);

    // export convex hull
    // std::ofstream output("output/convex_hull.off");
    // output << convex_hull;

    // //This will contain the extreme vertices
    // std::vector<Mesh::Vertex_index> extreme_vertices;
    // //call the function with the traits adapter for vertices
    //
    // CGAL::extreme_points_3(vertices(convex_hull),std::back_inserter(extreme_vertices), CGAL::make_extreme_points_traits_adapter(convex_hull.points()));
    // //print the number of extreme vertices
    // std::cout << "There are  " << extreme_vertices.size() << " extreme vertices in this mesh." << std::endl;

    printf("Establishing AABB tree...\n");
    Tree tree(faces(mesh).first, faces(mesh).second, mesh);

    printf("Generating particles...\r");

    Point_set particles;
    Type_map type;

    bool success = false;
    boost::tie (type, success) = particles.add_property_map<int> ("type", 0);
    assert(success);

    srand (time(NULL));
    int lattice_iter = 0;
    for (int i = -nx; i <= nx; i++){
      for (int j = -ny; j <= ny; j++){
        for (int k = -nz; k <= nz; k++){
          lattice_iter = lattice_iter + 1;
          printf("Generating particles... %3.0f%% \r", (double)(lattice_iter) / (double)((2*nx+1)*(2*ny+1)*(2*nz+1)) * 100.0);
          Vector a0(i,j,k);
          Vector epsilon(rand(),rand(),rand());
          Ray ray_0(centroid + lc*a0,epsilon);
          Ray ray_1(centroid + lc*a0 + lc*fcc_a1,epsilon);
          Ray ray_2(centroid + lc*a0 + lc*fcc_a2,epsilon);
          Ray ray_3(centroid + lc*a0 + lc*fcc_a3,epsilon);

          if (tree.number_of_intersected_primitives(ray_0) % 2 == 1)
            Point_set::iterator ita0 = particles.insert(centroid + lc*a0);
          if (tree.number_of_intersected_primitives(ray_1) % 2 == 1)
            Point_set::iterator ita1 = particles.insert(centroid + lc*a0 + lc*fcc_a1);
          if (tree.number_of_intersected_primitives(ray_2) % 2 == 1)
            Point_set::iterator ita2 = particles.insert(centroid + lc*a0 + lc*fcc_a2);
          if (tree.number_of_intersected_primitives(ray_3) % 2 == 1)
            Point_set::iterator ita3 = particles.insert(centroid + lc*a0 + lc*fcc_a3);
          // type[*ita0] = 1;
          // type[*ita1] = 1;
          // type[*ita2] = 1;
          // type[*ita3] = 1;
        }
      }
    }

    printf("Generating particles...\n");
    printf("\tNumer of particles %10ld\n",particles.number_of_points());

    const char* outfile = (argc > 3) ? argv[3] : "particles.xyz";
    std::ofstream outxyz(outfile);
    outxyz << "# Particle coordinates created by STL2Particle at " << ctime(&start_main) << std::endl;
    CGAL::write_xyz_point_set(outxyz, particles);
    outxyz.close();

    //
    //
    // std::cout << tree.number_of_intersected_primitives(ray_query)
    //     << " intersections(s) with ray query" << std::endl;


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

    const clock_t end_time = clock();
    printf("Elapsed time: %f sec\n", float( end_time - start_time ) / CLOCKS_PER_SEC);

    time_t end_main = time(NULL);
    printf("OFF2Particle ends at %s\n", ctime(&end_main));
    return EXIT_SUCCESS;
}
