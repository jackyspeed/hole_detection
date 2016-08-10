#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <ctime>
#include <math.h>

#define PI 3.14159265

class Node{
  public:
    float x;
    float y;
    float z;
    int prob;
    int n_count;
    Node(pcl::PointXYZ ptr);
    Node();
    Node* neighbors[10];
    void print_neighbors();
    void add_neighbor(pcl::PointXYZ pt);
    void decrement_counter();
};

Node::Node(){}

Node::Node(pcl::PointXYZ point){ 
  n_count = 0;
  x = point.x;
  y = point.y;
  z = point.z;
  prob = 0;
}

void Node::add_neighbor(pcl::PointXYZ pt){
  neighbors[n_count++] = new Node(pt);
}

void Node::decrement_counter(){
  n_count--;
}

void Node::print_neighbors(){
  for(int i = 0; i < n_count; i++){
    std::cout<<neighbors[i]->x<<" "<<neighbors[i]->y<<" "<<neighbors[i]->z<<std::endl;
  }
}

// prints help menu
void showHelp()
{
	std::cout << std::endl;
	std::cout << "Commands:" << std::endl;
	std::cout << "\n_help:  Show this help." << std::endl;
	std::cout << "_tree:  Runs kd tree algorithm with specified K number of closest points and/or with a specified radius." << std::endl;
	std::cout << "_quit:  Exits program." << std::endl;	
	std::cout << "_points:  Prints points contained in point cloud." << std::endl;
	std::cout << "_normals:  Prints normals of points in cloud." << std::endl;
	std::cout << "_visualize: Displays point cloud of .pcd file that has been processed in calculate_hole" << std::endl;
	std::cout << "_holes:  Calculates hole (returns set of points on boundary and visualizes image with boundary points red).\n" << std::endl;
}


// prints out all points in PCL dataset
void points(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
	for(unsigned i = 0; i < cloud->points.size(); i++){
		std::cout<<cloud->points[i]<<std::endl;
	}
}


// finds the normals of each point and prints it to standard out
void normals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;	

	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree3 (new pcl::search::KdTree<pcl::PointXYZ> ());

	ne.setSearchMethod (tree3);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (5);

	// Compute the features
	ne.compute (*cloud_normals);

	std::cout << "size of the normals " << cloud_normals->points.size() << std::endl ; 		
	for(unsigned i = 0; i < cloud_normals->points.size(); i++){
		std::cout<< cloud_normals->points[i] <<std::endl;
	}
}

// finds angle between two three dimensional vectors
float angle_between_vectors (float *nu, float *nv){
	float l1 = sqrt(nu[0]*nu[0] + nu[1]*nu[1] + nu[2]*nu[2]);
	float l2 = sqrt(nv[0]*nv[0] + nv[1]*nv[1] + nv[2]*nv[2]);
	float dot = nu[0]*nv[0] + nu[1]*nv[1] + nu[2]*nv[2];
	float param = dot/(l1*l2);
	//if (param < 0)
	//param = -(param);
	float angle = std::asin(param);
  angle = fabs(angle*180/PI);
	angle = floor(angle*100 + 0.5)/100 ;  // round off to two decimal places
	return angle ;
}


// constructs a kd tree with a k number of closest points or a radius of a
// boundary that encloses all points or both
void kd_tree(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int x=1, int y=1, int z=1){
	bool r = false;
	bool k = false;
	std::cout<<"Would you like a radius(r) search, a K(k) search or both(b): "<<std::endl;
	char input;
	while(true){
		std::cin >> input;	
		if(input == 'k'){
			k = true;
			break;
		}
		else if(input == 'r'){
			r = true;
			break;
		}
		else if(input == 'b'){
			k = true;
			r = true;
			break;
		}	
		else{
			std::cout<<"option '"<<input<<"' doesn't exist"<<std::endl;
		}
	}

	srand (time (NULL));

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	kdtree.setInputCloud (cloud);

	pcl::PointXYZ searchPoint;

	searchPoint.x = x;
	searchPoint.y = y;
	searchPoint.z = z;

	if(k){
		std::cout<< "Enter a K value: " <<std::endl;
		int K = 0;
		std::cin >> K;

		// K nearest neighbor search
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);

		std::cout << "K nearest neighbor search at (" << searchPoint.x 
			<< ", " << searchPoint.y 
			<< ", " << searchPoint.z
			<< ") with K=" << K << std::endl;

    std::ofstream new_file;
  	new_file.open ("new_file.pcd", std::ios_base::app);
    
		if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
			for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
				std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
					<< ", " << cloud->points[ pointIdxNKNSearch[i] ].y 
					<< ", " << cloud->points[ pointIdxNKNSearch[i] ].z 
					<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl; 
        new_file << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
					<< ", " << cloud->points[ pointIdxNKNSearch[i] ].y 
					<< ", " << cloud->points[ pointIdxNKNSearch[i] ].z 
					<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
      }
    }
    new_file.close();
	}

	if(r){
		std::cout<< "Enter a radius: " <<std::endl;
		float radius = 0;
		std::cin >> radius;

		// Neighbors within radius search
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		std::cout << "Neighbors within radius search at (" << searchPoint.x 
			<< ", " << searchPoint.y 
			<< ", " << searchPoint.z
			<< ") with radius=" << radius << std::endl;


		if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i){
				std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
					<< ", " << cloud->points[ pointIdxRadiusSearch[i] ].y 
					<< ", " << cloud->points[ pointIdxRadiusSearch[i] ].z 
					<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
      }
    std::cout << "points: " << pointIdxRadiusSearch.size() << std::endl;
	  }
  }
}

bool is_equal(pcl::PointXYZ first, pcl::PointXYZ second){
  return first.x == second.x && first.y == second.y && first.z == second.z;
}

/****************K POINTS & RADIUS POINTS*****************************/
void calculate_hole(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){

	srand (time (NULL));

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	kdtree.setInputCloud (cloud);

	pcl::PointXYZ search_point;

  int outer_k = 100;
  int inner_k = 10;
	// K nearest neighbor search
	std::vector<int> k_search(outer_k);
	std::vector<float> squared_dist(outer_k);
  std::vector<int> inner_k_search(inner_k);
	std::vector<float> inner_squared_dist(inner_k);
  
  Node node_points[10];

  for(int j = 0; j < 10; j++){
    search_point = cloud->points[j];
    node_points[j] = Node(search_point);
    if ( kdtree.nearestKSearch (search_point, outer_k, k_search, squared_dist) > 0 ){
      for (int i = 0; i < k_search.size() && node_points[j].n_count < 10; i++){
        bool contains_point = false;
        pcl::PointXYZ neighbor = cloud->points[k_search[i]];
        if( kdtree.nearestKSearch(neighbor, inner_k , inner_k_search, inner_squared_dist) > 0 && !is_equal(search_point, neighbor)){
          for (int k = 0; k < inner_k_search.size(); k++){
            pcl::PointXYZ neigh_bound = cloud->points[inner_k_search[k]];
            if (is_equal(search_point,neigh_bound)){
              contains_point = true;
            } 
          }
        }
        if(contains_point){
          node_points[j].add_neighbor(neighbor);
        }
      }
      node_points[j].decrement_counter();
      float v1[3];
      float v2[3];
      float max = 0;
      float angle = 0;
      Node vertex = node_points[j];
      Node start = *vertex.neighbors[0];
      Node end = *vertex.neighbors[1];
      v1[0] = current.x - vertex.x;
      v1[1] = current.y - vertex.y;
      v1[2] = current.z - vertex.z;
      int angles[vertex.n_count];
      int a_index = 0;
      for(int i = 0; i < vertex.n_count-1; i++, next = *vertex.neighbors[i+1]){
        v2[0] = next.x - vertex.x;
        v2[1] = next.y - vertex.y;
        v2[2] = next.z - vertex.z;
        angle = angle_between_vectors(v1,v2);
        angles[a_index] = angle;
        a_index++;
        if(angle > max){
          max = angle;
        }
      }
      for(int i = 0; i < a_index-1; i ++){

      }
      vertex.prob = max;
      node_points[j] = vertex;
    }
    std::cout<<"\n\n"<<search_point<<"\n\n"<<std::endl;
    std::cout<<"prob"<<node_points[j].prob<<std::endl;
    node_points[j].print_neighbors();
  }
  

float p1[3] = {1,1,1}; //said point 1
float p2[3] = {2,2,2}; //said point 2 

//angle_between_vector(p1,p2);

float dist = sqrt(pow(p2[0]-p1[0],2.0) + pow(p2[1]-p1[1],2.0) + pow(p2[2]-p1[2],2.0));
/**********************************************************************/

}

//function needed for viewing pcl
void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    /*pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;*/
    
}

void visualize(){
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>); 
	pcl::io::loadPCDFile ("new_file.pcd", *cloud); 

	pcl::visualization::CloudViewer viewer("Cloud Viewer"); 

	//blocks until the cloud is actually rendered 
	viewer.showCloud(cloud);   
	//use the following functions to get access to the underlying more advanced/powerful 
	//PCLVisualizer 

	//This will only get called once 
	viewer.runOnVisualizationThreadOnce (viewerOneOff); 

	//This will get called once per visualization iteration 
	//viewer.runOnVisualizationThread (viewerPsycho); 
	while (!viewer.wasStopped ()) 
	{ 
	//you can also do cool processing here 
	//FIXME: Note that this is running in a separate thread from viewerPsycho 
	//and you should guard against race conditions yourself... 
	}
}


/*****************************MAIN************************************/
int main (int argc, char** argv)
{
  Node node;
  std::cout<<"node "<<std::endl;
	// Fetch point cloud filename in arguments | Works with PCD and PLY files
	std::vector<int> filenames;
	bool file_is_pcd = false;

	filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

	if (filenames.size () != 1)  {
		filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

		if (filenames.size () != 1) {
			std::cout << std::endl;
			std::cout << "Usage: " << argv[0] << "cloud_filename.[pcd][ply]" << std::endl;
			return -1;
		} else {
			file_is_pcd = true;
		}
	}

	// Load file | Works with PCD and PLY files
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());

	if (file_is_pcd) {
		if (pcl::io::loadPCDFile (argv[filenames[0]], *cloud) < 0)  {
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			std::cout << std::endl;
			std::cout << "Usage: " << argv[0] << "cloud_filename.[pcd][ply]" << std::endl;
			return -1;
		}
	} else {
		if (pcl::io::loadPLYFile (argv[filenames[0]], *cloud) < 0)  {
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			std::cout << std::endl;
			std::cout << "Usage: " << argv[0] << "cloud_filename.[pcd][ply]" << std::endl;	
			return -1;
		}
	}
	showHelp();
	while(true){
		std::cout<<">> ";
		std::string line;
		getline(std::cin, line);
		// Show help
		if (line == "_quit"){
			break;
		}
		else if (line == "_help") {
			showHelp ();
		}
		else if (line == "_tree") {
			kd_tree (cloud);
			std::cin.ignore(INT_MAX, '\n');
		}
		else if (line == "_points") {
			points (cloud);
		}
		else if (line == "_normals") {
			normals (cloud);
		}
		else if (line == "_visualize") {
			visualize();
		}
		else if (line == "_holes") {
			calculate_hole (cloud);
			std::cin.ignore(INT_MAX, '\n');
		}
		else{
			std::cout <<"problem -> "<<line<< " *Error: Command not recognized enter '_help' to view help menu" << std::endl;
		}
	}
	return 0;
}

/*********************************************************************************************/
