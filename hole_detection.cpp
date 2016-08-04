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
float angle_between_vectors (float *nu, float *nv)
{
	float l1, l2, angle, param ;
	l1 = sqrt(nu[0]*nu[0] + nu[1]*nu[1] + nu[2]*nu[2]) ;
	l2 = sqrt(nv[0]*nv[0] + nv[1]*nv[1] + nv[2]*nv[2]) ;
	float dot ;
	dot = nu[0]*nv[0] + nu[1]*nv[1] + nu[2]*nv[2] ;
	param = dot/(l1*l2) ;
	//if (param < 0)
	//param = -(param) ;
	angle = std::acos(param) ;
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
    
		if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
		{
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


		if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
		{
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
				std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
					<< ", " << cloud->points[ pointIdxRadiusSearch[i] ].y 
					<< ", " << cloud->points[ pointIdxRadiusSearch[i] ].z 
					<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
		}
    std::cout << "points: " << pointIdxRadiusSearch.size() << std::endl;
	}
}


/****************K POINTS & RADIUS POINTS*****************************/
void calculate_hole(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){

	srand (time (NULL));

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	kdtree.setInputCloud (cloud);

	pcl::PointXYZ searchPoint;

	std::cout<< "Enter a K value: " <<std::endl;
	int K = 0;
	std::cin >> K;

	// K nearest neighbor search
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

  for(int j = 0; j < cloud->points.size(); j++){
    searchPoint = cloud->points[j];
    std::cout<<"\n\n"<<searchPoint<<"\n\n"<<std::endl;
    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
        std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
          << ", " << cloud->points[ pointIdxNKNSearch[i] ].y 
          << ", " << cloud->points[ pointIdxNKNSearch[i] ].z 
          << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl; 
      }
    }
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

class Node{
  public:
    int x;
    int y;
    int z;
    int counter;
    Node();
    Node *neighbors[10];
    void setXYZ(int c1, int c2, int c3);
    void add_neighbor(Node neighbor);
};

Node::Node(){
  counter = 0;
}

void Node::setXYZ(int c1, int c2, int c3){
  x = c1;
  y = c2;
  z = c3;
}

void Node::add_neighbor(Node neighbor){
  std::cout<<neighbors[counter]<<std::endl;
  *neighbors[counter++] = neighbor;
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
