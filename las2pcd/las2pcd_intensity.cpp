#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <liblas/liblas.hpp>
#include <pcl/common/io.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>

using namespace std;

int main(int argc, char *argv[])
{
	if(argc < 3 || argc > 6 || argc == 4 || argc == 5)
	{
		std::cerr << "ERROR: number of arguements error." << std::endl;
		std::cerr << "Please input the right parameters!" << std::endl;
		return 0;
	}
	string filePath;

	// getline(cin, filePath);
	// Edited to make fully command-line:
	filePath = argv[1];

	std::cerr << "INFO : Loading : " << filePath << std::endl;

	// instancing a new PCL pointcloud object
	pcl::PointCloud<pcl::PointXYZI> cloud;

	// Opening  the las file
	std::ifstream ifs(filePath.c_str(), std::ios::in | std::ios::binary);

	// Safeguard against opening failure
	if (ifs.fail())
	{
		std::cerr << "ERROR : Impossible to open the file : " << filePath << std::endl;
		return 1;
	}

	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs); // reading las file
	unsigned long int nbPoints = reader.GetHeader().GetPointRecordsCount();

	// Fill in the cloud data
	cloud.width = nbPoints; // This means that the point cloud is "unorganized"
	cloud.height = 1;				// (i.e. not a depth map)
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	cout << "INFO : " << cloud.points.size() << " points detected in " << filePath << endl;

	int i = 0;					 // counter
	uint16_t r1, g1, b1; // RGB variables for .las (16-bit coded)
	int r2, g2, b2;			 // RGB variables for converted values (see below)
	uint32_t rgb;				 // "packed" RGB value for .pcd
	double x0, y0, z0;
	reader.ReadNextPoint();
	if(argc == 3)
	{
		std::cout << "got input file path, output file path. will use the coordinates of first point of pointcloud as the origin." << std::endl;
		x0 = (reader.GetPoint().GetX());
		y0 = (reader.GetPoint().GetY());
		z0 = (reader.GetPoint().GetZ());
	}else if (argc == 6)
	{
		std::cout << "got input file path, output file path, x0, y0, z0. will use x0, y0, z0 as the origin." << std::endl;
		x0 = atof(argv[3]);
		y0 = atof(argv[4]);
		z0 = atof(argv[5]);
	}

	std::cout << "the origin coordinate is x0 = " << std::fixed << std::setprecision(8) << x0 << ", y0 = " << y0 << ", z0 = " << z0 << std::endl;

	while (reader.ReadNextPoint())
	{
		// get XYZ information
		cloud.points[i].x = (reader.GetPoint().GetX() - x0);
		cloud.points[i].y = (reader.GetPoint().GetY() - y0);
		cloud.points[i].z = (reader.GetPoint().GetZ() - z0);

		// get intensity information
		cloud.points[i].intensity = (reader.GetPoint().GetIntensity());

		i++; // ...moving on
	}

	// Allows output file to be set:
	pcl::io::savePCDFileBinary(argv[2], cloud);

	std::cerr << "Saved " << cloud.points.size() << " data points to pointcloud.pcd." << std::endl;

	return (0);
}
