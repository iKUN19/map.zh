#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>

using namespace std;

int applyGamma(int value, float gamma) {
  float normalized = value / 255.0f;
  float corrected = pow(normalized, gamma);  // gamma < 1增强对比度
  return std::min(255, std::max(0, (int)(corrected * 255)));
}

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
	pcl::PointCloud<pcl::PointXYZRGB> cloud;

  pcl::io::loadPCDFile<pcl::PointXYZRGB>(filePath, cloud);

  for (int i = 0; i < cloud.points.size(); ++i) {
    float rgb = *reinterpret_cast<float*>(&cloud.points[i].rgb);
    int r = cloud.points[i].r;
    int g = cloud.points[i].g;
    int b = cloud.points[i].b;
    r = applyGamma(r, 0.8f); // Gamma correction
    g = applyGamma(g, 0.8f);
    b = applyGamma(b, 0.8f);
    cloud.points[i].rgb = (r << 16) | (g << 8) | b; // Pack RGB back
  }
    // Allows output file to be set:
    pcl::io::savePCDFileBinary(argv[2], cloud);


	return (0);
}
