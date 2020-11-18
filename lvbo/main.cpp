#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char** argv)
{
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	// 填入点云数据
	pcl::io::loadPCDFile("map.pcd", *cloud);
	if(!cloud) std::cout<<"no input"<<std::endl;
	std::cout << "Cloud before filtering:" << std::endl;
	std::cout << *cloud << std::endl;
	// 创建滤波器对象
	//pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;  //创建滤波器
	//outrem.setInputCloud(cloud);
	//outrem.setRadiusSearch(1.5);
	//outrem.setMinNeighborsInRadius(2);
	//outrem.filter(*cloud_filtered);
        
	//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> so;//创建滤波器对象
	//so.setInputCloud(cloud);
	//so.setMeanK(50);
	//so.setStddevMulThresh(1.0);
	//so.filter(*cloud_filtered);

	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.5f,0.5f,0.5f);
	sor.filter(*cloud_filtered);

        pcl::io::savePCDFileBinary("map_filter.pcd", *cloud_filtered );
	std::cout << "Cloud after filtering: " << std::endl;
	std::cout << *cloud_filtered << std::endl;
	return 0;
}
