#include <pcl/io/pcd_io.h>
#include <pcl/features/esf.h>

int main(int argc, char** argv)
{
	// Cloud for storing the object.
	pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the ESF descriptor.
	pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor(new pcl::PointCloud<pcl::ESFSignature640>);

	// Note: you should have performed preprocessing to cluster out the object
	// from the cloud, and save it to this individual file.

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *object) != 0)
	{
		return -1;
	}

	// ESF estimation object.
	pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
	esf.setInputCloud(object);

	esf.compute(*descriptor);
}
