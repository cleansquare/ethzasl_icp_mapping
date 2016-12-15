#include <fstream>
#include <memory>

#include "ros/ros.h"
#include "ros/console.h"

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/IO.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"


using namespace std;
using namespace PointMatcherSupport;

class PublishVTK
{
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	typedef PointMatcherIO<float> PMIO;

	ros::NodeHandle& n;

	// Parameters
	const string mapFrame;
	const string cloudTopic;
	const string csvListFiles;
	const string dataDirectory;
	const string inputVtkAsciiFile;
	const string outputCsvFile;
	const double publishRate;
	const bool pauseEachMsg;

	ros::Publisher cloudPub;
	PMIO::FileInfoVector list;
	unsigned currentId;

public:
	PublishVTK(ros::NodeHandle& n);
	void convertAndPublish();
};

PublishVTK::PublishVTK(ros::NodeHandle& n):
	n(n),
	mapFrame(getParam<string>("mapFrameId", "/map")),
	cloudTopic(getParam<string>("cloudTopic", "/point_cloud")),
	csvListFiles(getParam<string>("csvListFiles", "")),
	dataDirectory(getParam<string>("dataDirectory", "")),
	inputVtkAsciiFile(getParam<string>("inputVtkAsciiFile", "")),
	outputCsvFile(getParam<string>("outputCsvFile", "")),
	publishRate(getParam<double>("publishRate", 1.0)),
	pauseEachMsg(getParam<bool>("pauseEachMsg", false))
{
	// ROS initialization
	cloudPub = n.advertise<sensor_msgs::PointCloud2>(cloudTopic, 1);

	if(csvListFiles != "")
	{
		list = PMIO::FileInfoVector(csvListFiles, dataDirectory);
	}
	currentId = 0;
}

void PublishVTK::convertAndPublish()
{
    cout << "PublishVTK::convertAndPublish started..." << endl;

	DP cloud;
	if(inputVtkAsciiFile != "") {
		cloud = DP::load(inputVtkAsciiFile);
        cout << "Loaded data from " << inputVtkAsciiFile << endl;
    }
	else
	{

		DP cloud;
		if(singleFile != "")
			cloud = DP::load(singleFile);
		else
		{
			if(csvListFiles != "")
			{
				if(pauseEachMsg)
				{
					cout << endl << "Press <ENTER> to continue or <CTRL-c>  to exit" << endl;
					cin.clear();
					cin.ignore(INT_MAX, '\n');
				}

				ROS_INFO_STREAM("Publishing file [" << currentId << "/" << list.size() << "]: " << list[currentId].readingFileName);
				cloud = DP::load(list[currentId].readingFileName);
				currentId++;
				if(currentId >= list.size())
				{
					cout << endl << "Press <ENTER> to restart or <CTRL-c>  to exit" << endl;
					cin.clear();
					cin.ignore(INT_MAX, '\n');
					currentId = 0;
				}
			}
		}
	}

	if(outputCsvFile != "") {
        stringstream nameStream;
        cout << "Writing output to " << outputCsvFile << endl;
        cloud.save(outputCsvFile);

		if(singleFile != "" || csvListFiles != ""){
	      stringstream nameStream;
	      nameStream << cloudTopic << "_" << "bla" << ".csv";
	      cloud.save(nameStream.str());

				cloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(cloud, mapFrame, ros::Time::now()));
	    }
		else
		{
			ROS_ERROR_STREAM("No files found");
			abort();
		}
	}
}


// Main function supporting the ExportVtk class
int main(int argc, char **argv)
{
	ros::init(argc, argv, "VtkToPointCloud_node");
	ros::NodeHandle n;
	PublishVTK pub(n);
	pub.convertAndPublish();
	
	return 0;
}
