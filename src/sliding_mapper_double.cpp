#include <fstream>

#include <boost/version.hpp>
#include <boost/thread.hpp>
#if BOOST_VERSION >= 104100
    #include <boost/thread/future.hpp>
#endif // BOOST_VERSION >=  104100

#include "ros/ros.h"
#include<ros/package.h>
#include "ros/console.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

#include "nabo/nabo.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"

#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include "eigen_conversions/eigen_msg.h"

// Services
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "map_msgs/SaveMap.h"
#include "ethzasl_icp_mapper/LoadMap.h"
#include "ethzasl_icp_mapper/CorrectPose.h"
#include "ethzasl_icp_mapper/SetMode.h"
#include "ethzasl_icp_mapper/GetMode.h"
#include "ethzasl_icp_mapper/GetBoundedMap.h" // FIXME: should that be moved to map_msgs?

using namespace std;
using namespace PointMatcherSupport;

class Mapper
{
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    typedef PM::Matches Matches;

    typedef typename Nabo::NearestNeighbourSearch<float> NNS;
    typedef typename NNS::SearchType NNSearchType;

    ros::NodeHandle& n;
    ros::NodeHandle& pn;

    // Subscribers
    ros::Subscriber odomSub;
    ros::Subscriber scanSub;
    ros::Subscriber cloudSub;

    // another velodyne, by yh for zhenhua project
    ros::Subscriber cloudSub_new;
    void gotCloud_new(const sensor_msgs::PointCloud2& cloudMsgIn);
    DP cloud_laser1;
    PM::TransformationParameters trans;
    tf::TransformListener tf_listener_calib;

    // Publishers
    ros::Publisher mapPub;
    ros::Publisher mapInSensorPub;
    ros::Publisher blindSpotCloudPub;
    ros::Publisher augmentPointCloudPub;
    ros::Publisher sensorOdomPub;
    ros::Publisher footPrintToPlaneOdomPub;

    // Time
    ros::Time mapCreationTime;
    ros::Time lastPoinCloudTime;

    // libpointmatcher
    PM::DataPointsFilters inputFilters;
    PM::DataPointsFilters mapPostFilters;
    PM::DataPointsFilters verticalCuboidFilter;
    PM::DataPoints *mapPointCloud;
    PM::DataPoints *mapPointCloudInSensorFrame;
    PM::DataPoints *newPointCloudInSensorFrame;
    PM::DataPoints *newPointCloudInbaseFootPrintFrame;
    PM::DataPoints *augmentPointCloud;
    PM::DataPoints *blindSpotPointCloud;
    PM::ICPSequence icp;
    unique_ptr<PM::Transformation> transformation;

    // multi-threading mapper
    #if BOOST_VERSION >= 104100
    typedef boost::packaged_task<PM::DataPoints*> MapBuildingTask;
    typedef boost::unique_future<PM::DataPoints*> MapBuildingFuture;
    boost::thread mapBuildingThread;
    MapBuildingTask mapBuildingTask;
    MapBuildingFuture mapBuildingFuture;
    bool mapBuildingInProgress;
    #endif // BOOST_VERSION >= 104100
    bool processingNewCloud;

    // Parameters
    bool publishMapTf;
    bool useConstMotionModel;
    bool localizing;
    bool mapping;
    bool odomReceived;
    int minReadingPointCount;
    int minMapPointCount;
    int inputQueueSize;
    double minOverlap;
    double maxOverlapToMerge;
    double tfRefreshPeriod;  //!< if set to zero, tf will be publish at the rate of the incoming point cloud messages
    string mapFrame;
    string planeOdomFrame;
    string baseFootPrintFrame;
//    string planeFootPrintFrame;
    string vtkFinalMapName; //!< name of the final vtk map

    const double robotMaximalHeight;
    const double robotChassisHeight;

    const double blindSpotDownlookAngleDetect;
    const double blindSpotDownlookAngleTrack;
    const double blindSpotTrackMinTranslate;

    const double heightFilterMin;
    const double heightFilterMax;
    const double heightFilterDistRatio;

    ofstream outSavePose;


    // map construction range
    const float sensorMaxRange;
    const float slidingMapMaxRange;

    // Parameters for dynamic filtering
    const float priorStatic; //!< ratio. Prior to be static when a new point is added
    const float priorDyn; //!< ratio. Prior to be dynamic when a new point is added
    const float maxAngle; //!< in rad. Openning angle of a laser beam
    const float eps_a; //!< ratio. Error proportional to the laser distance
    const float eps_d; //!< in meter. Fix error on the laser distance
    const float alpha; //!< ratio. Propability of staying static given that the point was dynamic
    const float beta; //!< ratio. Propability of staying dynamic given that the point was static
    const float maxDyn; //!< ratio. Threshold for which a point will stay dynamic
    const float dynT; //!< in meter. Distance at which a new point will be added in the global map.

    // Parameters for new data addition
    const float maxDistNewPoint; //!< in meter. Distance at which a new point will be added in the global map.

    PM::TransformationParameters TSensorRelative;
    PM::TransformationParameters TSensorToMap;
    PM::TransformationParameters lastTSensorToMap;
    PM::TransformationParameters TFootPrintToMap;
    PM::TransformationParameters lastTFootPrintToMap;
    PM::TransformationParameters TFootPrintToPlaneOdom;
    PM::TransformationParameters lastTFootPrintToPlaneOdom;

    PM::TransformationParameters TOdomPose;
    PM::TransformationParameters lastTOdomPose;

    boost::thread publishThread;
    boost::mutex publishLock;
    ros::Time publishStamp;

    tf::TransformListener tfListener;
    tf::TransformBroadcaster tfBroadcaster;

    const float eps;

public:
    Mapper(ros::NodeHandle& n, ros::NodeHandle& pn);
    ~Mapper();

protected:
    void gotOdom(const nav_msgs::Odometry& odomMsgIn);
    void gotScan(const sensor_msgs::LaserScan& scanMsgIn);
    void gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn);
    void processCloud(unique_ptr<DP> cloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq);
    void processNewMapIfAvailable();
    void setMap(DP* newPointCloud);
    DP* updateMap(DP* newPointCloud, const PM::TransformationParameters Ticp, bool updateExisting);
    void waitForMapBuildingCompleted();
    void dynamicProbabilityUpdate(DP* newPointCloud);
    void blindSpotCloudUpdate(DP* newPointCloud, const PM::TransformationParameters TSensorRelative, bool initialized);
    void groundCloudFilter(DP* newPointCloud);
    void cloudHeightTrim(DP* newPointCloud);
    void publishLoop(double publishPeriod);
    void publishTransform();
};

Mapper::Mapper(ros::NodeHandle& n, ros::NodeHandle& pn):
    n(n),
    pn(pn),
    mapPointCloud(0),
    mapPointCloudInSensorFrame(0),
    newPointCloudInSensorFrame(0),
    newPointCloudInbaseFootPrintFrame(0),
    blindSpotPointCloud(0),
    augmentPointCloud(0),
    transformation(PM::get().REG(Transformation).create("RigidTransformation")),
    #if BOOST_VERSION >= 104100
    mapBuildingInProgress(false),
    #endif // BOOST_VERSION >= 104100
    processingNewCloud(false),
    publishMapTf(getParam<bool>("publishMapTF", true)),
    useConstMotionModel(getParam<bool>("useConstMotionModel", false)),
    localizing(getParam<bool>("localizing", true)),
    mapping(getParam<bool>("mapping", true)),
    minReadingPointCount(getParam<int>("minReadingPointCount", 2000)),
    minMapPointCount(getParam<int>("minMapPointCount", 500)),
    inputQueueSize(getParam<int>("inputQueueSize", 10)),
    minOverlap(getParam<double>("minOverlap", 0.5)),
    maxOverlapToMerge(getParam<double>("maxOverlapToMerge", 0.9)),
    tfRefreshPeriod(getParam<double>("tfRefreshPeriod", 0.01)),
    mapFrame(getParam<string>("map_frame", "map")),
    planeOdomFrame(getParam<string>("plane_odom_frame", "plane_odom")),
    baseFootPrintFrame(getParam<string>("base_footprint_frame", "base_footprint")),

    vtkFinalMapName(getParam<string>("vtkFinalMapName", "finalMap.vtk")),
    robotMaximalHeight(getParam<double>("robotMaximalHeight", 1.5)),
    robotChassisHeight(getParam<double>("robotChassisHeight", 0.1)),
    blindSpotDownlookAngleDetect(getParam<double>("blindSpotDownlookAngleDetect", 14.5)),
    blindSpotDownlookAngleTrack(getParam<double>("blindSpotDownlookAngleTrack", 15.5)),
    blindSpotTrackMinTranslate(getParam<double>("blindSpotTrackMinTranslate", 0.1)),
    heightFilterMin(getParam<double>("heightFilterMin", 0.1)),
    heightFilterMax(getParam<double>("heightFilterMax", 0.5)),
    heightFilterDistRatio(getParam<double>("heightFilterDistRatio", 0.005)),
    priorStatic(getParam<double>("priorStatic", 0.5)),
    priorDyn(getParam<double>("priorDyn", 0.5)),
    maxAngle(getParam<double>("maxAngle", 0.02)),
    eps_a(getParam<double>("eps_a", 0.05)),
    eps_d(getParam<double>("eps_d", 0.02)),
    alpha(getParam<double>("alpha", 0.99)),
    beta(getParam<double>("beta", 0.99)),
    maxDyn(getParam<double>("maxDyn", 0.95)),
    dynT(getParam<double>("dynThreshold", 0.95)),
    sensorMaxRange(getParam<double>("sensorMaxRange", 80)),
    slidingMapMaxRange(getParam<double>("slidingMapMaxRange", 85)),
    maxDistNewPoint(pow(getParam<double>("maxDistNewPoint", 0.1),2)),
    TFootPrintToMap(PM::TransformationParameters::Identity(4, 4)),
    TFootPrintToPlaneOdom(PM::TransformationParameters::Identity(4, 4)),
    publishStamp(ros::Time::now()),
    tfListener(ros::Duration(30)),
    odomReceived(false),
    eps(0.0001)
{

    // Ensure proper states
    if(localizing == false)
        mapping = false;
    if(mapping == true)
        localizing = true;

    // set logger
    if (getParam<bool>("useROSLogger", false))
        PointMatcherSupport::setLogger(new PointMatcherSupport::ROSLogger);

    // load configs
    string configFileName;
    if (ros::param::get("~icpConfig", configFileName))
    {
        ifstream ifs(configFileName.c_str());
        if (ifs.good())
        {
            icp.loadFromYaml(ifs);
        }
        else
        {
            ROS_ERROR_STREAM("Cannot load ICP config from YAML file " << configFileName);
            icp.setDefault();
        }
    }
    else
    {
        ROS_INFO_STREAM("No ICP config file given, using default");
        icp.setDefault();
    }

    if (getParam<bool>("useROSLogger", false))
        PointMatcherSupport::setLogger(new PointMatcherSupport::ROSLogger);

    if (ros::param::get("~inputFiltersConfig", configFileName))
    {
        ifstream ifs(configFileName.c_str());
        if (ifs.good())
        {
            inputFilters = PM::DataPointsFilters(ifs);
        }
        else
        {
            ROS_ERROR_STREAM("Cannot load input filters config from YAML file " << configFileName);
        }
    }
    else
    {
        ROS_INFO_STREAM("No input filters config file given, not using these filters");
    }

    if (ros::param::get("~mapPostFiltersConfig", configFileName))
    {
        ifstream ifs(configFileName.c_str());
        if (ifs.good())
        {
            mapPostFilters = PM::DataPointsFilters(ifs);
        }
        else
        {
            ROS_ERROR_STREAM("Cannot load map post-filters config from YAML file " << configFileName);
        }
    }
    else
    {
        ROS_INFO_STREAM("No map post-filters config file given, not using these filters");
    }

    if (ros::param::get("~verticalCuboidFilterConfig", configFileName))
    {
        ifstream ifs(configFileName.c_str());
        if (ifs.good())
        {
            verticalCuboidFilter = PM::DataPointsFilters(ifs);
        }
        else
        {
            ROS_ERROR_STREAM("Cannot load verticalCuboidFilter config from YAML file " << configFileName);
        }
    }
    else
    {
        ROS_INFO_STREAM("No verticalCuboidFilter config file given, not using these filters");
    }

    // topics and services initialization
    if (getParam<bool>("subscribe_odom", true))
        odomSub = n.subscribe("odom_in", 1, &Mapper::gotOdom, this);

    if (getParam<bool>("subscribe_scan", true))
        scanSub = n.subscribe("scan", inputQueueSize, &Mapper::gotScan, this);

    if (getParam<bool>("subscribe_cloud", true))
    {
        cloudSub_new = n.subscribe("/velodyne1/velodyne_points", 1, &Mapper::gotCloud_new, this);
        cloudSub = n.subscribe("cloud_in", inputQueueSize, &Mapper::gotCloud, this);
    }

    mapPub = n.advertise<sensor_msgs::PointCloud2>("sliding_map", 1, true);
    mapInSensorPub = n.advertise<sensor_msgs::PointCloud2>("sliding_map_sensorFrame", 1, true);
    blindSpotCloudPub = n.advertise<sensor_msgs::PointCloud2>("blindSpotCloud", 1, true);
    augmentPointCloudPub = n.advertise<sensor_msgs::PointCloud2>("augmentPointCloud", 1, true);
    sensorOdomPub = n.advertise<nav_msgs::Odometry>("laser_odom_3d", 1, true);
    footPrintToPlaneOdomPub = n.advertise<nav_msgs::Odometry>("laser_odom_2d", 1, true);


    // refreshing tf transform thread
    publishThread = boost::thread(boost::bind(&Mapper::publishLoop, this, tfRefreshPeriod));

}

Mapper::~Mapper()
{
    #if BOOST_VERSION >= 104100
    // wait for map-building thread
    if (mapBuildingInProgress)
    {
        mapBuildingFuture.wait();
        if (mapBuildingFuture.has_value())
            delete mapBuildingFuture.get();
    }
    #endif // BOOST_VERSION >= 104100
    // wait for publish thread
    publishThread.join();
    // save point cloud
    if (mapPointCloud)
    {
        mapPointCloud->save(vtkFinalMapName);
        delete mapPointCloud;
    }
}

void Mapper::gotOdom(const nav_msgs::Odometry& odomMsgIn)
{
    TOdomPose = PointMatcher_ros::odomMsgToEigenMatrix<float>(odomMsgIn);
    odomReceived = true;
//    cout<<"odom received at time="<<odomMsgIn.header.stamp << "  and the tf matrix is"<<endl;
//    cout<<TOdomPose<<endl;
}

void Mapper::gotScan(const sensor_msgs::LaserScan& scanMsgIn)
{
    if(localizing)
    {
        const ros::Time endScanTime(scanMsgIn.header.stamp + ros::Duration(scanMsgIn.time_increment * (scanMsgIn.ranges.size() - 1)));
        unique_ptr<DP> cloud(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scanMsgIn, &tfListener, baseFootPrintFrame)));
        processCloud(move(cloud), scanMsgIn.header.frame_id, endScanTime, scanMsgIn.header.seq);
    }
}

void Mapper::gotCloud_new(const sensor_msgs::PointCloud2& cloudMsgIn)
{
     cloud_laser1 = DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));

      this->trans = PointMatcher_ros::eigenMatrixToDim<float>(
                 PointMatcher_ros::transformListenerToEigenMatrix<float>(
                 this->tf_listener_calib,
                 "laser1",
                 "laser2",
                 ros::Time(0)
             ), cloud_laser1.getHomogeneousDim());

     cloud_laser1 = transformation->compute(cloud_laser1, this->trans.inverse());
}

void Mapper::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{
    if(localizing)
    {
        if(cloud_laser1.features.cols() == 0)
        {
            return;
        }

        unique_ptr<DP> cloud(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn)));

        DP newCloud = this->cloud_laser1;
//        cout<<newcloud.features.rows()<<"   "<<newcloud.features.cols()<<endl;
//        cout<<cloud->features.rows()<<"   "<<cloud->features.cols()<<endl;

        // check
        if(newCloud.features.cols() != newCloud.descriptors.cols() ||
           cloud->features.cols() != cloud->descriptors.cols())
        {
            return;
        }

        newCloud.concatenate(*cloud);

        unique_ptr<DP> cloud_co(new DP(newCloud));

        processCloud(move(cloud_co), cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp, cloudMsgIn.header.seq);
    }
}

struct BoolSetter
{
public:
    bool toSetValue;
    BoolSetter(bool& target, bool toSetValue):
        toSetValue(toSetValue),
        target(target)
    {}
    ~BoolSetter()
    {
        target = toSetValue;
    }
protected:
    bool& target;
};

void Mapper::processCloud(unique_ptr<DP> newPointCloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq)
{
    cout<<"------------------------------------------------------"<<endl;
    cout<<"Time:    " <<std::fixed<<stamp.toSec()<<endl;

//    newPointCloud->save("/home/yh/zhenhua_cloud.vtk");

    timer timeProcess; // Print how long take the algo

    processingNewCloud = true;
    BoolSetter stopProcessingSetter(processingNewCloud, false);

    // if the future has completed, use the new map
    processNewMapIfAvailable();
//    cerr << "received new map" << endl;

    // IMPORTANT:  We need to receive the point clouds in local coordinates (scanner or robot)
    timer tProcess;

    // Convert point cloud
    const size_t goodCount(newPointCloud->features.cols());
    if (goodCount == 0)
    {
        ROS_ERROR("I found no good points in the cloud");
        return;
    }
    // Dimension of the point cloud, important since we handle 2D and 3D
    const int dimp1(newPointCloud->features.rows());

    // Fetch transformation from scanner to odom
    // Note: we don't need to wait for transform. It is already called in transformListenerToEigenMatrix()
    PM::TransformationParameters TFootPrintToSensor;
    try
    {
        TFootPrintToSensor = PointMatcher_ros::eigenMatrixToDim<float>(
                PointMatcher_ros::transformListenerToEigenMatrix<float>(
                tfListener,
                scannerFrame,
                baseFootPrintFrame,
                stamp
            ), dimp1);
    }
    catch(tf::ExtrapolationException e)
    {
        ROS_ERROR_STREAM("Extrapolation Exception. stamp = "<< stamp << " now = " << ros::Time::now() << " delta = " << ros::Time::now() - stamp << endl << e.what() );
        return;
    }
    PM::TransformationParameters TSensorToFootPrint = TFootPrintToSensor.inverse();

    // filters
    // // first stage filter
//    ROS_INFO("Processing new point cloud");
    {
        // Apply filters to incoming cloud, in scanner coordinates
        timer timeProcess_0;
        inputFilters.apply(*newPointCloud);
        cout<<"filter:  "<<timeProcess_0.elapsed()<<endl;
    }


    // Ensure a minimum amount of point after filtering
    const int ptsCount = newPointCloud->features.cols();
    if(ptsCount < minReadingPointCount)
    {
        ROS_ERROR_STREAM("Not enough points in newPointCloud: only " << ptsCount << " pts.");
        return;
    }

    if(newPointCloud->descriptorExists("probabilityStatic") == false)
        newPointCloud->addDescriptor("probabilityStatic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorStatic));

    if(newPointCloud->descriptorExists("probabilityDynamic") == false)
        newPointCloud->addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorDyn));

    if(newPointCloud->descriptorExists("dynamic_ratio") == false)
        newPointCloud->addDescriptor("dynamic_ratio", PM::Matrix::Zero(1, newPointCloud->features.cols()));

    if(newPointCloudInSensorFrame)
        delete newPointCloudInSensorFrame;
    newPointCloudInSensorFrame = new DP(newPointCloud->features,newPointCloud->featureLabels
                                     ,newPointCloud->descriptors, newPointCloud->descriptorLabels);

    if(newPointCloudInbaseFootPrintFrame)
        delete newPointCloudInbaseFootPrintFrame;
    newPointCloudInbaseFootPrintFrame = new DP(newPointCloud->features,newPointCloud->featureLabels
                                     ,newPointCloud->descriptors, newPointCloud->descriptorLabels);
    *newPointCloudInbaseFootPrintFrame = transformation->compute(*newPointCloud, TSensorToFootPrint);


    // Initialize the transformation to identity if empty
    if(!icp.hasMap())
    {
        // transformation initialization
        publishLock.lock();

        // odom transformation
        lastTFootPrintToMap = TFootPrintToMap = PM::TransformationParameters::Identity(dimp1, dimp1);
        // sensor transformation
        lastTSensorToMap = TSensorToMap = TSensorToFootPrint * TFootPrintToMap;

        TSensorRelative = PM::TransformationParameters::Identity(dimp1, dimp1);

        // set odom readings
        if(odomReceived)
            lastTOdomPose = TOdomPose;
        else
            lastTOdomPose = TOdomPose = PM::TransformationParameters::Identity(dimp1, dimp1);

        // plane odom transformation
        lastTFootPrintToPlaneOdom = TFootPrintToPlaneOdom = PM::TransformationParameters::Identity(dimp1, dimp1);
        publishLock.unlock();

        PM::TransformationParameters TIdentity = PM::TransformationParameters::Identity(dimp1, dimp1);
        // initialize blind spot cloud
        blindSpotCloudUpdate(newPointCloudInSensorFrame, TIdentity, false);

        // Initialize the map if empty
        ROS_INFO_STREAM("Creating an initial map");
        mapCreationTime = stamp;
        if(mapPointCloudInSensorFrame)
            delete mapPointCloudInSensorFrame;
        mapPointCloudInSensorFrame = new DP(newPointCloud->features,newPointCloud->featureLabels
                                            ,newPointCloud->descriptors, newPointCloud->descriptorLabels);
        setMap(updateMap(newPointCloud.release(), TSensorToMap, false));
        // we must not delete newPointCloud because we just stored it in the mapPointCloud

        return;
    }

    // Check dimension
    if (newPointCloud->features.rows() != icp.getInternalMap().features.rows())
    {
        ROS_ERROR_STREAM("Dimensionality missmatch: incoming cloud is " << newPointCloud->features.rows()-1 << " while map is " << icp.getInternalMap().features.rows()-1);
        return;
    }
    try
    {
        PM::TransformationParameters TOdomRelative = lastTOdomPose.inverse() * TOdomPose;
        lastTOdomPose = TOdomPose;
        PM::TransformationParameters TSensorReckon = TFootPrintToSensor * TOdomRelative * TSensorToFootPrint;
        PM::TransformationParameters TSensorToMapReckon = lastTSensorToMap * TSensorReckon;


        timer timeProcess_1;
        TSensorToMap = icp(*newPointCloud, TSensorToMapReckon);
        cout<<"icp: "<<timeProcess_1.elapsed()<<endl;

        ROS_DEBUG_STREAM("Ticp:\n" << TSensorToMap);

        TSensorRelative = lastTSensorToMap.inverse() * TSensorToMap;

        // Transform the last point cloud in new point cloud coordinates
        if(mapPointCloudInSensorFrame)
            delete mapPointCloudInSensorFrame;
        mapPointCloudInSensorFrame =  new DP(mapPointCloud->features,mapPointCloud->featureLabels
                                            ,mapPointCloud->descriptors, mapPointCloud->descriptorLabels);
        *mapPointCloudInSensorFrame = transformation->compute(*mapPointCloudInSensorFrame, TSensorToMap.inverse());

        // Ensure minimum overlap between scans
        const double estimatedOverlap = icp.errorMinimizer->getOverlap();
//        ROS_INFO_STREAM("Overlap: " << estimatedOverlap);
        if (estimatedOverlap < minOverlap)
        {
            ROS_ERROR_STREAM("Estimated overlap too small, ignoring ICP correction!");
            return;
        }

        // Compute tf
        publishStamp = stamp;
        publishLock.lock();
        TFootPrintToMap = TSensorToMap * TFootPrintToSensor;
        Eigen::Matrix3f FootPrintToMapRotation = TFootPrintToMap.block(0,0,3,3);
        Eigen::AngleAxisf FootPrintToMapAxisAngle(FootPrintToMapRotation);    // RotationMatrix to AxisAngle
        TFootPrintToMap.block(0,0,3,3) = FootPrintToMapAxisAngle.toRotationMatrix();  // AxisAngle      to RotationMatrix

        //cout<<"start to estimate plane footprint"<<endl;
        PM::TransformationParameters TFootPrintRelative = lastTFootPrintToMap.inverse() * TFootPrintToMap;
        PM::TransformationParameters planeFootPrintRelativeTF = PM::TransformationParameters::Identity(dimp1, dimp1);
        // Eular angle ZYX
        float yaw = atan2(TFootPrintRelative(1,0), TFootPrintRelative(0,0));
        planeFootPrintRelativeTF(0,0) = cos(yaw);
        planeFootPrintRelativeTF(0,1) = -sin(yaw);
        planeFootPrintRelativeTF(1,0) = sin(yaw);
        planeFootPrintRelativeTF(1,1) = cos(yaw);
        planeFootPrintRelativeTF(0,3) = TFootPrintRelative(0,3);
        planeFootPrintRelativeTF(1,3) = TFootPrintRelative(1,3);
        TFootPrintToPlaneOdom = lastTFootPrintToPlaneOdom * planeFootPrintRelativeTF;
        // normalization
        yaw = atan2(TFootPrintToPlaneOdom(1,0), TFootPrintToPlaneOdom(0,0));
        TFootPrintToPlaneOdom(0,0) = cos(yaw);
        TFootPrintToPlaneOdom(0,1) = -sin(yaw);
        TFootPrintToPlaneOdom(1,0) = sin(yaw);
        TFootPrintToPlaneOdom(1,1) = cos(yaw);

        timer timeProcess_2;

//        dynamicProbabilityUpdate(newPointCloudInSensorFrame);
//        blindSpotCloudUpdate(newPointCloudInSensorFrame, TSensorRelative, true);
//        // // blind augment cloud
//        if(augmentPointCloud)
//            delete augmentPointCloud;
//        augmentPointCloud = new DP(blindSpotPointCloud->features, blindSpotPointCloud->featureLabels
//                                    ,blindSpotPointCloud->descriptors, blindSpotPointCloud->descriptorLabels);
//        // Transfrom the point cloud to odom frame and apply filter
//        *augmentPointCloud = transformation->compute(*blindSpotPointCloud, TSensorToFootPrint);
//        // apply density based filter on the blind spot cloud
//        verticalCuboidFilter.apply(*augmentPointCloud);
//        groundCloudFilter(augmentPointCloud); // find obstacle in odom frame from blind point cloud
//        // sensor cloud height trim
//        cloudHeightTrim(newPointCloudInbaseFootPrintFrame);
//        verticalCuboidFilter.apply(*newPointCloudInbaseFootPrintFrame);
//        // combine interested blind spot cloud and height filtered sensor cloud
//        augmentPointCloud->concatenate(*newPointCloudInbaseFootPrintFrame);


        PM::TransformationParameters TPlaneOdomToMap = TFootPrintToMap * TFootPrintToPlaneOdom.inverse();
        Eigen::Matrix3f PlaneOdomToMapRotation = TPlaneOdomToMap.block(0,0,3,3);
        Eigen::AngleAxisf PlaneOdomToMapAxisAngle(PlaneOdomToMapRotation);    // RotationMatrix to AxisAngle
        TPlaneOdomToMap.block(0,0,3,3) = PlaneOdomToMapAxisAngle.toRotationMatrix();  // AxisAngle      to RotationMatrix

        cout<<"map: "<<timeProcess_2.elapsed()<<endl;

        // Publish tf
        if(publishMapTf == true)
        {
//            tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TFootPrintToMap, mapFrame, baseFootPrintFrame, stamp));
//            cout<<TPlaneOdomToMap<<endl;
            tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TPlaneOdomToMap, mapFrame, planeOdomFrame, stamp));
            tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TFootPrintToPlaneOdom, planeOdomFrame, baseFootPrintFrame, stamp));

//                outSavePose.open("/home/yh/sliding.txt", ios::out|ios::ate|ios::app);
//                outSavePose << stamp << "    "
//                            << TFootPrintToMap(0,3) << "    "
//                        << TFootPrintToMap(1,3)<< "   "
//                        << atan2(TFootPrintToMap(1,0), TFootPrintToMap(0,0)) <<endl;
//                outSavePose.close();

        }
        publishLock.unlock();
        processingNewCloud = false;

        //ROS_INFO_STREAM("TFootPrintToMap:\n" << TFootPrintToMap);
        // Publish sensor to map (map is initialized on baseFootPrintFrame)
        nav_msgs::Odometry sensorToMapMsg = PointMatcher_ros::eigenMatrixToOdomMsg<float>(TSensorToMap, mapFrame, stamp);
        sensorToMapMsg.header.frame_id = mapFrame;
        sensorToMapMsg.child_frame_id  = scannerFrame;

        nav_msgs::Odometry footPrintToPlaneOdomMsg = PointMatcher_ros::eigenMatrixToOdomMsg<float>(TFootPrintToPlaneOdom, mapFrame, stamp);
        footPrintToPlaneOdomMsg.header.frame_id = planeOdomFrame;
        footPrintToPlaneOdomMsg.child_frame_id  = baseFootPrintFrame;

        nav_msgs::Odometry footPrintToMapMsg = PointMatcher_ros::eigenMatrixToOdomMsg<float>(TFootPrintToMap, mapFrame, stamp);
        footPrintToPlaneOdomMsg.header.frame_id = mapFrame;
        footPrintToPlaneOdomMsg.child_frame_id  = baseFootPrintFrame;

        if (sensorOdomPub.getNumSubscribers())
            sensorOdomPub.publish(sensorToMapMsg);

        // Publish odom to map (map is initialized on baseFootPrintFrame)
        if (footPrintToPlaneOdomPub.getNumSubscribers())
            footPrintToPlaneOdomPub.publish(footPrintToPlaneOdomMsg);

        // replace for zhenhua project
//        if (footPrintToPlaneOdomPub.getNumSubscribers())
//            footPrintToPlaneOdomPub.publish(footPrintToMapMsg);

        // check if news points should be added to the map
        if (
            mapping &&
            ((estimatedOverlap < maxOverlapToMerge) || (icp.getInternalMap().features.cols() < minMapPointCount)) &&
            #if BOOST_VERSION >= 104100
            (!mapBuildingInProgress)
            #else // BOOST_VERSION >= 104100
            true
            #endif // BOOST_VERSION >= 104100
        )
        {
            // make sure we process the last available map
            mapCreationTime = stamp;
            #if BOOST_VERSION >= 104100
            // ROS_INFO("Adding new points to the map in background");
            mapBuildingTask = MapBuildingTask(boost::bind(&Mapper::updateMap, this, newPointCloud.release(), TSensorToMap, true));
            mapBuildingFuture = mapBuildingTask.get_future();
            mapBuildingThread = boost::thread(boost::move(boost::ref(mapBuildingTask)));
            mapBuildingInProgress = true;
            #else // BOOST_VERSION >= 104100
            ROS_INFO("Adding new points to the map");
            setMap(updateMap( newPointCloud.release(), TSensorToMap, true));
            #endif // BOOST_VERSION >=
        }
    }
    catch (PM::ConvergenceError error)
    {
        ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
        return;
    }

    if (blindSpotCloudPub.getNumSubscribers())
        blindSpotCloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*blindSpotPointCloud, scannerFrame, stamp));

    if (augmentPointCloudPub.getNumSubscribers())
        augmentPointCloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*augmentPointCloud, baseFootPrintFrame, stamp));

    if (mapInSensorPub.getNumSubscribers())
        mapInSensorPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloudInSensorFrame, scannerFrame, stamp));

    lastPoinCloudTime   = stamp;
    lastTFootPrintToMap = TFootPrintToMap;
    lastTSensorToMap    = TSensorToMap;
    lastTFootPrintToPlaneOdom = TFootPrintToPlaneOdom;

    cout<<"time cost is "<< timeProcess.elapsed() << endl;
}

void Mapper::processNewMapIfAvailable()
{
    #if BOOST_VERSION >= 104100
    if (mapBuildingInProgress && mapBuildingFuture.has_value())
    {
//        ROS_INFO_STREAM("New map available");
        setMap(mapBuildingFuture.get());
        mapBuildingInProgress = false;
    }
    #endif // BOOST_VERSION >= 104100
}

void Mapper::setMap(DP* newPointCloud)
{
    // delete old map
    if (mapPointCloud)
        delete mapPointCloud;

    // set new map
    mapPointCloud = newPointCloud;
    icp.setMap(*mapPointCloud);

    if (mapPub.getNumSubscribers())
       mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud, mapFrame, mapCreationTime));
}

void Mapper::dynamicProbabilityUpdate(DP* newPointCloud)
{
    const int newPointsCount(newPointCloud->features.cols());
    const int mapPointsCount(mapPointCloud->features.cols());

    // Build a range image of the reading point cloud (Senor Frame)
    PM::Matrix radius_newPts = newPointCloud->features.topRows(3).colwise().norm();
    PM::Matrix angles_newPts(2, newPointsCount); // 0=inclination, 1=azimuth
    // initializeing angular kd tree of new point cloud
    for(int i=0; i<newPointsCount; i++)
    {
        const float ratio = newPointCloud->features(2,i)/radius_newPts(0,i);
        angles_newPts(0,i) = acos(ratio);
        angles_newPts(1,i) = atan2(newPointCloud->features(1,i), newPointCloud->features(0,i));
    }
    std::shared_ptr<NNS> featureNNS;
    featureNNS.reset(NNS::create(angles_newPts));

    PM::Matrix radius_mapPts = mapPointCloudInSensorFrame->features.topRows(3).colwise().norm();
    PM::Matrix angles_mapPts(2, mapPointsCount); // 0=inclination, 1=azimuth
    // No atan in Eigen, so we are for to loop through it...
    for(int i=0; i<mapPointsCount; i++)
    {
        const float ratio = mapPointCloudInSensorFrame->features(2,i)/radius_mapPts(0,i);
        angles_mapPts(0,i) = acos(ratio);
        angles_mapPts(1,i) = atan2(mapPointCloudInSensorFrame->features(1,i), mapPointCloudInSensorFrame->features(0,i));
    }
    // Look for NN in spherical coordinates
    Matches::Dists dists(1,mapPointsCount);
    Matches::Ids   ids(1,mapPointsCount);

    featureNNS->knn(angles_mapPts, ids, dists, 1, 0, NNS::ALLOW_SELF_MATCH, maxAngle);

    DP::View viewOnnormals_Map        = mapPointCloudInSensorFrame->getDescriptorViewByName("normals");
    DP::View viewOnProbabilityStatic  = mapPointCloudInSensorFrame->getDescriptorViewByName("probabilityStatic");
    DP::View viewOnProbabilityDynamic = mapPointCloudInSensorFrame->getDescriptorViewByName("probabilityDynamic");
    DP::View viewOnDynamicRatio       = mapPointCloudInSensorFrame->getDescriptorViewByName("dynamic_ratio");

    for(int i=0; i < mapPointsCount; i++)
    {
        if(dists(i) != numeric_limits<float>::infinity())
        {
            // in local coordinates
            const Eigen::Vector3f newPt   = newPointCloud->features.col(ids(0,i)).head(3);
            const Eigen::Vector3f mapPt   = mapPointCloudInSensorFrame->features.col(i).head(3);
            const Eigen::Vector3f mapPt_n = mapPt.normalized();
            const float delta = (newPt - mapPt).norm();
            const float d_max = eps_a * newPt.norm();

            const Eigen::Vector3f normal_map = viewOnnormals_Map.col(i);

            // Weight for dynamic elements
            const float w_v  = eps + (1 - eps)*fabs(normal_map.dot(mapPt_n));
            //const float w_d1 = 1 + eps - acos(newPt.normalized().dot(mapPt_n))/maxAngle;
            const float w_d1 = eps + (1 - eps)*(sqrt(dists(i))/maxAngle);


            const float offset = delta - eps_d;
            float w_d2 = 1;
            if(delta < eps_d || mapPt.norm() > newPt.norm())
            {
                w_d2 = eps;
            }
            else
            {
                if (offset < d_max)
                {
                    w_d2 = eps + (1 - eps )*offset/d_max;
                }
            }

            float w_p2 = eps;
            if(delta < eps_d)
            {
                w_p2 = 1;
            }
            else
            {
                if(offset < d_max)
                {
                    w_p2 = eps + (1 - eps)*(1 - offset/d_max);
                }
            }

            // We don't update point behind the reading
            if((newPt.norm() + eps_d + d_max) >= mapPt.norm())
            {

                const float lastDyn = viewOnProbabilityDynamic(0,i);
                const float lastStatic = viewOnProbabilityStatic(0, i);

                const float c1 = (1 - (w_v*(1 - w_d1)));
                const float c2 = w_v*(1 - w_d1);

                // FIXME: this is a parameter
                const float maxDyn = 0.9; // ICRA 14
// 				const float maxDyn = 0.98; // ISER 14

                //Lock dynamic point to stay dynamic under a threshold
                if(lastDyn < maxDyn)
                {
                    viewOnProbabilityDynamic(0,i) = c1*lastDyn + c2*w_d2*((1 - alpha)*lastStatic + beta*lastDyn);
                    viewOnProbabilityStatic(0, i) = c1*lastStatic + c2*w_p2*(alpha*lastStatic + (1 - beta)*lastDyn);
                }
                else
                {
                    viewOnProbabilityStatic(0,i) = eps;
                    viewOnProbabilityDynamic(0,i) = 1-eps;
                }


                // normalization
                const float sumZ = viewOnProbabilityDynamic(0,i) + viewOnProbabilityStatic(0, i);
                assert(sumZ >= eps);

                viewOnProbabilityDynamic(0,i) /= sumZ;

                viewOnProbabilityStatic(0,i) /= sumZ;

                //viewOnDynamicRatio(0,mapId) =viewOnProbabilityDynamic(0, mapId);
                viewOnDynamicRatio(0,i) = w_d2;

            }
        }
    }
    mapPointCloud->addDescriptor("probabilityDynamic", viewOnProbabilityDynamic);
    mapPointCloud->addDescriptor("probabilityStatic",viewOnProbabilityStatic);
    mapPointCloud->addDescriptor("dynamic_ratio", viewOnDynamicRatio);
}

void Mapper::blindSpotCloudUpdate(DP* newPointCloud, const PM::TransformationParameters TSensorRelative, bool initialized)
{
    const double trackingTangent   = tan(- blindSpotDownlookAngleTrack*3.1415926/180) ;
    const double detectingTrangent = tan(- blindSpotDownlookAngleDetect*3.1415926/180);

    int cloudCount = 0;
    if(initialized)
    {
        DP *lastCloud = new DP(blindSpotPointCloud->features,blindSpotPointCloud->featureLabels,
                         blindSpotPointCloud->descriptors, blindSpotPointCloud->descriptorLabels);
        // Transform the last point cloud in new point cloud coordinates, sensor frame
        *lastCloud = transformation->compute( *lastCloud, TSensorRelative.inverse() );

        //tracking last blind Spot cloud
        const int lastCloudCount(lastCloud->features.cols());
        for(int i = 0; i<lastCloudCount; i++)
        {
            const Eigen::Vector2f pt_xy = lastCloud->features.col(i).head(2);
            float pt_z = lastCloud->features(2,i);
            if(pt_z < 0)
            {
                float trackingMaxNorm = -blindSpotTrackMinTranslate + pt_z/trackingTangent;
                if(pt_xy.norm() < trackingMaxNorm)
                {
                    blindSpotPointCloud->setColFrom(cloudCount, *lastCloud, i);
                    cloudCount ++;
                }
            }
        }
        delete lastCloud;
        if(cloudCount>0)
            blindSpotPointCloud->conservativeResize(cloudCount);
    }

    // add new scan
    if(!initialized)
        blindSpotPointCloud = new DP(newPointCloud->features,newPointCloud->featureLabels,
                               newPointCloud->descriptors, newPointCloud->descriptorLabels);
    else
        blindSpotPointCloud->concatenate(*newPointCloud);
    const int newCloudCount(newPointCloud->features.cols());
    for(int i = 0; i<newCloudCount; i++)
    {
        const Eigen::Vector2f pt_xy = newPointCloud->features.col(i).head(2);
        float pt_z = newPointCloud->features(2,i);
        if(pt_z < 0)
        {
            float pt_tangentVertical = pt_z/pt_xy.norm();
            if(pt_tangentVertical < detectingTrangent)
            {
                blindSpotPointCloud->setColFrom(cloudCount, *newPointCloud, i);
                cloudCount ++;
            }
        }
    }
    if(cloudCount>0)
        blindSpotPointCloud->conservativeResize(cloudCount);
}

void Mapper::groundCloudFilter(DP* newPointCloud)
{
    DP *tempCloud = new DP(newPointCloud->features, newPointCloud->featureLabels,
                         newPointCloud->descriptors, newPointCloud->descriptorLabels);
    int tempCloudCount = tempCloud->features.cols();

    float coff_a = 0, coff_b = 0, coff_c = 0, coff_d = 0;
    // 1. find the ground plane by fitting points cloud
    /*
    // //try to determine the ground plane by fitting point cloud
        // firstly, calculate the centriod point of cloud
    Eigen::Vector3f centriodPt(0.0f, 0.0f, 0.0f);
    for(int i = 0; i<cloudCount; i++)
    {
        Eigen::Vector3f pt = tempCloud->features.col(i).head(3);
        centriodPt = centriodPt + pt;
    }
    centriodPt = centriodPt/cloudCount;
        // secondly, recalculate cloud by centering in centriod point
    Eigen::MatrixXf ptCloudCenter(3, cloudCount);
    Eigen::MatrixXf ptCloudPlane(3, cloudCount);
    float xx = 0, xy = 0, xz = 0, yy = 0, yz =0 , zz = 0;
    int planeCount = 0;
    for(int i = 0; i<cloudCount; i++)
    {
        Eigen::Vector2f dpt = tempCloud->features.col(i).head(2);
        Eigen::Vector3f pt = tempCloud->features.col(i).head(3);
        pt = pt - centriodPt;
        ptCloudCenter.col(i) = pt;
        // only use the near points for plane fit
        if( dpt.norm() <  groundPlaneFitRadius )
        {
            xx += pt[0] * pt[0];
            xy += pt[0] * pt[1];
            xz += pt[0] * pt[2];
            yy += pt[1] * pt[1];
            yz += pt[1] * pt[2];
            zz += pt[2] * pt[2];

            ptCloudPlane.col(planeCount) = pt;
            planeCount ++;
        }
    }
        // thirdly, fit the plane by selected points
    float stdev = 2*groundPlaneFitMaxStdev;
    if(planeCount > groundPlaneFitLeastPoints)
    {
        float det_z = xx*yy - xy*xy;
        coff_a = (yz*xy - xz*yy) / det_z;
        coff_b = (xz*xy - yz*xx) / det_z;
        coff_c = 1;
        //coff_d = 0;
        // calculate standard deviation
        for(int i = 0; i<planeCount; i++)
        {
            Eigen::Vector3f pt = ptCloudPlane.col(i);
            float errorPt = coff_a*pt[0] + coff_b*pt[1] + coff_c*pt[2];
            stdev += errorPt*errorPt;
        }
        stdev = sqrt(stdev/planeCount);
    }
    */

    //2. find the ground plane by tranformation information
    coff_a = 0;
    coff_b = 0;
    coff_c = 1;

    Eigen::MatrixXf tempCloudToPlaneDist(1,tempCloudCount);
    Eigen::MatrixXf planarPts(2,tempCloudCount);
    for(int i=0;i<tempCloudCount;i++)
    {
        Eigen::Vector3f pt = tempCloud->features.col(i).head(3);
        tempCloudToPlaneDist(0,i) = coff_a*pt[0] + coff_b*pt[1] + coff_c*pt[2];
        Eigen::Vector2f ppt = tempCloud->features.col(i).head(2);
        planarPts.col(i) =ppt;
    }

    NNS* knnSearch = NNS::createKDTreeLinearHeap(planarPts);
    const int knn = 8;
    Eigen::MatrixXi indices(knn, tempCloudCount);
    Eigen::MatrixXf dists2(knn, tempCloudCount);
    knnSearch->knn(planarPts, indices, dists2, knn, 0, 0);

    int unevenPointCount = 0;
    Eigen::MatrixXi unevenFlag(1,tempCloudCount);
    unevenFlag.setZero(1,tempCloudCount);
    for(int i=0; i<tempCloudCount; i++)
    {
        if(unevenFlag(0,i)<1)
        {
            float distToPlane = tempCloudToPlaneDist(0,i);
            for(int k=0; k<knn; k++)
            {
                float distOfNN = tempCloudToPlaneDist(0,indices(k,i));
                float diffDist = fabs (distToPlane - distOfNN) ;

                if(diffDist > robotChassisHeight)
                {
                    newPointCloud->setColFrom(unevenPointCount, *tempCloud, i);
                    unevenFlag(0,i) = 1;
                    unevenFlag(0,indices(k,i)) = 1;
                    unevenPointCount ++;
                    break;
                }
            }
        }
    }
    newPointCloud->conservativeResize(unevenPointCount);

    delete tempCloud;
    delete knnSearch;
    // candidate process
/*
    float gridResolution = 0.1;
    Eigen::MatrixXf gridCloudMeanX(100,100);
    gridCloudMeanX.setZero(100,100);
    Eigen::MatrixXf gridCloudMeanY(100,100);
    gridCloudMeanY.setZero(100,100);
    Eigen::MatrixXf gridCloudMeanZ(100,100);
    gridCloudMeanZ.setZero(100,100);

    Eigen::MatrixXi gridCloudCount(100, 100);
    gridCloudCount.setZero(100,100);
    for(int i = 0; i<cloudCount; i++)
    {
        Eigen::Vector3f pt = tempCloud->features.col(i).head(3);
        float pt_x = pt[0];
        float pt_y = pt[1];
        float pt_z = pt[2];
        if(fabs(pt_x)<5 && fabs(pt_y)<5)
        {
          int id_x=floor ( (pt_x+5)/gridResolution );
          int id_y=floor ( (pt_y+5)/gridResolution );

          gridCloudMeanX(id_x,id_y) += pt_x;
          gridCloudMeanY(id_x,id_y) += pt_y;
          gridCloudMeanZ(id_x,id_y) += pt_z;
          gridCloudCount(id_x,id_y) +=1;
        }
    }


    //calculate the mean of the points
    for(int i = 0; i<100; i++)
    {
        for(int j = 0; j<100; j++)
        {
            if(gridCloudCount(i,j)>0)
            {
                gridCloudMeanX(i,j)/gridCloudCount(i,j);
                gridCloudMeanY(i,j)/gridCloudCount(i,j);
                gridCloudMeanZ(i,j)/gridCloudCount(i,j);
            }
        }
    }

    //calculate the distance of mean to the plane
    Eigen::MatrixXf gridCloudDistance(100,100);
    gridCloudDistance.setZero(100,100);
    for(int i = 0; i<100; i++)
    {
        for(int j = 0; j<100; j++)
        {
            if(gridCloudCount(i,j)>0)
            {
                float pt_x = gridCloudMeanX(i,j);
                float pt_y = gridCloudMeanY(i,j);
                float pt_z = gridCloudMeanZ(i,j);

                gridCloudDistance(i,j) = (coff_a*pt_x + coff_b*pt_y + coff_c*pt_z)/denominator;
            }
        }
    }


    //calculate the mean of the points
    int unevenGridCount = 0;
    for(int i = 0; i<100; i++)
    {
        for(int j = 0; j<100; j++)
        {
            bool gridEven = true;
            float distance = gridCloudDistance(i,j);
            if(gridCloudCount(i,j)>0)
            {
                for(int m=-1; m<2 ; m++)
                {
                    for(int n =-1;n<2;n++)
                    {
                        int idx = i+m;
                        int idy = j+n;
                        if(idx>=0 && idx<100 && idy>=0 && idy<100)
                        {
                            if(gridCloudCount(idx,idy)>0)
                            {
                                float temp_dist = gridCloudDistance(idx,idy);
                                if(fabs(distance-temp_dist)>0.1)
                                    gridEven=false;
                            }
                        }
                    }
                }
            }
            // find uneven grid
            if(!gridEven)
            {
                Eigen::Vector3f pt(gridCloudMeanX(i,j),gridCloudMeanY(i,j),gridCloudMeanZ(i,j));
                Eigen::Vector3f ptSensorFrame = pt + centriodPt;

                newPointCloud->features.cols(unevenGridCount).head(3) = ptSensorFrame;
                unevenGridCount++;
            }
        }
    }

    newPointCloud->conservativeResize(unevenGridCount);
*/
}

void Mapper::cloudHeightTrim(DP* newPointCloud) //this point cloud should be in odom frame
{
    DP *tempCloud = new DP(newPointCloud->features, newPointCloud->featureLabels,
                         newPointCloud->descriptors, newPointCloud->descriptorLabels);
    int cloudCount = newPointCloud->features.cols();

    int filteredCloudCount = 0;
    for(int i = 0; i<cloudCount; i++)
    {
        float pt_z = newPointCloud->features(2,i);

        if(pt_z < heightFilterMax) // cloud points that close to ground
        {
            const Eigen::Vector2f planarPt = tempCloud->features.col(i).head(2);
            float pt_xyNorm = planarPt.norm();

            // customized threshold for each point
            double threshold_z = heightFilterMin + heightFilterDistRatio * pt_xyNorm ;

            if(pt_z > threshold_z)  // cloud points that are not scanned from ground
            {
                newPointCloud->setColFrom(filteredCloudCount, *tempCloud, i);
                filteredCloudCount++;
            }
        }
        else if(pt_z < robotMaximalHeight)  // cloud points that is lower than robot height
        {
            newPointCloud->setColFrom(filteredCloudCount, *tempCloud, i);
            filteredCloudCount++;
        }
    }

    delete tempCloud;
    newPointCloud->conservativeResize(filteredCloudCount);
}

Mapper::DP* Mapper::updateMap(DP* newPointCloud, const PM::TransformationParameters Ticp, bool updateExisting)
{
    timer t;

    if (!updateExisting)
    {
        // FIXME: correct that, ugly
        cout << "Initial map creation" << endl;
        *newPointCloud = transformation->compute(*newPointCloud, Ticp);
        mapPostFilters.apply(*newPointCloud);
        return newPointCloud;
    }

    const int readPtsCount(newPointCloud->features.cols());
    const int mapPtsCount(mapPointCloud->features.cols());

    // Remove points out of sensor range
    // FIXME: this is a parameter
    PM::Matrix globalId(1, mapPtsCount);

    // find the visible partial "cut map" in local frame
    DP *mapPointCloudInSensorFrameInlier = new DP(mapPointCloudInSensorFrame->features,mapPointCloudInSensorFrame->featureLabels,
                                      mapPointCloudInSensorFrame->descriptors, mapPointCloudInSensorFrame->descriptorLabels);
    int mapPointCloudInSensorFrameInlierCount = 0;
    DP *mapPointCloudInSensorFrameOutlier = new DP(mapPointCloudInSensorFrame->features,mapPointCloudInSensorFrame->featureLabels,
                                       mapPointCloudInSensorFrame->descriptors, mapPointCloudInSensorFrame->descriptorLabels);
    int mapPointCloudInSensorFrameOutlierCount = 0;

    for (int i = 0; i < mapPtsCount; i++)
    {
        float pts_norm = mapPointCloudInSensorFrame->features.col(i).head(3).norm();
        // Inlier points need to be checked if they are dynamic points
        if (pts_norm < sensorMaxRange)
        {
            //cout<<"sensorMaxRange =" <<sensorMaxRange<<endl;
            mapPointCloudInSensorFrameInlier->setColFrom(mapPointCloudInSensorFrameInlierCount, *mapPointCloudInSensorFrame, i);
            globalId(0,mapPointCloudInSensorFrameInlierCount) = i;
            mapPointCloudInSensorFrameInlierCount++;
        }
        // Outlier points are all classified to static points
        else if ( pts_norm < slidingMapMaxRange)
        {
            //cout<<"slidingMapMaxRange =" <<slidingMapMaxRange<<endl;
            mapPointCloudInSensorFrameOutlier->setColFrom(mapPointCloudInSensorFrameOutlierCount, *mapPointCloudInSensorFrame, i);
            mapPointCloudInSensorFrameOutlierCount++;
        }
        // Points out of sliding map maximal range will not be added into the new sliding map
    }
    mapPointCloudInSensorFrameInlier->conservativeResize(mapPointCloudInSensorFrameInlierCount);
    mapPointCloudInSensorFrameOutlier->conservativeResize(mapPointCloudInSensorFrameOutlierCount);

    // Generate temporary map for density computation
    //DP tmp_map = (*mapPointCloud); // FIXME: this should be on mapPointCloudInSensorFrameInlier
    DP *tmp_map = new DP(mapPointCloudInSensorFrameInlier->features,mapPointCloudInSensorFrameInlier->featureLabels,
                        mapPointCloudInSensorFrameInlier->descriptors, mapPointCloudInSensorFrameInlier->descriptorLabels);
    tmp_map->concatenate(*newPointCloud); // this method can decrease the density of newPointCloud!!!!
    //cout << "build kdtree with points" << tmp_map.features.cols() << endl;

    // build and populate NNS
    std::shared_ptr<NNS> featureNNS;
    featureNNS.reset( NNS::create(tmp_map->features, tmp_map->features.rows() - 1, NNS::KDTREE_LINEAR_HEAP));

    PM::Matches matches_overlap(
        Matches::Dists(1, readPtsCount),
        Matches::Ids(1, readPtsCount)
    );

    featureNNS->knn(newPointCloud->features, matches_overlap.ids, matches_overlap.dists, 1, 0);

    DP overlap(newPointCloud->createSimilarEmpty());
    DP no_overlap(newPointCloud->createSimilarEmpty());

    int ptsOut = 0;
    int ptsIn = 0;
    for (int i = 0; i < readPtsCount; ++i)
    {
        if (matches_overlap.dists(i) > maxDistNewPoint)
        {
            no_overlap.setColFrom(ptsOut, *newPointCloud, i);
            ptsOut++;
        }
        else
        {
            overlap.setColFrom(ptsIn, *newPointCloud, i);
            ptsIn++;
        }
    }
    no_overlap.conservativeResize(ptsOut);
    overlap.conservativeResize(ptsIn);

    // shrink the newPointCloud to the new information
    *newPointCloud = no_overlap;

    // Merge point clouds to map
    newPointCloud->concatenate(*mapPointCloudInSensorFrameInlier);
    newPointCloud->concatenate(*mapPointCloudInSensorFrameOutlier);

    delete mapPointCloudInSensorFrameInlier;
    delete mapPointCloudInSensorFrameOutlier;
    delete tmp_map;

    // Correct new points using ICP result
    *newPointCloud = transformation->compute(*newPointCloud, Ticp);

    mapPostFilters.apply(*newPointCloud);

//    cout << "... end map creation" << endl;
//    ROS_INFO_STREAM("[TIME] New map available (" << newPointCloud->features.cols() << " pts), update took " << t.elapsed() << " [s]");

    return newPointCloud;
}

void Mapper::waitForMapBuildingCompleted()
{
    #if BOOST_VERSION >= 104100
    if (mapBuildingInProgress)
    {
        // we wait for now, in future we should kill it
        mapBuildingFuture.wait();
        mapBuildingInProgress = false;
    }
    #endif // BOOST_VERSION >= 104100
}

void Mapper::publishLoop(double publishPeriod)
{
    if(publishPeriod == 0)
        return;
    ros::Rate r(1.0 / publishPeriod);
    while(ros::ok())
    {
        publishTransform();
        r.sleep();
    }
}

void Mapper::publishTransform()
{
//    if(processingNewCloud == false && publishMapTf == true)
//    {
//        publishLock.lock();
//        // Note: we use now as timestamp to refresh the tf and avoid other buffer to be empty
//        tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TFootPrintToMap, mapFrame, baseFootPrintFrame, ros::Time::now()));
//        publishLock.unlock();
//    }
}
// Main function supporting the Mapper class
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sliding_mapper_double");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    Mapper mapper(n, pn);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
