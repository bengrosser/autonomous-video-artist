#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <geometry_msgs/Twist.h>

enum DriveAction
{
    FORWARD, LEFT, RIGHT
};

class TandemObstacleAvoidance
{
    private:
        ros::NodeHandle node;
        ros::Publisher velocity;
        ros::Publisher panorama;
        ros::Publisher height;
        ros::Publisher ground;
        ros::Publisher occlusions;
        DriveAction currentMOTION;
        DriveAction directionsPrimary;
        DriveAction directionsSecondary;
        std::list<int> heightRangeFrontSamples;
        double last_GROUND_CLOSEY, last_GROUND_CLOSEZ, last_GROUND_FARY, last_GROUND_FARZ;
        double GROUND_SLOPE, GROUND_YINTERCEPT;
        DriveAction groundLastForcedTurn;

        const char* directionRepresentation(DriveAction plan){
            switch(plan){
                case LEFT:
                    return "LEFT";
                case RIGHT:
                    return "RIGHT";
                default:
                    return "FORWARD";
            }
        }
    public:
        TandemObstacleAvoidance(ros::NodeHandle& handle):node(handle), velocity(node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1)), panorama(node.advertise<pcl::PointCloud<pcl::PointXYZ> >("panorama",1)), height(node.advertise<pcl::PointCloud<pcl::PointXYZ> >("height",1)), ground(node.advertise<pcl::PointCloud<pcl::PointXYZ> >("ground",1)), occlusions(node.advertise<pcl::PointCloud<pcl::PointXYZ> > ("occlusion",1)), currentMOTION(FORWARD),directionsPrimary(FORWARD), directionsSecondary(FORWARD),last_GROUND_CLOSEY(0), last_GROUND_CLOSEZ(0),last_GROUND_FARY(0), last_GROUND_FARZ(0),groundLastForcedTurn(LEFT){
        ros::MultiThreadedSpinner threads(3);
        ros::Subscriber heightRange=node.subscribe("/camera/depth/points", 1, & TandemObstacleAvoidance::heightRange, this);
        ros::Subscriber groundEdges=node.subscribe("/camera/depth_registered/points", 1, & TandemObstacleAvoidance::groundEdges, this);
        ros::Timer pilot=node.createTimer(ros::Duration(0.1), &TandemObstacleAvoidance::pilot, this);
        threads.spin();
        }
        
        //obstacle detection and motion planning 
        void heightRange (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
            //std::cout<<"heightRange"<<std::endl;
            double CROP_XRADIUS, CROP_YMIN, CROP_YMAX, CROP_ZMIN, CROP_ZMAX, HEIGHT_DOWNSAMPLING;
            int HEIGHT_SAMPLES;
            bool HEIGHT_VERBOSE;
            
            node.getParamCached("crop_xradius", CROP_XRADIUS);
            node.getParamCached("crop_ymin", CROP_YMIN);
            node.getParamCached("crop_ymax", CROP_YMAX);
            node.getParamCached("crop_zmin", CROP_ZMIN);
            node.getParamCached("crop_zmax", CROP_ZMAX);
            node.getParamCached("height_downsampling", HEIGHT_DOWNSAMPLING);
            node.getParamCached("height_samples", HEIGHT_SAMPLES);
            node.getParamCached("height_verbose", HEIGHT_VERBOSE);

            pcl::PassThrough<pcl::PointXYZ> crop;
            pcl::VoxelGrid<pcl::PointXYZ> downsample;
            pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr front (new pcl::PointCloud<pcl::PointXYZ>);
            int averageObstacles = 0;

            downsample.setInputCloud(cloud);
            if(HEIGHT_DOWNSAMPLING>=0) downsample.setLeafSize((float)HEIGHT_DOWNSAMPLING, (float)HEIGHT_DOWNSAMPLING, (float) HEIGHT_DOWNSAMPLING);
            downsample.filter(*downsampled);
            
            //crop
            crop.setInputCloud(downsampled);
            crop.setFilterFieldName("x");
            crop.setFilterLimits(-CROP_XRADIUS, CROP_XRADIUS);
            crop.filter(*front);
            
            crop.setInputCloud(front);
            crop.setFilterFieldName("y");
            crop.setFilterLimits(CROP_YMIN, CROP_YMAX);
            crop.filter(*front);

            crop.setInputCloud(front);
            crop.setFilterFieldName("z");
            crop.setFilterLimits(CROP_ZMIN, CROP_ZMAX);
            crop.filter(*front);
        
            if(currentMOTION!=FORWARD)
                heightRangeFrontSamples.clear();
            std::cout<<"front size"<<front->size()<<std::endl;
            heightRangeFrontSamples.push_front(front->size());
            while(heightRangeFrontSamples.size()>(unsigned) HEIGHT_SAMPLES)
                heightRangeFrontSamples.pop_back();

            for(std::list<int>::iterator location=heightRangeFrontSamples.begin(); location!=heightRangeFrontSamples.end(); ++location)
                averageObstacles+=(*location);
            averageObstacles /=heightRangeFrontSamples.size();
            std::cout<<"average obstacle"<<averageObstacles<<std::endl;
            if(averageObstacles>20){
                float centroidX = 0;
                for(pcl::PointCloud<pcl::PointXYZ>::iterator point=front->begin(); point<front->end(); point++)
                    centroidX+=point->x;
                centroidX/=front->size();
                if(HEIGHT_VERBOSE)
                    ROS_INFO("HEIGHT_RANGE :: Seeing %4d points in our way\n -> Centroid is at %.3f i", averageObstacles, centroidX);

                if(centroidX < 0){
                    directionsPrimary = RIGHT;
                }
                else{
                    directionsPrimary = LEFT;
                }
            }
            else
                directionsPrimary=FORWARD;
            panorama.publish(*downsampled);
            height.publish(*front);
        }
        
        void groundEdges(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
            double CROP_XRADIUS, CROP_YMIN, CROP_YMAX, CROP_ZMIN, CROP_ZMAX, GROUND_BUMPERFRONTAL, GROUND_BUMPERLATERAL, GROUND_CLOSEY, GROUND_CLOSEZ, GROUND_FARY, GROUND_FARZ, GROUND_TOLERANCEFINE, GROUND_TOLERANCEROUGH, GROUND_NORMALSMOOTHING, GROUND_THRESHOLDLOWER, GROUND_THRESHOLDHIGHER, GROUND_OUTLIERRADIUS;
            int GROUND_NORMALESTIMATION, GROUND_OUTLIERNEIGHBORS;
            bool GROUND_VERBOSE;

            node.getParamCached("crop_xradius", CROP_XRADIUS);
            node.getParamCached("crop_ymin", CROP_YMIN);
            node.getParamCached("crop_ymax", CROP_YMAX);
            node.getParamCached("crop_zmin", CROP_ZMIN);
            node.getParamCached("crop_zmax", CROP_ZMAX);
            node.getParamCached("ground_bumperfrontal", GROUND_BUMPERFRONTAL);
            node.getParamCached("ground_bumperlateral", GROUND_BUMPERLATERAL);
            node.getParamCached("ground_closey", GROUND_CLOSEY);
            node.getParamCached("ground_closez", GROUND_CLOSEZ);
            node.getParamCached("ground_fary", GROUND_FARY);
            node.getParamCached("ground_farz", GROUND_FARZ);
            node.getParamCached("ground_tolerancefine", GROUND_TOLERANCEFINE);
            node.getParamCached("ground_tolerancerough", GROUND_TOLERANCEROUGH);
            node.getParamCached("ground_normalsmoothing", GROUND_NORMALSMOOTHING);
            node.getParamCached("ground_thresholdlower", GROUND_THRESHOLDLOWER);
            node.getParamCached("ground_thresholdhigher", GROUND_THRESHOLDHIGHER);
            node.getParamCached("ground_outlierradius", GROUND_OUTLIERRADIUS);
            node.getParamCached("ground_normalestimation", GROUND_NORMALESTIMATION);
            node.getParamCached("ground_outlierneighbors", GROUND_OUTLIERNEIGHBORS);
            node.getParamCached("ground_verbose", GROUND_VERBOSE);

            if(GROUND_CLOSEY!=last_GROUND_CLOSEY || GROUND_CLOSEZ!=last_GROUND_CLOSEZ || GROUND_FARY != last_GROUND_FARY || GROUND_FARZ != last_GROUND_FARZ){
                GROUND_SLOPE = (GROUND_FARY-GROUND_CLOSEY) / (GROUND_FARZ-GROUND_CLOSEZ);
                GROUND_YINTERCEPT = (GROUND_CLOSEY+GROUND_FARY)/2-GROUND_SLOPE*(GROUND_CLOSEZ+GROUND_FARZ)/2;
                last_GROUND_CLOSEY=GROUND_CLOSEY;
                last_GROUND_FARY = GROUND_FARY;
                last_GROUND_CLOSEZ=GROUND_CLOSEZ;
                last_GROUND_FARZ = GROUND_FARZ;
            }
        
            pcl::PassThrough<pcl::PointXYZRGB> crop;
            //pcl::OrganizedEdgeDetection<pcl::PointXYZRGB, pcl::Label> detect;
            pcl::OrganizedEdgeBase<pcl::PointXYZRGB, pcl::Label> detect;

            pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> remove;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::Label> edgePoints;
            std::vector<pcl::PointIndices> edges;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr navigation(new pcl::PointCloud<pcl::PointXYZRGB>);
            int trueGroundPoints = 0;
            double trueGroundXTotal = 0;
            crop.setInputCloud(cloud);
            crop.setFilterFieldName("x");
            crop.setFilterLimits(-CROP_XRADIUS-GROUND_BUMPERLATERAL, CROP_XRADIUS+GROUND_BUMPERLATERAL);
            crop.setKeepOrganized(true);
            crop.filter(*points);
            
            crop.setInputCloud(points);
            crop.setFilterFieldName("y");
            crop.setFilterLimits(CROP_YMAX, 1);
            crop.setKeepOrganized(true);
            crop.filter(*points);
            
            crop.setInputCloud(points);
            crop.setFilterFieldName("z");
            crop.setFilterLimits(CROP_ZMIN, CROP_ZMAX+GROUND_BUMPERFRONTAL);
            crop.setKeepOrganized(true);
            crop.filter(*points);

            for(pcl::PointCloud<pcl::PointXYZRGB>::iterator location=points->begin(); location<points->end(); location++){
                double distanceFromGroundPlane = fabs(location->y-(GROUND_SLOPE*location->z+GROUND_YINTERCEPT));
                if(distanceFromGroundPlane > GROUND_TOLERANCEROUGH){
                    location->x=std::numeric_limits<float>::quiet_NaN();
                    location->y=std::numeric_limits<float>::quiet_NaN();
                    location->z=std::numeric_limits<float>::quiet_NaN();
                }
                else if(distanceFromGroundPlane <= GROUND_TOLERANCEFINE && fabs(location->x)<CROP_XRADIUS-GROUND_BUMPERLATERAL && location->z > GROUND_CLOSEZ+GROUND_BUMPERFRONTAL && location->z < CROP_ZMAX-GROUND_BUMPERFRONTAL){
                    trueGroundPoints++;
                    trueGroundXTotal+=location->x;
                }
            }
            if (trueGroundPoints > 0){
                detect.setInputCloud(points);
                detect.setEdgeType (detect.EDGELABEL_HIGH_CURVATURE+detect.EDGELABEL_NAN_BOUNDARY);
                /*if(GROUND_NORMALESTIMATION>=0)
                    detect.setHighCurvatureNormalEstimationMethod((pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal>::NormalEstimationMethod) GROUND_NORMALESTIMATION);
                if(GROUND_NORMALSMOOTHING>=0)
                    detect.setHighCurvatureNormalSmoothingSize((float)GROUND_NORMALSMOOTHING);
                if(GROUND_THRESHOLDLOWER>=0)
                    detect.setHighCurvatureEdgeThresholdLower((float)GROUND_THRESHOLDLOWER);
                if(GROUND_THRESHOLDHIGHER>=0)
                    detect.setHighCurvatureEdgeThresholdHigher((float)GROUND_THRESHOLDHIGHER);*/
                detect.compute(edgePoints, edges);

                if(GROUND_VERBOSE)
                    ROS_INFO("GROUND EDGES :: Saw raw %4lu curves and %4lu borders", edges[3].indices.size(), edges[0].indices.size());
                navigation->header = points->header;
                for(std::vector<pcl::PointIndices>::iterator edge=edges.begin(); edge<edges.end(); edge++){
                    for(std::vector<int>::iterator pointIndex=edge->indices.begin(); pointIndex<edge->indices.end(); pointIndex++){
                        if (fabs((*points) [*pointIndex].x)<CROP_XRADIUS-GROUND_BUMPERLATERAL && (*points)[*pointIndex].z > GROUND_CLOSEZ+GROUND_BUMPERFRONTAL && (*points)[*pointIndex].z<CROP_ZMAX-GROUND_BUMPERFRONTAL)
                            navigation->push_back((*points) [*pointIndex]);
                    }
                }
                if(GROUND_OUTLIERRADIUS>=0 && navigation->size() >0){
                    remove.setInputCloud(navigation);
                    remove.setRadiusSearch((float) GROUND_OUTLIERRADIUS);
                    if(GROUND_OUTLIERNEIGHBORS>=0)
                        remove.setMinNeighborsInRadius(GROUND_OUTLIERNEIGHBORS);
                    remove.filter(*navigation);
                }
            }    
            else if(GROUND_VERBOSE)
                ROS_INFO("GROUND EDGES :: Lost sight of the ground!");

            //plan on next move
            if(navigation->size()>0){
                float centroidX = 0;
                for(pcl::PointCloud<pcl::PointXYZRGB>::iterator point=navigation->begin(); point<navigation->end(); point++)
                    centroidX+=point->x;
                centroidX/=navigation->size();
                if(GROUND_VERBOSE)
                    ROS_INFO("GROUND EDGES :: Seeing %3lu offending points centered at %.3f i", navigation->size(), centroidX);

                if(centroidX <0)
                    directionsSecondary=RIGHT;
                else
                    directionsSecondary=LEFT;
                groundLastForcedTurn=directionsSecondary;
            } 
            else if(trueGroundPoints==0){
                if(GROUND_VERBOSE)
                    ROS_INFO("GROUND EDGES :: Ground has vanished; calling for emergency evasive maneuvers!");
                directionsSecondary=groundLastForcedTurn;
            }
            else{
                directionsSecondary=FORWARD;
                groundLastForcedTurn=trueGroundXTotal/trueGroundPoints>0?RIGHT:LEFT;
            }
            ground.publish(*points);
            occlusions.publish(*navigation);
        }

        void pilot(const ros::TimerEvent& time){
            //std::cout<<"pilot"<<std::endl;
           
            double DRIVE_LINEARSPEED, DRIVE_ANGULARSPEED;
            bool DRIVE_MOVE, DRIVE_VERBOSE;

            node.getParamCached("drive_linearspeed", DRIVE_LINEARSPEED);
            node.getParamCached("drive_angularspeed", DRIVE_ANGULARSPEED);
            node.getParamCached("drive_move", DRIVE_MOVE);
            node.getParamCached("drive_verbose", DRIVE_VERBOSE);

            geometry_msgs::Twist decision;
            switch(directionsPrimary){
                case LEFT:
                    if(DRIVE_VERBOSE) 
                        ROS_INFO("PILOT :: Turning %5s", "LEFT");
                    decision.angular.z=DRIVE_ANGULARSPEED;
                    break;
                case RIGHT:
                    if(DRIVE_VERBOSE)
                        ROS_INFO("PILOT :: Turning %5s", "RIGHT");
                    decision.angular.z=-DRIVE_ANGULARSPEED;
                    break;
                default:
                    decision.linear.x=DRIVE_LINEARSPEED;

            }

            if(DRIVE_MOVE){
                //std::cout<<"drive speed:"<<DRIVE_LINEARSPEED<<std::endl;
                velocity.publish(decision);
            }         

            /*DriveAction newMotion;
            geometry_msgs::Twist decision;
            if(directionsPrimary != directionsSecondary){
                //std::cout<<"in if"<<std::endl;
                if(DRIVE_VERBOSE)
                    ROS_INFO("PILOT :: One recommendation says %5s and the other counters %5s", directionRepresentation(directionsPrimary),directionRepresentation(directionsSecondary));
                if(directionsPrimary==FORWARD)
                    newMotion=directionsSecondary;
                else if(directionsSecondary==FORWARD)
                    newMotion=directionsPrimary;
                else 
                    newMotion=directionsSecondary;
            }
            else{
                //std::cout<<"newMotion value"<<std::endl;
                newMotion=directionsPrimary;

            }
            if(newMotion!=FORWARD && currentMOTION != FORWARD && newMotion!=currentMOTION){
                if(DRIVE_VERBOSE)
                    ROS_INFO("PILOT :: Overrode recommended oscillation");
                newMotion=currentMOTION;
            }
            
            switch(newMotion){
                case LEFT:
                    if(DRIVE_VERBOSE) 
                        ROS_INFO("PILOT :: Turning %5s", "LEFT");
                    decision.angular.z=DRIVE_ANGULARSPEED;
                    break;
                case RIGHT:
                    if(DRIVE_VERBOSE)
                        ROS_INFO("PILOT :: Turning %5s", "RIGHT");
                    decision.angular.z=-DRIVE_ANGULARSPEED;
                    break;
                default:
                    decision.linear.x=DRIVE_LINEARSPEED;
            }
            if(DRIVE_MOVE){
                //std::cout<<"drive speed:"<<DRIVE_LINEARSPEED<<std::endl;
                velocity.publish(decision);
            }
            currentMOTION=newMotion;*/
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "navigation");
    ros::NodeHandle node("surface");
    //std::cout<<"here"<<std::endl;
    node.setParam("crop_xradius", 0.2);
    node.setParam("crop_ymin", -0.07);
    node.setParam("crop_ymax", 0.35);
    node.setParam("crop_zmin", 0.0);
    node.setParam("crop_zmax", 1.5);
    node.setParam("height_downsampling", 0.04);
    node.setParam("height_samples", 5);  //number of samples to average: should be low enough to prevent false positives
    node.setParam("height_verbose", true);
    node.setParam("ground_bumperfrontal", 0.1); //extra uncropped space on the front and back edges of the plane whose edges, borders, and presence are disregarded
    node.setParam("ground_bumperlateral", 0.02); //extra uncropped space on the left and right edges of the plane whose edges, borders, and presence are disregarded
    node.setParam("ground_closey", 0.3525); //y-coordinate of the closet point on the ground
    node.setParam("ground_closez", 0.8);  //corresponding z-coordinate for bumper border and modeling the plane
    node.setParam("ground_fary", 0.47); //y-coordinate of a far point on the ground
    node.setParam("ground_farz", 2.5);  //corresponding z-coordinate for modeling the plane
    node.setParam("ground_tolerancefine", 0.03);  //maximum y-coordinate deviation of points that are still considered part of the ground itself
    node.setParam("ground_tolerancerough", 0.1);  //maximum y-coordinate deviation of pints that are evaluated at all
    node.setParam("ground_normalsmoothing", -1.0); //smoothing size for normal estimation (negative for default)
    node.setParam("ground_thresholdlower", 1.0); //for curvature-based edge detection: cutoff for consideration as possible edges (negative for default)
    node.setParam("ground_thresholdhigher", 1.7);  //for curvature-based edge detection: cutoff for definite classification as edges (negative for default)
    node.setParam("ground_outlierradius",0.05); //radius used for neighbor search to filter out outliers (negative to disable outlier removal)
    node.setParam("ground_normalestimation",-1); //normal estimation method: as defined in IntegralImageNormalEstimation (negative for default)
    node.setParam("ground_outlierneighbors",6); //minimum neighbors to be spared by outlier persecution (negative by default)
    node.setParam("ground_verbose", false);
    node.setParam("drive_linearspeed",0.1);
    node.setParam("drive_angularspeed",0.18);
    node.setParam("drive_move", true);
    node.setParam("drive_verbose", false);
    
    TandemObstacleAvoidance workhorse(node);

  
    //cleanup
    node.deleteParam("crop_xradius");
    node.deleteParam("crop_ymin");
    node.deleteParam("crop_ymax");
    node.deleteParam("crop_zmin");
    node.deleteParam("crop_zmax");
    node.deleteParam("height_downsampling");
    node.deleteParam("height_samples");
    node.deleteParam("height_verbose");
    node.deleteParam("ground_bumperfrontal");
    node.deleteParam("ground_bumperlateral");
    node.deleteParam("ground_closey");
    node.deleteParam("ground_closez");
    node.deleteParam("ground_fary");
    node.deleteParam("ground_farz");
    node.deleteParam("ground_tolerancefine");
    node.deleteParam("ground_tolerancerough");
    node.deleteParam("ground_normalsmoothing");
    node.deleteParam("ground_thresholdlower");
    node.deleteParam("ground_thresholdhigher");
    node.deleteParam("ground_outlierradius");
    node.deleteParam("ground_normalestimation");
    node.deleteParam("ground_outlierneighbors");
    node.deleteParam("ground_verbose");
    node.deleteParam("drive_linearspeed");
    node.deleteParam("drive_angularspeed");
    node.deleteParam("drive_move");
    node.deleteParam("drive_verbose");
    return 0;
}

/*
class TurtleOperator
{
public:
TurtleOperator();
virtual ~TurtleOperator() {};

void moveForward();

private:
ros::NodeHandle nodeHandle;
ros::Publisher operatorPublisher;
float x_value;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_turtle");

    TurtleOperator turtleOperator;

    std::cout << "Starting to spin …" << std::endl;

    ros::Rate loopRate(15);

    while(ros::ok())
    {
    turtleOperator.moveForward();
    ROS_INFO("Published twist");
    loopRate.sleep();
    }
return EXIT_SUCCESS;
}

TurtleOperator::TurtleOperator(): x_value(0.25)
{
operatorPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",1);
}

void TurtleOperator::moveForward()
{
geometry_msgs::Twist twist;

twist.linear.x = x_value;
twist.linear.y = 0;
twist.linear.z = 0;

twist.angular.x = 0;
twist.angular.y = 0;
twist.angular.z = 0;

operatorPublisher.publish(twist);
//this->operatorPublisher.publish(twist);
}
*/
