/************************************************************
 * Name: get_paper.cpp
 * Author: Jeffrey Wang, Michelle Sit
 * Date: 02/24/2018
 *
 * Description:
 ***********************************************************/

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <cmvision/Blobs.h>
#include <stdio.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <time.h>
#include <math.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/* SPECIFIED CONSTANTS */
#define FREQ 10.0

#define WALL_FOLLOW_DISTANCE 0.35
#define WALL_FOLLOW_SPEED 0.25
#define WALL_ANGLE -0.08
#define WALL_MAX_THRESH 5.0
#define WALL_MIN_THRESH 0.1

#define OBJ_REACHED_DIST 0.6

/* TUNABLE CONSTANTS */
#define ANGLE_CONTROL_P 0.4
#define ANGLE_CONTROL_I 0.0
#define ANGLE_CONTROL_D 0.1
#define DIST_CONTROL_P 0.1
#define DIST_CONTROL_I 0.0
#define DIST_CONTROL_D 0.05

#define WALL_FOLLOW_DIST_GAIN 0.5

#define AUTO_MOVE_SPEED_FORWARD 0.15
#define AUTO_MOVE_SPEED_ROTATE 0.25

/* IMMUTABLE CONSTANTS */
#define PI 3.14159265
#define PIXEL_PER_DEGREE 10.6666666667
#define CENTER_OF_FRAME 320

/* GLOBAL FLAGS */
bool noMoreWall = 0;
bool noBlobs = 1; // flag for whether blobs are present or not
// bool hitObstacle = 0;
bool runOnce = 0; // use for initializing states
bool programEnd = 0; // used when program terminates

/* GLOBAL VALUES */
int state = 0; // state of the robot
double zAngle = 0.0; // angle robot is at relative to wall/blob
double wallDist = 0.0; // distance from wall while wallfollowing
double objDist = 0.0; // distance the robot is away from an object it's trying to approach

using namespace std;

/************************************************************
 * PID CLASS
 ***********************************************************/
class Controller {
    double Kp;
    double Ki;
    double Kd;
    double prev_error;
    double integral;
    double dt;

public:
    // Constructor
    Controller(double Pvalue, double Ivalue, double Dvalue, double loopTime) {
        Kp = Pvalue;
        Ki = Ivalue;
        Kd = Dvalue;
        dt = loopTime;

        prev_error = 0;
        integral = 0;
    }

    // Controller
    double run(double target, double current) {
        double error = target - current;

        integral = integral + error*dt;
        double derivative = (error - prev_error)/dt;
        prev_error = error;

        return Kp*error + Ki*integral + Kd*derivative;
    }
};

/************************************************************
* Function Name: getMedian
* Returns the median of an array
************************************************************/
double getMedian(double data[], int size) {
    // find all nans
    int newSize;
    newSize = size;
    for(int i = 0; i < size; ++i) {
        if (isnan(data[i])) {
            newSize--;
        }
    }
    ROS_INFO("Getting median of %d size array", newSize);
    if(newSize < 1) {
        //ROS_INFO("NaN");
        return data[0];
    }
    // make a copy of the array
    double sorted[newSize];
    int k = 0;
    for(int i = 0; i < size; ++i) {
        if (!isnan(data[i])) {
            sorted[k] = data[i];
            k++;
        }
    }
    if(newSize == 1) {
        return sorted[0];
    }
    // sort the copy
    for(int i = newSize-1; i > 0; --i) {
        for(int j = 0; j < i; ++j) {
            if(sorted[j] > sorted[j+1]) {
                double temp = sorted[j];
                sorted[j] = sorted[j+1];
                sorted[j+1] = temp;
            }
        }
    }

    // get the middle value(s)
    double median = 0;
    if ((size%2) == 0) {
        median = (sorted[size/2] + sorted[(size/2)-1])/2.0;
    } else {
        median = sorted[size/2];
    }

    // delete copy
    return median;
}

/************************************************************
* Function Name: moveRobot
* Moves the robot a set amount - currently open loop control
* TODO - make closed loop eventually with odometry
************************************************************/
// Should it be const ros::Publisher& pub ??
void moveRobot(double dist, string type, ros::Publisher& pub) {

    ROS_INFO("Automatically moving robot...");
    ros::Rate loop_rate(FREQ);
    geometry_msgs::Twist t;

    double Vx = 0.0;
    double Vz = 0.0;
    int runTime = 0;

    // determine direction and runtime
    if (type.compare("forward")==0) {
        if (dist > 0) {
            Vx = AUTO_MOVE_SPEED_FORWARD;
        } else {
            Vx = -AUTO_MOVE_SPEED_FORWARD;
        }
        runTime = (int) (dist/AUTO_MOVE_SPEED_FORWARD*FREQ);
    } else if (type.compare("rotate")==0) {
        if (dist > 0) {
            Vz = AUTO_MOVE_SPEED_ROTATE;
        } else {
            Vz = -AUTO_MOVE_SPEED_ROTATE;
        }
        runTime = (int) (dist/AUTO_MOVE_SPEED_ROTATE*FREQ);
    }

    int counter = 0;
    //runtime loop
    while(ros::ok()) {
        if(counter > runTime) {
            break;
        }
        t.linear.x = Vx;
        t.linear.y = 0.0;
        t.linear.z = 0.0;
        t.angular.x = 0.0;
        t.angular.y = 0.0;
        t.angular.z = Vz;
        // ros::spinOnce(); // should this be removed?? nec. for obstacle detection
        loop_rate.sleep();
        pub.publish(t);
        counter++;
    }
}

/************************************************************
 * Function Name: blobsCallBack
 * Parameters: const cmvision::Blobs
 * Returns: void
 *
 * Description: This is the callback function of the /blobs topic
 *
 * TODO: change color values to those of the desired objects
 ***********************************************************/
void blobsCallBack (const cmvision::Blobs& blobsIn) {
    //this gets the centroid of the color blob corresponding to the goal.
    /************************************************************
    * These blobsIn.blobs[i].red, blobsIn.blobs[i].green, and blobsIn.blobs[i].blue values depend on the
    * values those are provided in the colos.txt file.
    * For example, the color file is like:
    *
    * [Colors]
    * (255, 0, 0) 0.000000 10 RED
    * (255, 255, 0) 0.000000 10 YELLOW
    * [Thresholds]
    * ( 127:187, 142:161, 175:197 )
    * ( 47:99, 96:118, 162:175 )
    *
    * Now, if a red blob is found, then the blobsIn.blobs[i].red will be 255, and the others will be 0.
    * Similarly, for yellow blob, blobsIn.blobs[i].red and blobsIn.blobs[i].green will be 255, and blobsIn.blobs[i].blue will be 0.
    ************************************************************/
    ROS_INFO("Blobs callback - state: %d", state);
    switch(state) {
    case 3 :
    {   // STATE 3: look for printer
        int xSum = 0;
        int numBlobs = 0;

        for (int i = 0; i < blobsIn.blob_count; i++) {
            //Look for "teal"
            // TODO change to printer color
            if (blobsIn.blobs[i].green == 255 && blobsIn.blobs[i].blue == 255) {
                xSum += blobsIn.blobs[i].x; //stores the average x-coordinate of the goal blobs
                numBlobs++;
            }
        }

        // convert from pixel to angle
	if (numBlobs > 0) {
        	noBlobs = 0;
        	zAngle = (xSum/numBlobs - CENTER_OF_FRAME)/PIXEL_PER_DEGREE * PI / 180;
        	ROS_INFO("Found (state %d) blob at %f degrees.", state, zAngle*180/PI);
	} else {
		ROS_INFO("No blobs found");
	}
        break;
    }
    case 5 :
    {   // STATE 5: Look for wall
        int xSum = 0;
        int numBlobs = 0;

        for (int i = 0; i < blobsIn.blob_count; i++) {
            //Look for "teal"
            // TODO change to wall color
            if (blobsIn.blobs[i].green == 255 && blobsIn.blobs[i].blue == 255) {
                xSum += blobsIn.blobs[i].x; //stores the average x-coordinate of the goal blobs
                numBlobs++;
            }
        }

        // convert from pixel to angle
	if (numBlobs > 0) {
        	noBlobs = 0;
        	zAngle = (xSum/numBlobs - CENTER_OF_FRAME)/PIXEL_PER_DEGREE * PI / 180;
        	ROS_INFO("Found (state %d) blob at %f degrees.", state, zAngle*180/PI);
	} else {
		ROS_INFO("No blobs found.");
	}
        break;
    }
    default :
        // don't do anything
        break;
    }
}

/************************************************************
 * Function Name: PointCloud_Callback
 * Parameters: const PointCloud::ConstPtr
 * Returns: void
 *
 * Description: This is the callback function of the PointCloud
 * 				topic, function depends on state
 *
 * TODO: add in the rest of the states
 ***********************************************************/
void PointCloud_Callback (const PointCloud::ConstPtr& cloud) {
    // Determine state
    ROS_INFO("PointCloud callback - state: %d", state);
    switch(state) {
    case 0 :
    {   // STATE 0: Do nothing
        break;
    }
    case 1 :
    case 2 :
    {   // STATE 1&2: follow wall on the right until corner
        // get the depth at the edge of the frame
		
		//DEBUG
        /*
        vector< vector<double> > pclData;
        for(int k = 0; k < 480; k++){
        	vector<double> pclRow;
        	for(int i = 0; i < 640; i++){
        		const pcl::PointXYZ & pt=cloud->points[640*(k)+(i)];
        		cout << pt.z;
        		pclRow.push_back(pt.z);
        	}
        	pclData.push_back(pclRow);
        	cout << '\n';
        }
        */

        double x1;
        double x2;
        int i = 620; //right end of the frame - ish
        do {
            double yvals[240];
            for(int k = 0; k < 240; k++) {
                const pcl::PointXYZ & pt=cloud->points[640*(k)+(i)];
                yvals[k] = pt.z;
            }
            x1 = getMedian(yvals, 240);
            ROS_INFO("Got x1: %f", x1);
            // delete[] yvals;
            i--;
        } while(isnan(x1));

        // get the depth approximately 5 degrees away from edge
        int j = i-50;
        do {
            double yvals[240];
            for(int k = 0; k < 240; k++) {
                const pcl::PointXYZ & pt=cloud->points[640*(k)+(j)];
                yvals[k] = pt.z;
            }
            x2 = getMedian(yvals, 240);
            ROS_INFO("Got x2: %f", x2);
            //delete[] yvals;
            j--;
        } while(isnan(x2));

        // check if wall is still there
        if ((x1 > WALL_MAX_THRESH) || (x2 > WALL_MAX_THRESH) || (x1 < WALL_MIN_THRESH) || (x2 < WALL_MIN_THRESH)) {
            ROS_INFO("DETECTED END OF WALL");
            noMoreWall = 1;
            break;
        } else {
            noMoreWall = 0;
        }

        // calculate the angle of the robot relative to the wall
        double rad = (double)(i-j) / PIXEL_PER_DEGREE * PI / 180; //true distance
        double c = sqrt(pow(x1,2) + pow(x2,2) - 2*x1*x2*cos(rad));
        zAngle = PI - acos((pow(c,2) + pow(x1,2) - pow(x2,2))/(2*c*x1)) - PI/6;

        // calculate distance away from wall
        wallDist = x1*sin(zAngle + PI/6);
        break;
    }
    case 3 :
    {   // STATE 3: Look for printer, limit X FOV
        // right now, just returns closest distance
        double z_min = 20.0;
        for(int k = 0; k < 240; k++) {
            for(int i = 120; i < 520; i++) {
                const pcl::PointXYZ & pt=cloud->points[640*(180+k)+(i)];
                //Find min z
                if(pt.z < z_min) {
                    z_min = pt.z;
                }
            }
        }
        objDist = z_min;
        break;
    }
    case 4 :
        // STATE 4: Look for a person - maybe
        break;
    case 5 :
    {   // STATE 5: Look for wall
        // right now, just returns closest distance
        double z_min = 20.0;
        for(int k = 0; k < 240; k++) {
            for(int i = 120; i < 520; i++) {
                const pcl::PointXYZ & pt=cloud->points[640*(180+k)+(i)];
                //Find min z
                if(pt.z < z_min) {
                    z_min = pt.z;
                }
            }
        }
        objDist = z_min;
        break;
    }
    case 6 :
    case 7 :
    {   // STATE 6&7: follow wall on the left until corner
        double x1;
        double x2;

        // get the depth at the edge of the frame
        int i = 70; //left end of the frame - ish
        do {
            double yvals[240];
            for(int k = 0; k < 240; k++) {
                const pcl::PointXYZ & pt=cloud->points[640*(k)+(i)];
                yvals[k] = pt.z;
            }
            x1 = getMedian(yvals, 240);
            ROS_INFO("Got x1: %f", x1);
            // delete[] yvals;
            i++;
        } while(isnan(x1));

        // get the depth approximately 5 degrees away from edge
        int j = i+50;
        do {
            double yvals[240];
            for(int k = 0; k < 240; k++) {
                const pcl::PointXYZ & pt=cloud->points[640*(k)+(j)];
                yvals[k] = pt.z;
            }
            x2 = getMedian(yvals, 240);
            ROS_INFO("Got x2: %f", x2);
            //delete[] yvals;
            j++;
        } while(isnan(x2));

        // check if wall is still there
        if ((x1 > WALL_MAX_THRESH) || (x2 > WALL_MAX_THRESH) || (x1 < WALL_MIN_THRESH) || (x2 < WALL_MIN_THRESH)) {
            ROS_INFO("DETECTED END OF WALL");
            noMoreWall = 1;
            break;
        } else {
            noMoreWall = 0;
        }

        // calculate the angle of the robot relative to the wall
        double rad = (double)(i-j) / PIXEL_PER_DEGREE * PI / 180; //true distance
        double c = sqrt(pow(x1,2) + pow(x2,2) - 2*x1*x2*cos(rad));
        zAngle = PI - acos((pow(c,2) + pow(x1,2) - pow(x2,2))/(2*c*x1)) - PI/6;

        // calculate distance away from wall
        wallDist = x1*sin(zAngle + PI/6);
        break;
    }
    default :
        ROS_INFO("YO SOMEONE DIDNT CATCH ALL STATES");
        break;
    }

    // Check for obstacle...
    // TODO eventually.
}

/**********************************************************
 * Function Name: main
 * Parameters: int argc, char **argv
 * Returns: int
 *
 * Description: This is the main function. It will subscribe
 *				to the /blobs and /PointCloud topic and act
 *        according to the state it is in.
 *
 * TODO: add in the rest of the states
 ***********************************************************/
int main(int argc, char **argv) {
	
    ros::init(argc, argv, "get_paper");
	
	// check if user wants to skip states
	if (argc > 1) {
		state = atoi(argv[1]);
	}

    // Create handle that will be used for both subscribing and publishing.
    ros::NodeHandle n;

    //subscribe to /PointCloud2 topic
    ros::Subscriber PCSubscriber = n.subscribe<PointCloud>("/camera/depth/points", 1, PointCloud_Callback);

    //subscribe to /blobs topic
    ros::Subscriber blobsSubscriber = n.subscribe("/blobs", 100, blobsCallBack);

    // publishing the geometry message twist message
    ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);

    ROS_INFO("Program started!");

    // Set the loop frequency in Hz.
    ros::Rate loop_rate(FREQ);

    // translation vector
    geometry_msgs::Twist t;

    //runtime loop
    while(ros::ok()) {
        t.linear.x = 0.0;
        t.linear.y = 0.0;
        t.linear.z = 0.0;
        t.angular.x = 0.0;
        t.angular.y = 0.0;
        t.angular.z = 0.0;


        switch(state) {
        case 0 :
        {
            // STATE 0: Wait for user input
            ROS_INFO("State 0...");
            string inputString;
            while(1) {
                cout << "Ready to go (y/n)?\n";
                getline(cin, inputString);
                if(inputString.compare("y") == 0) {
                    break;
                }
            }

            // transition to state 1
            state = 1;
            break;
        }
        case 1 :
        {   // STATE 1: Move along wall till corner
            ROS_INFO("State 1...");
            if (noMoreWall) {
                // transition to state 2
                state = 2;
                moveRobot(1.25, "forward", velocityPublisher);
                moveRobot(1.5*PI, "rotate", velocityPublisher);
                moveRobot(0.5, "forward", velocityPublisher);
                while (noMoreWall) {
                    // wait for PCL to call back
                    ros::spinOnce();
                }
                break;
            }

            // calculate new angle (taking into account wallDist)
            Controller angleControl(ANGLE_CONTROL_P, ANGLE_CONTROL_I, ANGLE_CONTROL_D, 1/FREQ);
            double targetAngle = (wallDist - WALL_FOLLOW_DISTANCE)*WALL_FOLLOW_DIST_GAIN + WALL_ANGLE;
            if (targetAngle > 0.95*PI/2) {
                targetAngle = 0.95*PI/2;
            } else if (targetAngle < -0.95*PI/2) {
                targetAngle = -0.95*PI/2;
            }

            // Move
            t.linear.x = WALL_FOLLOW_SPEED;
            t.linear.y = 0.0;
            t.linear.z = 0.0;
            t.angular.x = 0.0;
            t.angular.y = 0.0;
            t.angular.z = -angleControl.run(targetAngle, zAngle);

            // DEBUG
            ROS_INFO("Angle is: [%f], target is set to [%f].", zAngle*180/PI, targetAngle*180/PI);
            break;
        }
        case 2 :
        {   // STATE 2: Move along wall till corner
            ROS_INFO("State 2...");
            if (noMoreWall) {
                // transition to state 3
                moveRobot((9*PI-3)/18, "rotate", velocityPublisher);
                state = 3;
                runOnce = 1;
                noBlobs = 1;
                break;
            }

            // calculate new angle (taking into account wallDist)
            Controller angleControl(ANGLE_CONTROL_P, ANGLE_CONTROL_I, ANGLE_CONTROL_D, 1/FREQ);
            double targetAngle = (wallDist - WALL_FOLLOW_DISTANCE)*WALL_FOLLOW_DIST_GAIN + WALL_ANGLE;
            if (targetAngle > 0.95*PI/2) {
                targetAngle = 0.95*PI/2;
            } else if (targetAngle < -0.95*PI/2) {
                targetAngle = -0.95*PI/2;
            }

            // Move
            t.linear.x = WALL_FOLLOW_SPEED;
            t.linear.y = 0.0;
            t.linear.z = 0.0;
            t.angular.x = 0.0;
            t.angular.y = 0.0;
            t.angular.z = -angleControl.run(targetAngle, zAngle);
            break;
        }
        case 3 :
        {
            // STATE 3: Find the printer and approach it
            ROS_INFO("State 3...");

            // find the printer
            Controller angleControl(ANGLE_CONTROL_P, ANGLE_CONTROL_I, ANGLE_CONTROL_D, 1/FREQ);

            // find blobs first
            while(noBlobs) {
                // move forward, tentatively
                t.linear.x = 0.1;
                t.linear.y = 0.0;
                t.linear.z = 0.0;
                t.angular.x = 0.0;
                t.angular.y = 0.0;
                t.angular.z = 0.0;

                ros::spinOnce();
                loop_rate.sleep();
                velocityPublisher.publish(t);
            }

            // blobs are found, orient robot
            while(runOnce) {
                if (zAngle < 0.1 && zAngle > -0.1) {
                    break;
                }

                t.linear.x = 0.0;
                t.linear.y = 0.0;
                t.linear.z = 0.0;
                t.angular.x = 0.0;
                t.angular.y = 0.0;
                t.angular.z = -angleControl.run(0.0, -zAngle);

                ros::spinOnce();
                loop_rate.sleep();
                velocityPublisher.publish(t);
            }
            runOnce = 0;

            // now approach the printer
            Controller distControl(DIST_CONTROL_P, DIST_CONTROL_I, DIST_CONTROL_D, 1/FREQ);
            t.linear.x = -distControl.run(OBJ_REACHED_DIST, objDist);
            t.linear.y = 0.0;
            t.linear.z = 0.0;
            t.angular.x = 0.0;
            t.angular.y = 0.0;
            t.angular.z = angleControl.run(0.0, -zAngle);

            // check to see if reached printer
            if (objDist-OBJ_REACHED_DIST < 0.05) {
                // transition to state 4
                moveRobot(PI, "rotate", velocityPublisher);
                state = 4;
            }
            break;
        }
        case 4 :
        {   // STATE 4: look for object, then ask for help (TODO)
            ROS_INFO("State 4...");
            string inputString;
            while(1) {
                cout << "Please load the printed papers on my second shelf. Done (y/n)?\n";
                getline(cin, inputString);
                if(inputString.compare("y") == 0) {
                    break;
                }
            }

            // transition to state 5
            state = 5;
            noBlobs = 1;
            moveRobot(PI, "rotate", velocityPublisher);
            break;
        }
        case 5 :
        {
            // STATE 5: Papers are loaded, find wall
            ROS_INFO("State 5...");
            Controller angleControl(ANGLE_CONTROL_P, ANGLE_CONTROL_I, ANGLE_CONTROL_D, 1/FREQ);

            // find blobs first
            while(noBlobs) {
                // move forward, tentatively
                t.linear.x = 0.1;
                t.linear.y = 0.0;
                t.linear.z = 0.0;
                t.angular.x = 0.0;
                t.angular.y = 0.0;
                t.angular.z = 0.0;

                ros::spinOnce();
                loop_rate.sleep();
                velocityPublisher.publish(t);
            }

            // blobs found, now orient to wall direction
            while(runOnce) {
                if (zAngle < 0.1 && zAngle > -0.1) {
                    break;
                }

                t.linear.x = 0.0;
                t.linear.y = 0.0;
                t.linear.z = 0.0;
                t.angular.x = 0.0;
                t.angular.y = 0.0;
                t.angular.z = angleControl.run(0.0, -zAngle);

                ros::spinOnce();
                loop_rate.sleep();
                velocityPublisher.publish(t);
            }
            runOnce = 0;

            // now approach the wall
            Controller distControl(DIST_CONTROL_P, DIST_CONTROL_I, DIST_CONTROL_D, 1/FREQ);
            t.linear.x = -distControl.run(OBJ_REACHED_DIST, objDist);
            t.linear.y = 0.0;
            t.linear.z = 0.0;
            t.angular.x = 0.0;
            t.angular.y = 0.0;
            t.angular.z = angleControl.run(0.0, -zAngle);

            // check to see if reached wall
            if (objDist-OBJ_REACHED_DIST < 0.05) {
                // transition to state 6
                moveRobot(PI/3, "rotate", velocityPublisher);
                state = 6;
            }
            break;
        }
        case 6 :
        {   // STATE 6: Move along wall on the left till corner
            ROS_INFO("State 6...");
            if (noMoreWall) {
                // transition to state 7
                state = 7;
                moveRobot(1.25, "forward", velocityPublisher);
                moveRobot(0.6*PI, "rotate", velocityPublisher);
                moveRobot(0.5, "forward", velocityPublisher);
                while (noMoreWall) {
                    // wait for PCL to call back
                    ros::spinOnce();
                }
                break;
            }

            // calculate new angle (taking into account wallDist)
            Controller angleControl(ANGLE_CONTROL_P, ANGLE_CONTROL_I, ANGLE_CONTROL_D, 1/FREQ);
            double targetAngle = (wallDist - WALL_FOLLOW_DISTANCE)*WALL_FOLLOW_DIST_GAIN + WALL_ANGLE;
            if (targetAngle > 0.95*PI/2) {
                targetAngle = 0.95*PI/2;
            } else if (targetAngle < -0.95*PI/2) {
                targetAngle = -0.95*PI/2;
            }

            // Move
            t.linear.x = WALL_FOLLOW_SPEED;
            t.linear.y = 0.0;
            t.linear.z = 0.0;
            t.angular.x = 0.0;
            t.angular.y = 0.0;
            t.angular.z = angleControl.run(targetAngle, zAngle);

            // DEBUG
            ROS_INFO("Angle is: [%f], target is set to [%f].", zAngle*180/PI, targetAngle*180/PI);
            break;
        }
        case 7 :
        {   // STATE 7: Move along wall till doorway
            ROS_INFO("State 7...");
            if (noMoreWall) {
                ROS_INFO("Papers delivered!");
                programEnd = 1;
                break;
            }

            // calculate new angle (taking into account wallDist)
            Controller angleControl(ANGLE_CONTROL_P, ANGLE_CONTROL_I, ANGLE_CONTROL_D, 1/FREQ);
            double targetAngle = (wallDist - WALL_FOLLOW_DISTANCE)*WALL_FOLLOW_DIST_GAIN + WALL_ANGLE;
            if (targetAngle > 0.95*PI/2) {
                targetAngle = 0.95*PI/2;
            } else if (targetAngle < -0.95*PI/2) {
                targetAngle = -0.95*PI/2;
            }

            // Move
            t.linear.x = WALL_FOLLOW_SPEED;
            t.linear.y = 0.0;
            t.linear.z = 0.0;
            t.angular.x = 0.0;
            t.angular.y = 0.0;
            t.angular.z = angleControl.run(targetAngle, zAngle);
            break;
        }
        default :
            ROS_INFO("YO SOMEONE DIDNT CATCH ALL STATES");
            break;
        }
        if(programEnd) {
            break;
        }

        ROS_INFO("rotating this amount: %f", t.angular.z);
        velocityPublisher.publish(t);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
