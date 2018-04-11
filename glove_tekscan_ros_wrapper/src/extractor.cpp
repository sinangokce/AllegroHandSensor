/*VIRTUALFRAMERECORDEXTRACTOR CPP*/
/*==========================================================================
 * 
 * A tool to extract ros bag of bulb data to plain text file
 *
 *========================================================================*/

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include <boost/foreach.hpp>
/*
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "SynTouchPublisher/biotac_message.h"
#include "tf/tfMessage.h"
*/
#include "HandWritingDataWrapper/HandWritingDataWrapper.h"
/*
#define SYNTOUCH_F1_DATA_TOPIC      "/contactforceF1"
#define SYNTOUCH_F2_DATA_TOPIC      "/contactforceF2"
#define SYNTOUCH_F3_DATA_TOPIC      "/contactforceF3"
#define ALLEGRO_JOINTS_DATA_TOPIC   "/allegro/joint_state"
#define SYNTOUCH_F1_RAW_DATA_TOPIC  "/finger1"
#define SYNTOUCH_F2_RAW_DATA_TOPIC  "/finger2"
#define SYNTOUCH_F3_RAW_DATA_TOPIC  "/finger3"
#define NETFT_DATA_TOPIC            "/netft_data"
#define VISION_DATA_TOPIC           "/tf"
*/

#define HANDWRITING_DATA_TOPIC      "/HandWritingData"
int main(int argc, char** argv)
{
    //extract parms
    if(argc != 3)
    {
        printf("Please specify name of bag file as well as output text file.\n");
        exit(1);
    }

    std::string bagFileName = argv[1];
    std::string textFileName = argv[2];

    //prepare bag structure
    rosbag::Bag bag;
    bag.open(bagFileName.c_str(), rosbag::bagmode::Read);

    //construct a view for message query
    //rosbag::View view(bag, rosbag::TopicQuery(SYNTOUCH_F3_DATA_TOPIC));
    //rosbag::View view(bag, rosbag::TopicQuery(SYNTOUCH_F1_RAW_DATA_TOPIC));
    //rosbag::View view(bag, rosbag::TopicQuery(NETFT_DATA_TOPIC));
    //rosbag::View view(bag, rosbag::TopicQuery(VISION_DATA_TOPIC));
    //rosbag::View view(bag, rosbag::TopicQuery(ALLEGRO_JOINTS_DATA_TOPIC));
    rosbag::View view(bag, rosbag::TopicQuery(HANDWRITING_DATA_TOPIC));

    FILE* pTextFile = fopen(textFileName.c_str(), "w");

    //extract messages
    if(pTextFile != NULL)
    {
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            //geometry_msgs::WrenchStamped::ConstPtr pData = m.instantiate< geometry_msgs::WrenchStamped > ();
            //SynTouchPublisher::biotac_message::ConstPtr pData = m.instantiate< SynTouchPublisher::biotac_message > ();
            //sensor_msgs::JointState::ConstPtr pData = m.instantiate< sensor_msgs::JointState > ();
            //tf::tfMessage::ConstPtr pData = m.instantiate< tf::tfMessage > ();
            HandWritingDataWrapper::HandWritingDataWrapper::ConstPtr pData = m.instantiate< HandWritingDataWrapper::HandWritingDataWrapper > ();

            if(pData != NULL)
            {
                //extract dimensions interest us
                //fprintf(pTextFile, "%lf %lf %lf\n", pData->wrench.force.x, pData->wrench.force.y, pData->wrench.force.z);
                
                //for syntouch raw data
                //for E
                /*     
                for(int i = 0; i < 19; ++i)
                {
                    fprintf(pTextFile, "%d ", pData->E[i]);
                }
                */
                
                //for Pac
                /*
                for(int i = 0; i < 22; ++i)
                {
                    fprintf(pTextFile, "%d ", pData->Pac[i]);
                }
                
                //for Pdc, Tac, Tdc
                fprintf(pTextFile, "%d %d %d\n", pData->Pdc, pData->Tac, pData->Tdc);
                */
                /*
                for(int i = 0; i < 16; ++i)
                {
                    fprintf(pTextFile, "%lf ", pData->position[i]);
                }
                */
                
                //for netft data
                //fprintf(pTextFile, "%lf %lf %lf %lf %lf %lf\n", pData->wrench.force.x, pData->wrench.force.y, pData->wrench.force.z, pData->wrench.torque.x, pData->wrench.torque.y, pData->wrench.torque.z);
                //for vision tf data
                //fprintf(pTextFile, "%lf %lf %lf %lf\n", pData->transforms[0].transform.rotation.x, pData->transforms[0].transform.rotation.y, pData->transforms[0].transform.rotation.z, pData->transforms[0].transform.rotation.w);
                //for handwriting data
                //fprintf(pTextFile, "%lf %lf %lf %lf %lf", pData->tip_position[0], pData->tip_position[1], pData->tip_force_x, pData->tip_force_y, pData->tip_force_z);
                //tip position, angle and pressure
                fprintf(pTextFile, "%lf %lf %lf %lf %lf ", pData->tip_position[0], pData->tip_position[1], pData->tilt_angle[0], pData->tilt_angle[1], pData->pressure);
                //avg tactile signature from thumb, index and middle
                fprintf(pTextFile, "%lf %lf %lf %lf %lf %lf\n", pData->thumb1_f_avg, pData->thumb1_s_avg, pData->index1_f_avg, pData->index1_s_avg, pData->middle1_f_avg, pData->middle1_s_avg);
                fprintf(pTextFile, "\n");
            }
            else
            {
                printf("Fail to instantiate the data.\n");
            }
        }
    }
    else
    {
        printf("Fail to create file %s.\n", textFileName.c_str());
    }

    fclose(pTextFile);
    bag.close();

    return 0;
}

