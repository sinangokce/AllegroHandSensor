#include <ros/ros.h>
//#include "ros/ros.h"
#include <string>
#include <yarp/os/all.h>
#include "geometry_msgs/WrenchStamped.h"
#include "tf/transform_listener.h"
#include "glove_tekscan_ros_wrapper/LasaDataStreamWrapper.h"

//#define TABLET_IN_PORT      "/HandWritingWrapperTablet/in"
//#define TABLET_OUT_PORT     "/tabletReader/out"
//we need data from glove and force torque sensor
#define HAND_IN_PORT        "/glove_tekscan_ros_wrapperHand/in"
#define HAND_OUT_PORT       "/gloveout1"
#define WRAP_TOPIC          "/LasaDataStream"
#define NETFT_TOPIC         "/netft_data"
#define TACT_MAT_COLS       51 
#define TACT_MAT_ROWS       29

//wrapper for synchronize and redistribute different sensory stream...

//name of object to track the motion of pen
const char* sVisionObj = "/OptiTrakPenObj_root";

using namespace yarp::os;
ros::Publisher publisher;
ros::Subscriber netftSub;

int glove_raw_data[23];
//matrix for tactile raw data, note the data needs to be mapped to concrete patches, no assumption can be made regarding its layout.
int tactile_raw_data[TACT_MAT_ROWS * TACT_MAT_COLS];
int thumb1[4 * 4];
int thumb2[3 * 4];
int index1[4 * 4];
int index2[3 * 4];
int index3[3 * 4];
int middle1[4 * 4];
int middle2[3 * 4];
int middle3[3 * 4];
int ring1[4 * 4];
int ring2[3 * 4];
int ring3[3 * 4];
int pinky1[4 * 4];
int pinky2[3 * 4];
int pinky3[3 * 4];
int palm1[4 * 19];
int palm2[5 * 19];
int palm31[4 * 8];
int palm32[5 * 4];

int sthumb1[4 * 4];
int sthumb2[3 * 4];
int sindex1[4 * 4];
int sindex2[3 * 4];
int sindex3[3 * 4];
int smiddle1[4 * 4];
int smiddle2[3 * 4];
int smiddle3[3 * 4];
int sring1[4 * 4];
int sring2[3 * 4];
int sring3[3 * 4];
int spinky1[4 * 4];
int spinky2[3 * 4];
int spinky3[3 * 4];
int spalm1[4 * 19];
int spalm2[5 * 19];
int spalm31[4 * 8];
int spalm32[5 * 4];
geometry_msgs::WrenchStamped currWrench;

//take information from tactile raw matrix
void FillSubMatrix(int start_row, int end_row, int start_col, int end_col, int* dest)
{
    //printf("FillSubMatrix\n");
    if(end_row < start_row || end_col < start_col)
    {
        printf("Incorrect specification of sub-matrix");
    }
    for(int i = 0; i < (end_row - start_row + 1); ++i)
    {
        for(int j = 0; j < (end_col - start_col + 1); ++j)
        {
            dest[i * (end_col - start_col + 1) + j] 
                = tactile_raw_data[TACT_MAT_COLS * (start_row + i) + j + start_col];
        }
    }
}

//parse tekscan data and map them into corresponding array
void ParseTekscanData()
{
    //note thumb and sthumb is exchanged...
    //for sthumb1 4 x 4
    FillSubMatrix(15, 18, 0, 3, thumb1);
    //for sthumb2 3 x 4
    FillSubMatrix(20, 22, 0, 3, thumb2);
    //for index1 4 x 4
    FillSubMatrix(0, 3, 6, 9, index1);
    //for index2 3 x 4
    FillSubMatrix(5, 7, 6, 9, index2);
    //for index3 3 x 4
    FillSubMatrix(9, 11, 6, 9, index3);
    //for middle1 4 x 4
    FillSubMatrix(0, 3, 11, 14, middle1);
    //for middle2 3 x 4
    FillSubMatrix(5, 7, 11, 14, middle2);
    //for middle3 3 x 4
    FillSubMatrix(9, 11, 11, 14, middle3);
    //for ring1 4 x 4   //changed   //changed again <hyin/Sep-25-2013>
    FillSubMatrix(0, 3, 16, 19, ring1);
    //for ring2 3 x 4   //changed
    FillSubMatrix(5, 7, 16, 19, ring2);
    //for ring3 3 x 4   //changed
    FillSubMatrix(9, 11, 16, 19, ring3);
    //for pinky1 4 x 4
    FillSubMatrix(0, 3, 21, 24, pinky1);
    //for pinky2 3 x 4
    FillSubMatrix(5, 7, 21, 24, pinky2);
    //for pinky3 3 x 4
    FillSubMatrix(9, 11, 21, 24, pinky3);
    //for palm1 4 x 19
    FillSubMatrix(13, 16, 6, 24, palm1);
    //for palm2 5 x 19
    FillSubMatrix(24, 28, 5, 13, palm2);
    //for palm31 4 x 8
    FillSubMatrix(25, 28, 17, 24, palm31);
    //for palm32 5 x 4
    FillSubMatrix(20, 24, 21, 24, palm32);
    //printf("complete for frontal side\n");
    int d = 25;
    //note sthumb and thumb is exchanged...
    //for thumb1 4 x 4
    FillSubMatrix(15, 18, 1 + d, 4 + d, sthumb1);
    //for thumb2 3 x 4
    FillSubMatrix(20, 22, 1 + d, 4 + d, sthumb2);
    //for sindex1 4 x 4
    FillSubMatrix(0, 3, 7 + d, 10 + d, sindex1);
    //for sindex2 3 x 4
    FillSubMatrix(5, 7, 7 + d, 10 + d, sindex2);
    //for sindex3 3 x 4
    FillSubMatrix(9, 11, 7 + d, 10 + d, sindex3);
    //for smiddle1 4 x 4
    FillSubMatrix(0, 3, 12 + d, 15 + d, smiddle1);
    //for smiddle2 3 x 4
    FillSubMatrix(5, 7, 12 + d, 15 + d, smiddle2);
    //for smiddle3 3 x 4
    FillSubMatrix(9, 11, 12 + d, 15 + d, smiddle3);
    //for sring1 4 x 4
    FillSubMatrix(0, 3, 17 + d, 20 + d, sring1);
    //for sring2 3 x 4
    FillSubMatrix(5, 7, 17 + d, 20 + d, sring2);
    //for sring3 3 x 4
    FillSubMatrix(9, 11, 17 + d, 20 + d, sring3);
    //for spinky1 4 x 4
    FillSubMatrix(0, 3, 22 + d, 25 + d, spinky1);
    //for spinky2 3 x 4
    FillSubMatrix(5, 7, 22 + d, 25 + d, spinky2);
    //for spinky3 3 x 4
    FillSubMatrix(9, 11, 22 + d, 25 + d, spinky3);
    //for spalm1 4 x 19
    FillSubMatrix(13, 16, 7 + d, 25 + d, spalm1);
    //for spalm2 5 x 9
    FillSubMatrix(24, 28, 6 + d, 14 + d, spalm2);
    //for spalm31 4 x 8
    FillSubMatrix(25, 28, 18 + d, 25 + d, spalm31);
    //for spalm32 5 x 4
    FillSubMatrix(20, 24, 22 + d, 25 + d, spalm32);

    return;
}

void GetTekscanRawDataFromBottle(Bottle* pBtl)
{
    //write data from tekscan data bottle
    //<hyin/Sep-25-2013> now tekscan data also includes calibrated ones, so the size of data is doubled, but we still need
    //first half...
    if(pBtl->size() == TACT_MAT_ROWS * TACT_MAT_COLS * 2)
    {
        //printf("correct length of data\n");
        for(int i = 0; i < pBtl->size() / 2; ++i)
        {
            Value& tmpVal = pBtl->get(i);
            tactile_raw_data[i] = tmpVal.asInt();
        }
        //printf("before parse tekscan data\n");
        ParseTekscanData();
        //printf("after parse tekscan data\n");
    }
    else
    {
        printf("Unexpected length of tekscan data.\n");
    }
    return;
}

void GetDataGloveRawDataFromBottle(Bottle* pBtl)
{
    //write data from data glove
    if(pBtl->size() == 27)
    {
        //the last four field is time stamp...
        for(int i = 0; i < pBtl->size() && i < 23; ++i)
        {
            Value& tmpVal = pBtl->get(i);
            glove_raw_data[i] = tmpVal.asInt();
        }
    }
    else
    {
        printf("Unexpected length of cyberglove data.\n");
    }
}

void GetHandData(Bottle* pBtl)
{
    //a header: string and a list
    for(int i = 0; i < pBtl->size() / 2; ++i)
    {
        Value& header = pBtl->get(i * 2);
        /*
        if(header.isString())
        {
            printf("header is string\n");

        }
        else if(header.isList())
        {
            printf("header is list\n");
        }
        */
        //check header of the bottle
        //const char* header_str = header.asString().c_str();
        std::string sHeader = header.asString().c_str();
        //printf("header: %s\n", header_str);
        //printf("header: %s\n", sHeader.c_str());
        if(strcmp(sHeader.c_str(), "glove") == 0)
        {
            //printf("Find glove data\n");

            //glove data
            Value& gloveVal = pBtl->get(i * 2 + 1);
            Bottle* gloveBtl = gloveVal.asList();
            GetDataGloveRawDataFromBottle(gloveBtl);
        }
        else if(strcmp(sHeader.c_str(), "tekscan") == 0)
        {
            //printf("Find tekscan data\n");
            //tekscan data
            Value& tekscanVal = pBtl->get(i * 2 + 1);
            Bottle* tekscanBtl = tekscanVal.asList();
            GetTekscanRawDataFromBottle(tekscanBtl);
        }
        //printf("\n");
    }
    return;
}


#define HAND_DATA_MSG_FILL(source, dest, dest_avg, size) \
    do { \
        sum = 0;    \
        for(int i = 0; i < (size); ++i) \
        {   \
            msg.dest.push_back(source[i]);   \
            sum += source[i];               \
        }   \
        msg.dest_avg = double(sum) / (size);        \
    }while(0)

void FillHandDataForMsg(glove_tekscan_ros_wrapper::LasaDataStreamWrapper& msg)
{
    int sum;
    //<hyin/Sep-25-2013>
    //warning: mapping has been revised, thumb/index/middle/ring/pinky/palm do not necessarily
    //correspond to real thumb/index/middle/ring/pinky/palm, they just describe region of sensors
    //refer to parseTekscanData_newsensor.m for latest mapping
    //seems only front and side of thumb phalanges are changed...
    HAND_DATA_MSG_FILL(thumb1, thumb1_f, thumb1_f_avg, 4 * 4);
    HAND_DATA_MSG_FILL(thumb2, thumb2_f, thumb2_f_avg, 3 * 4);
    HAND_DATA_MSG_FILL(index1, index1_f, index1_f_avg, 4 * 4);
    HAND_DATA_MSG_FILL(index2, index2_f, index2_f_avg, 3 * 4);
    HAND_DATA_MSG_FILL(index3, index3_f, index3_f_avg, 3 * 4);
    HAND_DATA_MSG_FILL(middle1, middle1_f, middle1_f_avg, 4 * 4);
    HAND_DATA_MSG_FILL(middle2, middle2_f, middle2_f_avg, 3 * 4);
    HAND_DATA_MSG_FILL(middle3, middle3_f, middle3_f_avg, 3 * 4);
    HAND_DATA_MSG_FILL(ring1, ring1_f, ring1_f_avg, 4 * 4);
    HAND_DATA_MSG_FILL(ring2, ring2_f, ring2_f_avg, 3 * 4);
    HAND_DATA_MSG_FILL(ring3, ring3_f, ring3_f_avg, 3 * 4);
    HAND_DATA_MSG_FILL(pinky1, pinky1_f, pinky1_f_avg, 4 * 4);
    HAND_DATA_MSG_FILL(pinky2, pinky2_f, pinky2_f_avg, 3 * 4);
    HAND_DATA_MSG_FILL(pinky3, pinky3_f, pinky3_f_avg, 3 * 4);
    //note palm is different...
    HAND_DATA_MSG_FILL(spalm1, palm1_f, palm1_f_avg, 4 * 19);
    HAND_DATA_MSG_FILL(spalm2, palm2_f, palm2_f_avg, 5 * 9);
    HAND_DATA_MSG_FILL(spalm31, palm31_f, palm31_f_avg, 4 * 8);
    HAND_DATA_MSG_FILL(spalm32, palm32_f, palm32_f_avg, 5 * 4);

    HAND_DATA_MSG_FILL(sthumb1, thumb1_s, thumb1_s_avg, 4 * 4);
    HAND_DATA_MSG_FILL(sthumb2, thumb2_s, thumb2_s_avg, 3 * 4);
    HAND_DATA_MSG_FILL(sindex1, index1_s, index1_s_avg, 4 * 4);
    HAND_DATA_MSG_FILL(sindex2, index2_s, index2_s_avg, 3 * 4);
    HAND_DATA_MSG_FILL(sindex3, index3_s, index3_s_avg, 3 * 4);
    HAND_DATA_MSG_FILL(smiddle1, middle1_s, middle1_s_avg, 4 * 4);
    HAND_DATA_MSG_FILL(smiddle2, middle2_s, middle2_s_avg, 3 * 4);
    HAND_DATA_MSG_FILL(smiddle3, middle3_s, middle3_s_avg, 3 * 4);
    HAND_DATA_MSG_FILL(sring1, ring1_s, ring1_s_avg, 4 * 4);
    HAND_DATA_MSG_FILL(sring2, ring2_s, ring2_s_avg, 3 * 4);
    HAND_DATA_MSG_FILL(sring3, ring3_s, ring3_s_avg, 3 * 4);
    HAND_DATA_MSG_FILL(spinky1, pinky1_s, pinky1_s_avg, 4 * 4);
    HAND_DATA_MSG_FILL(spinky2, pinky2_s, pinky2_s_avg, 3 * 4);
    HAND_DATA_MSG_FILL(spinky3, pinky3_s, pinky3_s_avg, 3 * 4);
    //note palm is different
    HAND_DATA_MSG_FILL(palm1, palm1_s, palm1_s_avg, 4 * 19);
    HAND_DATA_MSG_FILL(palm2, palm2_s, palm2_s_avg, 5 * 9);
    HAND_DATA_MSG_FILL(palm31, palm31_s, palm31_s_avg, 4 * 8);
    HAND_DATA_MSG_FILL(palm32, palm32_s, palm32_s_avg, 5 * 4);
    
    for(int i = 0; i < 23; ++i)
    {
        msg.finger_pos.push_back(glove_raw_data[i]);
    }
    
    return;
}

void NetFTDataCallback(const geometry_msgs::WrenchStamped& msg)
{
    currWrench = msg;
    return;
}

int main(int argc, char** argv)
{
    bool bHandPortReady = false;
    Network yarp;
    
    //BufferedPort<Bottle> tabletIn, handIn;
    //yarp port for receiving data from glove
    BufferedPort<Bottle> handIn;
    //printf("Try to open hand port...\n"); 
    if(!handIn.open(HAND_IN_PORT))
    {
        printf("Fail to open hand port... Ignore\n");
        bHandPortReady = false;
    }
    else
    {
        printf("Found hand port, connecting...\n");
        bHandPortReady = true;
    }
    
    if(bHandPortReady)
    {
        bool success = Network::connect(HAND_OUT_PORT, HAND_IN_PORT);
        if(!success)
        {
            printf("Fail to connect glove out port to local in port...Ignore\n");
            bHandPortReady = false;
        }
    }
    

    //ros stuff
    ros::init(argc, argv, "LasaDataStreamNode");
    ros::NodeHandle hNode;
    ros::Rate rate(100);
    publisher = hNode.advertise< glove_tekscan_ros_wrapper::LasaDataStreamWrapper >( WRAP_TOPIC, 5 );
    netftSub = hNode.subscribe( NETFT_TOPIC, 5, &NetFTDataCallback );

    //prepare a transform listener
    tf::TransformListener listener;

    while(ros::ok())
    {
        ros::spinOnce();
        glove_tekscan_ros_wrapper::LasaDataStreamWrapper msg;
       
        if(bHandPortReady)
        {
            //try glove data, synchronous call here, it should be slower than tablet...
            //synchronization issue here, how can we ensure the ros::spinOnce() is run when no bottle is received in this cycle
            Bottle* handBtl = handIn.read();
            if(handBtl)
            {
                //printf("Received bottle of length %d\n", handBtl->size());

                //refresh data
                GetHandData(handBtl);
                //fill data to msg
                FillHandDataForMsg(msg);
            }
        }
        //now whether tablet is ready or not does not matter...
        msg.tip_force_x = currWrench.wrench.force.x;
        msg.tip_force_y = currWrench.wrench.force.y;
        msg.tip_force_z = currWrench.wrench.force.z;
        msg.tip_torque_x = currWrench.wrench.torque.x;
        msg.tip_torque_y = currWrench.wrench.torque.y;
        msg.tip_torque_z = currWrench.wrench.torque.z;

        //get latest vision information - pen pose
        tf::StampedTransform transform;
        try
        {
            //printf("try to get transform\n");
            listener.lookupTransform("/vision", "/OptiTrakPenObj_root", ros::Time(), transform);
            //printf("got transform\n");
            //fill pos and transform
            msg.obj_pose.translation.x = transform.getOrigin().getX();
            msg.obj_pose.translation.y = transform.getOrigin().getY();
            msg.obj_pose.translation.z = transform.getOrigin().getZ();

            msg.obj_pose.rotation.x = transform.getRotation().getX();
            msg.obj_pose.rotation.y = transform.getRotation().getY();
            msg.obj_pose.rotation.z = transform.getRotation().getZ();
            msg.obj_pose.rotation.w = transform.getRotation().getW();
        }
        catch (tf::TransformException e)
        {
            //printf("caught an exception\n");
            ROS_ERROR_THROTTLE(1, "%s", e.what());
        }

        msg.header.stamp = ros::Time::now();
        //printf("Ready to send...\n");
        publisher.publish(msg);
        //printf("Sending message done...\n");
        rate.sleep();
    }
    hNode.shutdown();

    return 0;
}
