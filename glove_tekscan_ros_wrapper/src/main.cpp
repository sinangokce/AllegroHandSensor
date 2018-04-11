#include "ros/ros.h"
#include <string>
#include <yarp/os/all.h>
#include "geometry_msgs/WrenchStamped.h"
#include "HandWritingDataWrapper/HandWritingDataWrapper.h"

#define TABLET_IN_PORT      "/HandWritingWrapperTablet/in"
#define TABLET_OUT_PORT     "/tabletReader/out"
#define HAND_IN_PORT        "/HandWritingWrapperHand/in"
#define HAND_OUT_PORT       "/gloveout1"
#define WRAP_TOPIC          "/HandWritingData"
#define NETFT_TOPIC         "/netft_data"
#define TACT_MAT_COLS       51
#define TACT_MAT_ROWS       29

using namespace yarp::os;
ros::Publisher publisher;
ros::Subscriber netftSub;

int glove_raw_data[23];
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
    //for ring1 4 x 4   //changed
    FillSubMatrix(0, 3, 16, 19, smiddle1);
    //for ring2 3 x 4   //changed
    FillSubMatrix(5, 7, 16, 19, smiddle2);
    //for ring3 3 x 4   //changed
    FillSubMatrix(9, 11, 16, 19, smiddle3);
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
    FillSubMatrix(0, 3, 12 + d, 15 + d, ring1);
    //for smiddle2 3 x 4
    FillSubMatrix(5, 7, 12 + d, 15 + d, ring2);
    //for smiddle3 3 x 4
    FillSubMatrix(9, 11, 12 + d, 15 + d, ring3);
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
    if(pBtl->size() == TACT_MAT_ROWS * TACT_MAT_COLS)
    {
        //printf("correct length of data\n");
        for(int i = 0; i < pBtl->size(); ++i)
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

void FillHandDataForMsg(HandWritingDataWrapper::HandWritingDataWrapper& msg)
{
    int sum;
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
    Network yarp;
    
    BufferedPort<Bottle> tabletIn, handIn;
    
    if(!tabletIn.open(TABLET_IN_PORT))
    {
        printf("Fail to open tablet port... Abort\n");
        //return 1;
    }
    
    /*
    if(!handIn.open(HAND_IN_PORT))
    {
        printf("Fail to open hand port... Abort\n");
        //return 1;
    }
    */

    bool success = Network::connect(TABLET_OUT_PORT, TABLET_IN_PORT);
    if(!success)
    {
        printf("Fail to connect tablet out port to local in port...\n");
        //return 1;
    }
    
    /*
    bool success = Network::connect(HAND_OUT_PORT, HAND_IN_PORT);
    if(!success)
    {
        printf("Fail to connect glove out port to local in port...\n");
        return 1;
    }
    */

    //ros stuff
    ros::init(argc, argv, "HandWritingDataWrapper");
    ros::NodeHandle hNode;
    ros::Rate rate(20);
    publisher = hNode.advertise< HandWritingDataWrapper::HandWritingDataWrapper >( WRAP_TOPIC, 5 );
    netftSub = hNode.subscribe( NETFT_TOPIC, 5, &NetFTDataCallback );

    while(ros::ok())
    {
        ros::spinOnce();
        HandWritingDataWrapper::HandWritingDataWrapper msg;
        
        //try glove data, synchronous call here, it should be slower than tablet...
        /*
        Bottle* handBtl = handIn.read();
        if(handBtl)
        {
            //printf("Received bottle of length %d\n", handBtl->size());

            //refresh data
            GetHandData(handBtl);
            //fill data to msg
            FillHandDataForMsg(msg);
        }
*/
        bool bTabletReady = false;
        Bottle* bottle = tabletIn.read(false);
        if(bottle)
        {
            //get tip position
            Value& tip_pos_val = bottle->get(0);
            Bottle* tip_pos_btl = tip_pos_val.asList();
            Value& tip_pos_x = tip_pos_btl->get(0);
            Value& tip_pos_y = tip_pos_btl->get(1);
            msg.tip_position.push_back(tip_pos_x.asDouble());
            msg.tip_position.push_back(tip_pos_y.asDouble());
            //get tilt angle of pen
            Value& tilt_angle_val = bottle->get(1);
            Bottle* tilt_angle_btl = tilt_angle_val.asList();
            Value& tilt_ang_x = tilt_angle_btl->get(0);
            Value& tilt_ang_y = tilt_angle_btl->get(1);
            msg.tilt_angle.push_back(tilt_ang_x.asDouble());
            msg.tilt_angle.push_back(tilt_ang_y.asDouble());
            //get pressure
            Value& pressure_val = bottle->get(2);
            msg.pressure = pressure_val.asDouble();

            bTabletReady = true;
        }
        
        /* 
        if(bTabletReady)
        {
            //time stamp
            msg.header.stamp = ros::Time::now();
            publisher.publish(msg);
        }
        */
        //now whether tablet is ready or not does not matter...
        msg.tip_force_x = currWrench.wrench.force.x;
        msg.tip_force_y = currWrench.wrench.force.y;
        msg.tip_force_z = currWrench.wrench.force.z;
        msg.tip_torque_x = currWrench.wrench.torque.x;
        msg.tip_torque_y = currWrench.wrench.torque.y;
        msg.tip_torque_z = currWrench.wrench.torque.z;
        msg.header.stamp = ros::Time::now();
        //publisher.publish(msg);
         
        if(bTabletReady)
        {
            //time stamp
            msg.header.stamp = ros::Time::now();
            publisher.publish(msg);
        }
        
 
        //rate.sleep();
    }

    return 0;
}
