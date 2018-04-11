#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include <boost/foreach.hpp>

#include "HandWritingDataWrapper/HandWritingDataWrapper.h"

#define DATA_TOPIC      "/HandWritingData"

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
    rosbag::View view(bag, rosbag::TopicQuery(DATA_TOPIC));

    FILE* pTextFile = fopen(textFileName.c_str(), "w");

    //extract messages
    if(pTextFile != NULL)
    {
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            HandWritingDataWrapper::HandWritingDataWrapper::ConstPtr pData = m.instantiate< HandWritingDataWrapper::HandWritingDataWrapper >();
            if(pData != NULL)
            {
                //extract dimensions interest us
                //fprintf(pTextFile, "%lf %lf %lf %lf %lf\n", pData->tip_position[0], pData->tip_position[1], pData->tilt_angle[0], pData->tilt_angle[1], pData->pressure);
                //tactile signature of thumb frontal side
                for(int i = 0; i < pData->thumb1_f.size(); ++i)
                {
                    fprintf(pTextFile, "%d ", pData->thumb1_f[i]);
                }
                //tactile signature of index frontal side
                for(int i = 0; i < pData->index1_f.size(); ++i)
                {
                    fprintf(pTextFile, "%d ", pData->index1_f[i]);
                }
                //tactile signature of middle side
                for(int i = 0; i < pData->middle1_s.size(); ++i)
                {
                    fprintf(pTextFile, "%d ", pData->middle1_s[i]);
                }
                //tilt_angle & pressure
                //fprintf(pTextFile, "%lf %lf %lf\n", pData->tilt_angle[0], pData->tilt_angle[1], pData->pressure);
                //interaction force & torque from F/T sensor
                fprintf(pTextFile, "%lf %lf %lf %lf %lf %lf\n", pData->tip_force_x, pData->tip_force_y, pData->tip_force_z, pData->tip_torque_x, pData->tip_torque_y, pData->tip_torque_z);
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
