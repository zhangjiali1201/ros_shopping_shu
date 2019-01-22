#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include <cstdio>
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"

using namespace std;

class MapGenerator
{

public:
    MapGenerator(const std::string& mapname) : mapname_(mapname),saved_map_(false)
    {
   
        ROS_INFO("Waiting for the map");
        map_sub_ = n.subscribe("map",1,&MapGenerator::mapCallback,this);
        n.setParam("permit_save_map",false);            //set the parameter for whether it is time to save map
    }

    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
    {
        bool permit_save_map  = false;
        while(true)
        {
            n.getParam("permit_save_map",permit_save_map);
            std::cout << "permit_save_map: " << (permit_save_map ? "true" : "false") << std::endl;
            if(permit_save_map == false)
                continue;

            ROS_INFO("Received a %d X %d map @%.3f m/pix",
                    map->info.width,
                    map->info.height,
                    map->info.resolution);
            std::string mapdatafile = mapname_ + ".pgm";
            ROS_INFO("Writing map occupancy data to %s",mapdatafile.c_str());
            FILE* out = fopen(mapdatafile.c_str(),"w");
            if(!out)
            {
                ROS_ERROR("Couldn't save map file to %s",mapdatafile.c_str());
                return;
            }

            fprintf(out,"P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
                    map->info.resolution, map->info.width, map->info.height);
            for(unsigned int y=0; y<map->info.height; y++){
                for(unsigned int x=0; x<map->info.width;x++){
                    unsigned int i = x + (map->info.height-y-1)*map->info.width;
                    if(map->data[i] == 0){  //occu[0,0.1)
                        fputc(254,out);
                    }else if(map->data[i] == +100){ //occu(0.65,1]
                        fputc(000,out);
                    }else{      //occu[0.1,0.65]
                        fputc(205,out);
                    }
                }
            }
            fclose(out);

            std::string mapmetadatafile = mapname_ + ".yaml";
            ROS_INFO("Writing map occupancy data to %s",mapmetadatafile.c_str());
            FILE* yaml = fopen(mapmetadatafile.c_str(),"w");

            /*
            resolution : 0.10000
            origin: [0.0,0.0,0.0]
            negate : 0
            occupied_thresh: 0.65
            free_thresh : 0.196
            */

            geometry_msgs::Quaternion orientation = map->info.origin.orientation;
            tf::Matrix3x3 mat(tf::Quaternion(orientation.x,orientation.y,orientation.z,orientation.w));
            double yaw,pitch,roll;
            mat.getEulerYPR(yaw,pitch,roll);

            fprintf(yaml,"image: %s\nresolution: %f\norigin: [%f,%f,%f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
                    mapdatafile.c_str(),map->info.resolution,map->info.origin.position.x,map->info.origin.position.y,yaw);
            
            fclose(yaml);

            ROS_INFO("Done\n");
            saved_map_ = true;
            return;
        }
    }

bool saved_map_;

private:
    ros::NodeHandle n;
    std::string mapname_;
    ros::Subscriber map_sub_;
};

#define USAGE "Usage:\n"\
            "map_saver_shopping -h\n"\
            "map_saver_shopping [-f <mapname>]"

int main(int argc,char **argv)
{
    ros::init(argc,argv,"map_saver_shopping");
    std::string mapname="//home//ros//robocup//map//defaultmap";

    for(int i=1;i<argc;i++)
    {
        if(!strcmp(argv[i],"-h"))
        {
            puts(USAGE);
            return 0;
        }
        else if(!strcmp(argv[i],"-f"))
        {
            if(++i < argc)
                mapname = argv[i];
            else
            {
                puts(USAGE);
                return 1;
            }
        }
        else
        {
            puts(USAGE);
            return 1;
        }
    }

    MapGenerator mg(mapname);
    while(!mg.saved_map_ && ros::ok())
        ros::spinOnce();

    return 0;

}
