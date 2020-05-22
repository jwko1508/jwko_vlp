/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "DEPRECATED USAGE: map_server <map> <resolution>\n" \
              "  map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

class MapServer
{
  public:
    /** Trivial constructor */
    MapServer(const std::string& fname, double res, const std::string& mapname)
    {
        double origin_gps_x_;
        double origin_gps_y_;

        std::string mapfname = "";
        double origin[3];
        int negate;
        double occ_th, free_th;
        MapMode mode = TRINARY;
        std::string frame_id;


        ros::NodeHandle n_;
      ros::NodeHandle private_nh("~");
      private_nh.param("frame_id", frame_id, std::string("odom"));


        deprecated = (res != 0);
      if (!deprecated) {
        //mapfname = fname + ".pgm";
        //std::ifstream fin((fname + ".yaml").c_str());
        std::ifstream fin(fname.c_str());
        if (fin.fail()) {
          ROS_ERROR("Map_server could not open %s.", fname.c_str());
          exit(-1);
        }
#ifdef HAVE_NEW_YAMLCPP
        // The document loading process changed in yaml-cpp 0.5.
        YAML::Node doc = YAML::Load(fin);
#else
        YAML::Parser parser(fin);
        YAML::Node doc;
        parser.GetNextDocument(doc);
#endif
        try { 
          doc["resolution"] >> res; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["negate"] >> negate; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain a negate tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["occupied_thresh"] >> occ_th; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["free_thresh"] >> free_th; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
          exit(-1);
        }
        try { 
          std::string modeS = "";
          doc["mode"] >> modeS;

          if(modeS=="trinary")
            mode = TRINARY;
          else if(modeS=="scale")
            mode = SCALE;
          else if(modeS=="raw")
            mode = RAW;
          else{
            ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
            exit(-1);
          }
        } catch (YAML::Exception) { 
          ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
          mode = TRINARY;
        }
        try { 
          doc["origin"][0] >> origin[0]; 
          doc["origin"][1] >> origin[1]; 
          doc["origin"][2] >> origin[2];

        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain an origin tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["image"] >> mapfname; 
          // TODO: make this path-handling more robust
          if(mapfname.size() == 0)
          {
            ROS_ERROR("The image tag cannot be an empty string.");
            exit(-1);
          }
          if(mapfname[0] != '/')
          {
            // dirname can modify what you pass it
            char* fname_copy = strdup(fname.c_str());
            mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
            free(fname_copy);
          }
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain an image tag or it is invalid.");
          exit(-1);
        }
      } else {
        private_nh.param("negate", negate, 0);
        private_nh.param("occupied_thresh", occ_th, 0.65);
        private_nh.param("free_thresh", free_th, 0.196);
        mapfname = fname;
        origin[0] = origin[1] = origin[2] = 0.0;
      }

        if (n_.getParam("gps/origin/x", origin_gps_x_)) {
            ROS_INFO("gps/origin/x : %f",origin_gps_x_);
        }
        else {
            ROS_INFO("Failed to get param gps/origin/x");
            origin_gps_x_ = 0.0;
        }
        if (n_.getParam("gps/origin/y", origin_gps_y_)) {
            ROS_INFO("gps/origin/y : %f",origin_gps_y_);
        }
        else {
            ROS_INFO("Failed to get param gps/origin/y");
            origin_gps_y_ = 0.0;
        }

        origin[0]-= origin_gps_x_;
        origin[1]-= origin_gps_y_;
        ROS_INFO("main map_origin_x: %f, map_origin_y: %f\n",*(origin),*(origin+1));

      ROS_INFO("Loading %s from image \"%s\"", mapname.c_str(), mapfname.c_str());
      map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin, mode);
      map_resp_.map.info.map_load_time = ros::Time::now();
      map_resp_.map.header.frame_id = frame_id;
      map_resp_.map.header.stamp = ros::Time::now();
      ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
               map_resp_.map.info.width,
               map_resp_.map.info.height,
               map_resp_.map.info.resolution);
      meta_data_message_ = map_resp_.map.info;

        std::string service_name = mapname + "_static_map";
        std::string metadata_name = mapname + "_map_metadata";
        std::string map_name = mapname + "_map";

      service = n.advertiseService(service_name, &MapServer::mapCallback, this);
      //pub = n.advertise<nav_msgs::MapMetaData>("map_metadata", 1,

      // Latched publisher for metadata
      metadata_pub = n.advertise<nav_msgs::MapMetaData>(metadata_name, 1, true);
      metadata_pub.publish( meta_data_message_ );

      // Latched publisher for data
      map_pub = n.advertise<nav_msgs::OccupancyGrid>(map_name, 1, true);
      map_pub.publish( map_resp_.map );
    }

  private:
    ros::NodeHandle n;
    ros::Publisher map_pub;
    ros::Publisher metadata_pub;
    ros::ServiceServer service;
    bool deprecated;

    /** Callback invoked when someone requests our service */
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res )
    {
      // request is empty; we ignore it

      // = operator is overloaded to make deep copy (tricky!)
      res = map_resp_;
      ROS_INFO("Sending map");

      return true;
    }

    /** The map data is cached here, to be sent out to service callers
     */
    nav_msgs::MapMetaData meta_data_message_;
    nav_msgs::GetMap::Response map_resp_;

    /*
    void metadataSubscriptionCallback(const ros::SingleSubscriberPublisher& pub)
    {
      pub.publish( meta_data_message_ );
    }
    */

};

int main(int argc, char **argv)
{
    printf("pharos_map_server Start!\n");
  ros::init(argc, argv, "map_server", ros::init_options::AnonymousName);
    ros::NodeHandle private_nh("~");

    std::string roadinfo_map;
    std::string verticalinfo_map;
    std::string drivable_map_perception;
    std::string drivable_map_planner;

    private_nh.param("roadinfo_map", roadinfo_map, std::string(""));
    private_nh.param("verticalinfo_map", verticalinfo_map, std::string(""));
    private_nh.param("drivable_map_perception", drivable_map_perception, std::string(""));
    private_nh.param("drivable_map_planner", drivable_map_planner, std::string(""));

    double res = 0.0;
  try
  {
    MapServer ms(roadinfo_map, res, "roadinfo");
    MapServer ms2(verticalinfo_map, res, "verticalinfo");
    MapServer ms3(drivable_map_perception, res, "drivable_map_perception");
    MapServer ms4(drivable_map_planner, res, "drivable_map_planner");

      ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("map_server exception: %s", e.what());
    return -1;
  }

  return 0;
}

