#include "ros/ros.h"
#include "std_msgs/String.h"

#include <visualization_msgs/MarkerArray.h>
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <tf/transform_datatypes.h>

class CSVRow{
public:
        std::string const& operator[](std::size_t index) const{
                        return m_data[index];
                }
        std::size_t size() const
                {
                        return m_data.size();
                }
        void readNextRow(std::istream& str)
                {
                        std::string         line;
                        std::getline(str, line);

                        std::stringstream   lineStream(line);
                        std::string         cell;

                        m_data.clear();
                        while(std::getline(lineStream, cell, ','))
                        {
                                m_data.push_back(cell);
                        }
                        // This checks for a trailing comma with no data after it.
                        if (!lineStream && cell.empty())
                        {
                                // If there was a trailing comma then add an empty element.
                                m_data.push_back("");
                        }
                }
private:
        std::vector<std::string>    m_data;
};



std::istream& operator>>(std::istream& str, CSVRow& data)
{
        data.readNextRow(str);
        return str;
}




bool getHeatMapColor(float value, float *red, float *green, float *blue)
{
  const int NUM_COLORS = 4;
  static float color[NUM_COLORS][3] = { {0,0,1}, {0,1,0}, {1,1,0}, {1,0,0} };
    // A static array of 4 colors:  (blue,   green,  yellow,  red) using {r,g,b} for each.

  int idx1;        // |-- Our desired color will be between these two indexes in "color".
  int idx2;        // |
  float fractBetween = 0;  // Fraction between "idx1" and "idx2" where our value is.

  if(value <= 0)      {  idx1 = idx2 = 0;            }    // accounts for an input <=0
  else if(value >= 1)  {  idx1 = idx2 = NUM_COLORS-1; }    // accounts for an input >=0
  else
  {
    value = value * (NUM_COLORS-1);        // Will multiply value by 3.
    idx1  = floor(value);                  // Our desired color will be after this index.
    idx2  = idx1+1;                        // ... and before this index (inclusive).
    fractBetween = value - float(idx1);    // Distance between the two indexes (0-1).
  }

  *red   = (color[idx2][0] - color[idx1][0])*fractBetween + color[idx1][0];
  *green = (color[idx2][1] - color[idx1][1])*fractBetween + color[idx1][1];
  *blue  = (color[idx2][2] - color[idx1][2])*fractBetween + color[idx1][2];
}


int main(int argc, char **argv){
        ros::init(argc, argv, "cliff_marker_pub");
        ros::NodeHandle n;
        ros::NodeHandle param("~");;
        std::string mapFile;

        param.param<std::string>("map_file", mapFile, "map.csv");
        ros::Publisher cliff_pub = n.advertise<visualization_msgs::MarkerArray>("cliff_marker_pub", 1000);
        ros::Rate loop_rate(10);
        ROS_INFO_STREAM(mapFile);



        std::ifstream file(mapFile.c_str());
        CSVRow row;
        std::vector<CSVRow> rows;
        while(file >> row)
        {
                rows.push_back(row);
        }




        while (ros::ok()){

                visualization_msgs::MarkerArray yaws_array_msg_clear;
                yaws_array_msg_clear.markers.resize(1);
                yaws_array_msg_clear.markers[0].header.frame_id = "world";
                yaws_array_msg_clear.markers[0].header.stamp = ros::Time::now();
                yaws_array_msg_clear.markers[0].type = visualization_msgs::Marker::ARROW;
                yaws_array_msg_clear.markers[0].action = 3;
                cliff_pub.publish(yaws_array_msg_clear);
                cliff_pub.publish(yaws_array_msg_clear);
                visualization_msgs::MarkerArray yaws_array_msg;
                yaws_array_msg.markers.resize(rows.size());
                for(int array_index=0;array_index<rows.size();array_index++){
                        yaws_array_msg.markers[array_index].header.frame_id = "world";
                        yaws_array_msg.markers[array_index].header.stamp = ros::Time::now();
                        yaws_array_msg.markers[array_index].ns = "cliff_map";
                        yaws_array_msg.markers[array_index].id = array_index + 1000000;
                        yaws_array_msg.markers[array_index].type =
                                visualization_msgs::Marker::ARROW;
                        yaws_array_msg.markers[array_index].action =
                                visualization_msgs::Marker::ADD;
                        yaws_array_msg.markers[array_index].pose.position.x = atof(rows[array_index][0].c_str());
                        yaws_array_msg.markers[array_index].pose.position.y = atof(rows[array_index][1].c_str());
                        yaws_array_msg.markers[array_index].pose.position.z = 0.5;
                        yaws_array_msg.markers[array_index].pose.orientation =
                                tf::createQuaternionMsgFromRollPitchYaw(0, 0, atof(rows[array_index][2].c_str()));
                        yaws_array_msg.markers[array_index].scale.x = atof(rows[array_index][3].c_str());
                        yaws_array_msg.markers[array_index].scale.y = 0.1;
                        yaws_array_msg.markers[array_index].scale.z = 0.1;
                        float r,g,b;
                        getHeatMapColor(atof(rows[array_index][4].c_str()),&r,&g,&b);

                        yaws_array_msg.markers[array_index].color.a = 1.0;
                        yaws_array_msg.markers[array_index].color.r = r;
                        yaws_array_msg.markers[array_index].color.b = g;
                        yaws_array_msg.markers[array_index].color.g = b;
                        array_index++;
                }
                cliff_pub.publish(yaws_array_msg);
                cliff_pub.publish(yaws_array_msg);


                ros::spinOnce();
                loop_rate.sleep();
        }


        return 0;
}
