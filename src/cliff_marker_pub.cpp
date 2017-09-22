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

class CSVRow
{
public:
        std::string const& operator[](std::size_t index) const
                {
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

int main(int argc, char **argv){
        ros::init(argc, argv, "cliff_marker_pub");
        ros::NodeHandle n;

        ros::Publisher cliff_pub = n.advertise<visualization_msgs::MarkerArray>("cliff_marker_pub", 1000);
        ros::Rate loop_rate(10);



        std::ifstream file("/home/tzkr/matlab_workspace/cs_toolbox/cs_toolbox_timless/map_ILIAD.csv");
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
                        yaws_array_msg.markers[array_index].scale.x = 0.5;
                        yaws_array_msg.markers[array_index].scale.y = 0.1;
                        yaws_array_msg.markers[array_index].scale.z = 0.1;
                        yaws_array_msg.markers[array_index].color.a = 1.0;
                        yaws_array_msg.markers[array_index].color.r = 0.0;
                        yaws_array_msg.markers[array_index].color.b = 0.0;
                        yaws_array_msg.markers[array_index].color.g = 1.0;
                        array_index++;
                }
                cliff_pub.publish(yaws_array_msg);
                cliff_pub.publish(yaws_array_msg);


                ros::spinOnce();
                loop_rate.sleep();
        }


        return 0;
}
