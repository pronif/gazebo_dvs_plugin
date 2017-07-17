/**---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
 * This file is part of the Neurorobotics Platform software
 * Copyright (C) 2014,2015,2016,2017 Human Brain Project
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * ---LICENSE-END**/
/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/*
 * IMPLEMENTATION INSPIRED BY
 * https://github.com/PX4/sitl_gazebo/blob/master/src/gazebo_opticalFlow_plugin.cpp
 */

#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>

#include <ros/ros.h>
#include <ros/console.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/util/system.hh>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <gazebo_dvs_plugin/dvs_plugin.hpp>

//#include <cmath>

using namespace std;
using namespace cv;

namespace gazebo
{
    // Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(DvsPlugin)
    
    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    DvsPlugin::DvsPlugin()
    : SensorPlugin(), width(0), height(0), depth(0)
    {
        initialized=false;
        save_csv = false;
        csv_address = "/home/events.csv";
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    DvsPlugin::~DvsPlugin()
    {
        this->parentSensor.reset();
        this->camera.reset();
        myfile.close();
        ROS_ERROR("THIS IS THE DESTRUCTOR");
    }
    
    bool sortByTime(const dvs_msgs::Event &lhs, const dvs_msgs::Event &rhs) { return lhs.ts < rhs.ts; }
    
    void DvsPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
        if (!_sensor)
            gzerr << "Invalid sensor pointer." << endl;
        
#if GAZEBO_MAJOR_VERSION >= 7
        this->parentSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
        this->camera = this->parentSensor->Camera();
#else
        this->parentSensor = boost::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
        this->camera = this->parentSensor->GetCamera();
#endif
        
        if (!this->parentSensor)
        {
            gzerr << "DvsPlugin not attached to a camera sensor." << endl;
            return;
        }
        
#if GAZEBO_MAJOR_VERSION >= 7
        this->width = this->camera->ImageWidth();
        this->height = this->camera->ImageHeight();
        this->depth = this->camera->ImageDepth();
        this->format = this->camera->ImageFormat();
#else
        this->width = this->camera->GetImageWidth();
        this->height = this->camera->GetImageHeight();
        this->depth = this->camera->GetImageDepth();
        this->format = this->camera->GetImageFormat();
#endif
        
        if (_sdf->HasElement("robotNamespace"))
            namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        else
            gzwarn << "[gazebo_ros_dvs_camera] Please specify a robotNamespace." << endl;
        
        node_handle_ = ros::NodeHandle(namespace_);
        
        string sensorName = "";
        if (_sdf->HasElement("cameraName"))
            sensorName = _sdf->GetElement("cameraName")->Get<std::string>() + "/";
        else
            gzwarn << "[gazebo_ros_dvs_camera] Please specify a cameraName." << endl;
        
        string topicName = "events";
        if (_sdf->HasElement("eventsTopicName"))
            topicName = _sdf->GetElement("eventsTopicName")->Get<std::string>();
        
        const string topic = sensorName + topicName;
        
        if (_sdf->HasElement("eventThreshold"))
            this->event_threshold = _sdf->GetElement("eventThreshold")->Get<float>();
        else
            gzwarn << "[gazebo_ros_dvs_camera] Please specify a DVS threshold." << endl;
        
        //saving settings
        if (_sdf->HasElement("save_csv"))
            this->save_csv = _sdf->GetElement("save_csv")->Get<bool>();
        
        if (_sdf->HasElement("csv_address"))
            this->csv_address = _sdf->GetElement("csv_address")->Get<std::string>();
        
        event_pub_ = node_handle_.advertise<dvs_msgs::EventArray>(topic, 10, 10.0);
        
        this->newFrameConnection = this->camera->ConnectNewImageFrame(
                                                                      boost::bind(&DvsPlugin::OnNewFrame, this, _1, this->width, this->height, this->depth, this->format));
        
        this->parentSensor->SetActive(true);
        
        if (save_csv)
            myfile.open (csv_address);
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    // Update the controller
    void DvsPlugin::OnNewFrame(const unsigned char *_image,
                               unsigned int _width, unsigned int _height, unsigned int _depth,
                               const std::string &_format)
    {
#if GAZEBO_MAJOR_VERSION >= 7
        _image = this->camera->ImageData(0);
#else
        _image = this->camera->GetImageData(0);
#endif
        
        /*
         #if GAZEBO_MAJOR_VERSION >= 7
         float rate = this->camera->RenderRate();
         #else
         float rate = this->camera->GetRenderRate();
         #endif
         if (!isfinite(rate))
         rate =  30.0;
         float dt = 1.0 / rate;
         */
        
        // convert given frame to opencv image
        cv::Mat input_image(_height, _width, CV_8UC3);
        input_image.data = (uchar*)_image;
        
        // color to grayscale
        cv::Mat curr_image_rgb(_height, _width, CV_8UC3);
        cvtColor(input_image, curr_image_rgb, CV_RGB2BGR);
        cvtColor(curr_image_rgb, input_image, CV_BGR2GRAY);
        
        cv::Mat curr_image = input_image;
        
        
        if (!initialized) {
            initialized=true;
            // TODO a single dimensional array with indexing function is much faster
            sbuf = new double*[_width];
            for (int x=0;x<_width;++x)
                sbuf[x] = new double[_height];
            
            prev = new double*[_width];
            for (int x=0;x<_width;++x)
                prev[x] = new double[_height];
            
            //initialize sbuf with the logarithm of the intensity of the first image
            int index=0;
            for (int x=0;x<_width;++x)
                for (int y=0;y<_height;++y){
                    sbuf[x][y]=std::log(1.0+curr_image.at<uchar>(y,x));
                    index++;
                }
            
            //initialize previous image
            for (int x=0;x<_width;++x)
                for (int y=0;y<_height;++y){
                    prev[x][y]=curr_image.at<uchar>(y,x);
                }
        }
        
        
        /* TODO any encoding configuration should be supported
         try {
         cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*_image, sensor_msgs::image_encodings::BGR8);
         std::cout << "Image: " << std::endl << " " << cv_ptr->image << std::endl << std::endl;
         }
         catch (cv_bridge::Exception& e)
         {
         ROS_ERROR("cv_bridge exception %s", e.what());
         std::cout << "ERROR";
         }
         */
        
        assert(_height == height && _width == width);
        if (this->has_last_image)
        {
            time_curr = ros::Time::now();
            this->processDelta(&curr_image);
        }
        this->has_last_image = true;
    }
    
    void DvsPlugin::processDelta(cv::Mat *curr_image)
    {
        double curr[curr_image->cols + 1][curr_image->rows + 1];
        double theta = event_threshold;
        
        std::vector<dvs_msgs::Event> events;
        
        for(int i=0; i<curr_image->rows; i++) {
            for(int j=0; j<curr_image->cols; j++)  {
                curr[i][j] = std::log(1.0+curr_image->at<uchar>(i, j));
            }
        }
        
        double time = time_curr.toSec();
        double t;
        // loop over all the pixels
        for(int i=0; i<curr_image->rows; i++) {
            for(int j=0; j<curr_image->cols; j++)  {
                
                int max_iter = 0;
                while (sbuf[i][j]+theta<curr[i][j]) {
                    sbuf[i][j] += theta;
                    t = ((time-time_prev.toSec())/(curr[i][j]-prev[i][j])) *(sbuf[i][j]-prev[i][j]) + time_prev.toSec();
                    fillEvent(t, j, i, 1, events);
                    if (max_iter>10)
                        break;
                }
                
                max_iter = 0;
                while (curr[i][j]<sbuf[i][j]-theta) {
                    sbuf[i][j] -= theta;
                    t = ((time-time_prev.toSec())/(curr[i][j]-prev[i][j])) *(sbuf[i][j]-prev[i][j]) + time_prev.toSec();
                    fillEvent(t, j, i, 0, events);
                    if (max_iter>10)
                        break;
                }
                
            }
        }
        
        //update persistent variables
        time_prev = time_curr;
        for(int i=0; i<curr_image->rows; i++) {
            for(int j=0; j<curr_image->cols; j++)  {
                prev[i][j] = curr[i][j];
            }
        } 
        
        // order the events
        //sort(events.begin(), events.end(), sortByTime);
        for(int i=0; i<events.size(); i++) {
            int pol = events[i].polarity;
            myfile << events[i].ts.toSec() << "," << events[i].x << "," << events[i].y << "," << pol << "\n";
            myfile.flush();	 
        } 
        
        this->publishEvents(&events);
    }
    
    void DvsPlugin::fillEvent(double t, int x, int y, bool polarity, std::vector<dvs_msgs::Event>& events)
    {
        ros::Time time_stamp = ros::Time(t);
        
        dvs_msgs::Event event;
        event.x = x;
        event.y = y;
        event.ts = time_stamp;
        event.polarity = polarity;
        events.push_back(event);
    }
    
    void DvsPlugin::publishEvents(std::vector<dvs_msgs::Event> *events)
    {
        if (events->size() > 0)
        {
            dvs_msgs::EventArray msg;
            msg.events.clear();
            msg.events.insert(msg.events.end(), events->begin(), events->end());
            msg.width = width;
            msg.height = height;
            
            // TODO what frame_id is adequate?
            msg.header.frame_id = namespace_;
            msg.header.stamp = ros::Time::now();
            
            event_pub_.publish(msg);
        }
    }
    
}
