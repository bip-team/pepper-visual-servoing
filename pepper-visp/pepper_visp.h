/**
    @file
    @author Jan Michalczyk 

    @brief
*/

#pragma once

#include <iostream>
#include <string>
#include <sys/time.h>
#include <boost/shared_ptr.hpp>

#include <alerror/alerror.h>
#include <alvision/alimage.h>
#include <alcommon/albroker.h>
#include <alcommon/alproxy.h>
#include <alvision/alvisiondefinitions.h>
#include <alproxies/alvideodeviceproxy.h>

#include "vp_pepper_visp_config.h"

#include <visp/vpXmlParserHomogeneousMatrix.h>
#include <visp/vpImageConvert.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp3/blob/vpDot2.h>
#include <visp/vpDisplayX.h>
#include <visp/vpIoTools.h>
#include <visp/vpImageIo.h>
#include <visp3/core/vpMath.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/core/vpPoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/vs/vpServoDisplay.h>

namespace pepper_visp
{
    class Camera
    {
        public:
            enum Id
            {
                FOREHEAD = 0, 
                CHIN     = 1
            };
    };


    class PepperVS
    {
        public:
            PepperVS()
            {
                std::string robot_ip = "10.42.0.61";
                int robot_port       = 9559;
                Camera::Id camera_id = Camera::FOREHEAD;
                setParameters(robot_ip, robot_port, camera_id);
                initialize();
            }
            
            
            PepperVS(const std::string& robot_ip, const int robot_port, Camera::Id camera_id)
            {
                setParameters(robot_ip, robot_port, camera_id);
                initialize();
            }


            void getImage(vpImage<unsigned char>& image)
            {
                /* Retrieve an image from the camera.
                * The image is returned in the form of a container object, with the
                * following fields:
                * 0 = width
                * 1 = height
                * 2 = number of layers
                * 3 = colors space index (see alvisiondefinitions.h)
                * 4 = time stamp (seconds)
                * 5 = time stamp (micro seconds)
                * 6 = image buffer (size of width * height * number of layers)
                */

                image_ = video_proxy_->getImageRemote(client_name_);

                image_width_  = static_cast<int>(image_[0]);
                image_height_ = static_cast<int>(image_[1]);

                // Access the image buffer (6th field) and assign it to the ViSP image container
                unsigned char *image_buffer = (unsigned char*)image_[6].GetBinary();

                // Tells to ALVideoDevice that it can give back the image buffer to the
                // driver. Optional after a getImageRemote but MANDATORY after a getImageLocal.
                //video_proxy_->releaseImage(client_name_);

                image.resize(image_height_, image_width_);

                vpImageConvert::BGRToGrey(image_buffer, static_cast<unsigned char*>(image.bitmap), image_width_, image_height_);
            }


            unsigned int getImageWidth() const
            {
                return(static_cast<unsigned int>(image_width_));
            }
            
            
            unsigned int getImageHeight() const
            {
                return(static_cast<unsigned int>(image_height_));
            }
            
            
            const vpCameraParameters& getIntrinsicCameraParameters() const
            {
                return(intrinsic_camera_parameters_);
            }

            
            void callPepperController(const vpColVector& velocity_twist)
            {
                try
                {
                    std::vector<double> angular_velocity;
                    angular_velocity.push_back(velocity_twist[3]);
                    angular_velocity.push_back(velocity_twist[4]);
                    angular_velocity.push_back(velocity_twist[5]);
                    pepper_controller_proxy_->callVoid<std::vector<double> >("setTagAngularVelocity", angular_velocity); 
                }
                catch(const std::exception& e)
                {
                    std::cerr << "exception connecting to proxy: " << e.what() << std::endl;
                    throw;
                }
            }


            void writeVelocityToFile(const vpColVector& vp_col_vector)
            {
                for(std::size_t i = 0; i < vp_col_vector.size() - 1; ++i)
                {
                    octave_output_stream_ << vp_col_vector[i] << ", ";
                }
                octave_output_stream_ << vp_col_vector[vp_col_vector.size() - 1] << ";" << std::endl;
            }


            ~PepperVS()
            {
                cleanup();
            }


        private:
            void initialize()
            {
                try
                {

#ifdef PEPPER_VISP_USE_PEPPER_CONTROLLER
                    pepper_controller_proxy_.reset(new AL::ALProxy("PepperController", robot_ip_, robot_port_));  
#endif
                    
                    video_proxy_.reset(new AL::ALVideoDeviceProxy(robot_ip_, robot_port_));
                    video_proxy_->unsubscribeAllInstances(client_name_);
                    client_name_ = video_proxy_->subscribeCamera(client_name_, camera_id_, camera_resolution_, AL::kBGRColorSpace, fps_);
                    
                    image_ = video_proxy_->getImageRemote(client_name_);

                    image_width_  = static_cast<int>(image_[0]);
                    image_height_ = static_cast<int>(image_[1]);
                    
                    video_proxy_->releaseImage(client_name_);
                    
                    parseIntrinsicCameraParameters();

#ifdef PEPPER_VISP_LOG_VELOCITY
                    initializeOctaveLogger("velocity-log.m");
#endif

                }
                catch(const std::exception& e)
                {
                    std::cerr << "Exception in initialization: " << e.what() << std::endl;
                    throw;
                }
            }


            void setParameters(const std::string& robot_ip, const int robot_port, Camera::Id camera_id)
            {
                robot_ip_          = robot_ip;
                robot_port_        = robot_port;
                fps_               = 30;
                camera_name_       = "pepper_forehead_camera";
                client_name_       = "pepper_visp";
                camera_id_         = camera_id;
                camera_resolution_ = AL::kQVGA;
            }
            

            void initializeOctaveLogger(const std::string& filename)
            {
                const std::string full_path = PEPPER_VISP_LOG_FILE_PATH + filename;
                octave_output_stream_.open(full_path.c_str());
                if(!octave_output_stream_)
                {
                    throw std::runtime_error("Cannot open output file."); 
                }
                octave_output_stream_ << "velocity = " << "[" << std::endl;
            }


            void closeOctaveLogger()
            {
                octave_output_stream_ << "]" << std::endl;
                octave_output_stream_.close();
            }

            
            void parseIntrinsicCameraParameters()
            {
                vpXmlParserCamera vp_xml_parser;
                if(vp_xml_parser.parse(intrinsic_camera_parameters_, PEPPER_VISP_INTRINSIC_CAMERA_FILE,
                                camera_name_, vpCameraParameters::perspectiveProjWithDistortion,
                                image_width_, image_height_) != vpXmlParserCamera::SEQUENCE_OK)
                {
                    throw std::runtime_error("Cannot parse camera parameters."); 
                }
            }

            
            void cleanup()
            {
                try
                {
                    closeOctaveLogger();
                    if(!video_proxy_)
                    {
                        video_proxy_->unsubscribe(client_name_);
                    }
                }
                catch(...)
                {
                }
            }
       

        private:
            boost::shared_ptr<AL::ALVideoDeviceProxy> video_proxy_;
            boost::shared_ptr<AL::ALProxy>            pepper_controller_proxy_;  
            vpCameraParameters                        intrinsic_camera_parameters_;

            std::string robot_ip_;
            int         robot_port_;
            std::string client_name_;
            int         fps_;
            int         image_width_;
            int         image_height_;
            AL::ALValue image_;
            std::string camera_name_;
            int         camera_id_;
            int         camera_resolution_;

            std::ofstream octave_output_stream_;
    };
}//pepper_visp
