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

#include <visp/vpImageConvert.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>

#include "yaml-cpp/yaml.h"

namespace pepper_visp
{
    /**
     * @brief Id of the camera
     */
    class CameraId
    {
        public:
            enum Id
            {
                FOREHEAD = 0, 
                CHIN     = 1
            };
    };


    /**
     * @brief Class for interfacing with pepper cameras
     */
    class PepperVS
    {
        public:
            /**
             * @brief Constructor
             */
            PepperVS()
            {
                std::string robot_ip = "10.42.0.61";
                int robot_port       = 9559;
                CameraId::Id camera_id = CameraId::FOREHEAD;
                setParameters(robot_ip, robot_port, camera_id);
                initialize();
            }
            
            
            /**
             * @brief Constructor
             *
             * @param[in] robot_ip 
             * @param[in] robot_port 
             * @param[in] camera_id
             */
            PepperVS(const std::string& robot_ip, const int robot_port, const CameraId::Id camera_id)
            {
                setParameters(robot_ip, robot_port, camera_id);
                initialize();
            }
            
            
            /**
             * @brief Constructor
             *
             * @param[in] config_file 
             */
            PepperVS(const std::string& config_file)
            {
                readParameters(config_file);
                initialize();
            }


            /**
             * @brief Get image
             *
             * @param[in, out] image 
             */
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


            /**
             * @brief Get image width
             *
             * @return image width
             */
            unsigned int getImageWidth() const
            {
                return(static_cast<unsigned int>(image_width_));
            }
            
            
            /**
             * @brief Get image height
             *
             * @return image haight
             */
            unsigned int getImageHeight() const
            {
                return(static_cast<unsigned int>(image_height_));
            }
            
            
            /**
             * @brief Get intrinsic camera parameters
             *
             * @return camera parameters
             */
            const vpCameraParameters& getIntrinsicCameraParameters() const
            {
                return(intrinsic_camera_parameters_);
            }
            
            
            /**
             * @brief Call PepperController class function with tag name
             *
             * @param[in] velocity_twist
             * @param[in] tag_name
             */
            void callPepperController(const vpColVector& velocity_twist, const std::string& tag_name)
            {
                try
                {
                    std::vector<double> velocity(velocity_twist.size());
                    for(std::size_t i = 0; i < velocity.size(); ++i)
                    {
                        velocity[i] = velocity_twist[i];
                    }
                    
                    pepper_controller_proxy_->callVoid<std::vector<double>, std::string>("setTagVelocityByName", velocity, tag_name); 
                }
                catch(const std::exception& e)
                {
                    std::cerr << "Exception calling PepperController: " << e.what() << std::endl;
                    throw;
                }
            }
            
            
            /**
             * @brief Call PepperController class with zero velocity
             *             argument
             */
            void callPepperControllerZeroVelocity(const std::string& tag_name)
            {
                std::size_t velocity_twist_size = 6;
                vpColVector zero_velocity_twist(velocity_twist_size, 0.0);
                callPepperController(zero_velocity_twist, tag_name);
            }


            /**
             * @brief Write velocity twist to file
             *
             * @param[in] velocity_twist
             */
            void writeVelocityToFile(const vpColVector& velocity_twist)
            {
                for(std::size_t i = 0; i < velocity_twist.size() - 1; ++i)
                {
                    octave_output_stream_ << velocity_twist[i] << ", ";
                }
                octave_output_stream_ << velocity_twist[velocity_twist.size() - 1] << ";" << std::endl;
            }


            /**
             * @brief Destructor
             */
            ~PepperVS()
            {
                cleanup();
            }


        private:
            /**
             * @brief Initialize
             */
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


            /**
             * @brief Set parameters
             *
             * @param[in] robot_ip
             * @param[in] robot_port
             * @param[in] camera_id
             */
            void setParameters(const std::string& robot_ip, const int robot_port, CameraId::Id camera_id)
            {
                robot_ip_          = robot_ip;
                robot_port_        = robot_port;
                fps_               = 30;
                camera_name_       = "pepper_forehead_camera";
                client_name_       = "pepper_visp";
                camera_id_         = camera_id;
                camera_resolution_ = AL::kQVGA;
            }
            
            
            /**
             * @brief Read parameters from config file
             *
             * @param[in] config_file
             */
            void readParameters(const std::string& config_file)
            {
                try
                {
                    std::ifstream fin(config_file.c_str());
                    YAML::Parser parser(fin);
                    YAML::Node config;
                    parser.GetNextDocument(config);
                    
                    config["robot_ip"]          >> robot_ip_;           
                    config["robot_port"]        >> robot_port_;         
                    config["fps"]               >> fps_;                
                    config["camera_name"]       >> camera_name_;
                    config["client_name"]       >> client_name_;
                    config["camera_id"]         >> camera_id_;          
                    config["camera_resolution"] >> camera_resolution_;  
                }
                catch(const std::exception& e)
                {
                    std::cerr << "Exception in reading configuration file: " << e.what() << std::endl;
                    throw;
                }
            }
            

            /**
             * @brief Initialize octave logger
             *
             * @param[in] filename
             */
            void initializeOctaveLogger(const std::string& filename)
            {
                const std::string full_path = PEPPER_VISP_DATA_PATH + filename;
                octave_output_stream_.open(full_path.c_str());
                if(!octave_output_stream_)
                {
                    throw std::runtime_error("Cannot open output file."); 
                }
                octave_output_stream_ << "velocity = " << "[" << std::endl;
            }


            /**
             * @brief Close octave logger
             */
            void closeOctaveLogger()
            {
                octave_output_stream_ << "]" << std::endl;
                octave_output_stream_.close();
            }

            
            /**
             * @brief Parse intrinsic camera parameters
             */
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

            
            /**
             * @brief cleanup
             */
            void cleanup()
            {
                try
                {

#ifdef PEPPER_VISP_LOG_VELOCITY
                    closeOctaveLogger();
#endif                    

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
