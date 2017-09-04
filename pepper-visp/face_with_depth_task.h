/**
    @file
    @author Jan Michalczyk 
    @copyright 2016-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief  Task minimizing error between desired and current 
            position of face and keeping face at desired distance
*/

#pragma once

#include "vp_pepper_visp_config.h"

#include "pepper_visp.h"

#include <visp/vpImage.h>
#include <visp3/blob/vpDot2.h>
#include <visp/vpDisplayX.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/core/vpPoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp/vpPixelMeterConversion.h>

#include "config-yaml/config_reader.h"

namespace pepper_visp
{
    /**
     * @brief Task minimizing error between desired and current 
     *        position of face and keeping face at desierd distance
     */
    class FaceWithDepthTask
    {
        public:
            /**
             * @brief Constructor
             *
             * @param[in] camera_parameters
             */
            FaceWithDepthTask(const vpCameraParameters& camera_parameters) : camera_parameters_(camera_parameters)                              
            {
                setDefaults();
                initialize();
            }
            
            
            /**
             * @brief Constructor
             *
             * @param[in] config_file
             * @param[in] camera_parameters
             */
            FaceWithDepthTask(const vpCameraParameters& camera_parameters, 
                              const std::string&        config_file) : camera_parameters_(camera_parameters)                              
            {
                readParameters(std::string(PEPPER_VISP_CONFIG_PATH) + config_file);
                initialize();
            }


            /**
             * @brief Initialize task
             */
            void initializeTask(const vpImage<unsigned char>& image)
            {
                desired_face_cog_.set_ij(image.getHeight()/2.0, image.getWidth()/2.0);                     

                current_s_.buildFrom(current_x_, current_y_, current_z_);
                desired_s_.buildFrom(desired_x_, desired_y_, desired_z_);
                task_.addFeature(current_s_, desired_s_);
            }


            /**
             * @brief Update task
             *
             * @param[in] desired_depth
             * @param[in] current_depth
             */
            void update(const vpImagePoint& current_face_cog) 
            {
                vpPixelMeterConversion::convertPoint(camera_parameters_, 
                                                     current_face_cog, 
                                                     current_x_, 
                                                     current_y_);
                
                current_s_.buildFrom(current_x_, current_y_, current_z_);
                
                vpPixelMeterConversion::convertPoint(camera_parameters_, 
                                                     desired_face_cog_, 
                                                     desired_x_, 
                                                     desired_y_);
                
                desired_s_.buildFrom(desired_x_, desired_y_, desired_z_);
            }


            /**
             * @brief Display servo
             *
             * @param[in] image 
             */
            void displayServo(const vpImage<unsigned char>& image) const
            {
                vpServoDisplay::display(task_, camera_parameters_, image);
            }
            

            /**
             * @brief Get velocity
             *
             * @param[in, out] velocity
             */
            void getVelocity(vpColVector& velocity)
            {
                velocity = task_.computeControlLaw();
            }
            

            /**
             * @brief Destructor
             */
            ~FaceWithDepthTask()
            {
                task_.kill();
            }

            
        private:
            /**
             * @brief Read parameters from config file
             *
             * @param[in] config_file
             */
            void readParameters(const std::string& config_file)
            {
                yaml_config::ConfigReader config_reader(config_file); 
                try
                {
                    config_reader.readScalar("face_with_depth_task", "lambda", lambda_); 
                }
                catch(const std::exception& e)
                {
                    std::cerr << "Exception in reading configuration file: " << e.what() << std::endl;
                    throw;
                }
            }


            /**
             * @brief Set default parameters
             */
            void setDefaults()
            {
                lambda_ = 0.2;
            }
            
            
            /**
             * @brief Initialize
             */
            void initialize()
            {
                desired_x_, desired_y_, current_x_, current_y_ = 0.0;
                desired_z_, current_z_                         = 0.8;
                
                task_.setLambda(lambda_);
                task_.setServo(vpServo::EYEINHAND_CAMERA);
                task_.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
            }


        private:
            vpCameraParameters camera_parameters_;
            vpFeaturePoint     current_s_;
            vpFeaturePoint     desired_s_;
            vpServo            task_;
            vpImagePoint       desired_face_cog_;

            double             lambda_;

            double current_x_, current_y_, current_z_;
            double desired_x_, desired_y_, desired_z_;
    };
}//pepper_visp
