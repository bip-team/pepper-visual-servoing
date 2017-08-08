/**
    @file
    @author Jan Michalczyk 

    @brief  Task minimizing error between desired and current 
            position of blob(s) and keeping blob at desierd distance
*/

#pragma once

#include "pepper_visp.h"

#include "vp_pepper_visp_config.h"

#include <visp/vpImage.h>
#include <visp3/blob/vpDot2.h>
#include <visp/vpDisplayX.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/core/vpPoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/visual_features/vpFeatureDepth.h>

namespace pepper_visp
{
    /**
     * @brief Task minimizing error between desired and current 
     *        position of blob(s) and keeping blob at desierd distance
     */
    class BlobsWithDepthTask
    {
        public:
            /**
             * @brief Constructor
             *
             * @param[in] camera_parameters
             */
            BlobsWithDepthTask(vpCameraParameters camera_parameters) : camera_parameters_(camera_parameters)                              
            {
                initialize();
            }


            /**
             * @brief Initialize task
             *
             * @param[in] blobs 
             * @param[in] desired_depth
             */
            void initializeTask(const std::vector<vpDot2>& blobs, const double desired_depth)
            {
                s_current_.resize(blobs.size());
                s_desired_.resize(blobs.size());
                for(std::size_t i = 0; i < blobs.size(); ++i)
                {
                    vpFeatureBuilder::create(s_current_[i], camera_parameters_, blobs[i]);
                    vpFeatureBuilder::create(s_desired_[i], camera_parameters_, blobs[i]);
                    task_.addFeature(s_current_[i], s_desired_[i]);
                }
                
                current_depth_feature_.buildFrom(s_current_[blobs.size() - 1].get_x(),
                                                 s_current_[blobs.size() - 1].get_y(), desired_depth, 0);
                desired_depth_feature_.buildFrom(s_desired_[blobs.size() - 1].get_x(),
                                                 s_desired_[blobs.size() - 1].get_y(), desired_depth, 0);

                task_.addFeature(current_depth_feature_, desired_depth_feature_);
            }


            /**
             * @brief Update task
             *
             * @param[in] blobs 
             * @param[in] desired_depth
             * @param[in] current_depth
             */
            void update(const std::vector<vpDot2>& blobs, const double desired_depth, const double current_depth)
            {
                for(std::size_t i = 0; i < blobs.size(); ++i)
                {
                    vpFeatureBuilder::create(s_current_[i], camera_parameters_, blobs[i]);
                }
                
                current_depth_feature_.buildFrom(s_current_[blobs.size() - 1].get_x(), s_current_[blobs.size() - 1].get_y(),
                                                current_depth, log(current_depth / desired_depth));
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
            ~BlobsWithDepthTask()
            {
                task_.kill();
            }

            
        private:
            /**
             * @brief Initialize
             */
            void initialize()
            {
                task_.setServo(vpServo::EYEINHAND_CAMERA);
                task_.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
                task_.setLambda(0.01);
                //task.setLambda(0.1);
            }


        private:
            vpCameraParameters          camera_parameters_;
            std::vector<vpFeaturePoint> s_current_;
            std::vector<vpFeaturePoint> s_desired_;
            vpFeatureDepth              current_depth_feature_;
            vpFeatureDepth              desired_depth_feature_;
            vpServo                     task_;
    };
}//pepper_visp
