/**
    @file
    @author Jan Michalczyk 
    @copyright 2016-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief  Blob(s) tracker with depth reconstruction
*/

#pragma once

#include <visp/vpImage.h>
#include <visp3/blob/vpDot2.h>
#include <visp/vpDisplayX.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp/vpCameraParameters.h>

namespace pepper_visp
{
    /**
     * @brief Blob(s) tracker with depth reconstruction
     */
    class BlobsWithDepthTracker
    {
        public:
            /**
             * @brief Constructor
             *
             * @param[in] number_of_features 
             * @param[in] desired_depth
             * @param[in] camera_parameters
             */
            BlobsWithDepthTracker(const std::size_t         number_of_features,
                                  const double              desired_depth,
                                  const vpCameraParameters& camera_parameters) : number_of_features_(number_of_features),
                                                                                 desired_depth_(desired_depth),
                                                                                 camera_parameters_(camera_parameters)                              
            {
                initialize();
            }


            /**
             * @brief Initialize tracker by click
             *
             * @param[in] image 
             */
            void initializeByClick(const vpImage<unsigned char>& image)
            {
                vpDisplay::display(image);
                
                std::cout << "Choose " << number_of_features_ << " blob(s) to track" << std::endl;
                vpDisplay::displayText(image, vpImagePoint(10, 10), "Click in the blob to initialize the tracker", vpColor::red);
                vpDisplay::flush(image);
                    
                std::size_t i = 0; 
                while(true)
                {
                    if(vpDisplay::getClick(image, image_points_[i], false))
                    {
                        blobs_[i].setGraphics(true);
                        blobs_[i].setGraphicsThickness(2);
                        blobs_[i].initTracking(image, image_points_[i]);
                        i++;
                    }

                    if(i == number_of_features_)
                    {
                        break;
                    }
                }

                computeDepthToAreaCoeff();
            }


            /**
             * @brief Track features
             *
             * @param[in] image
             */
            void track(const vpImage<unsigned char>& image)
            {
                for(std::size_t i = 0; i < blobs_.size(); ++i)
                {
                    blobs_[i].track(image);
                }
            }


            /**
             * @brief Get current depth
             *
             * @return current_depth
             */
            double getCurrentDepth() const
            {
                return(depth_to_area_ * (1.0 / sqrt(blobs_[number_of_features_ - 1].getArea() /
                                       (camera_parameters_.get_px() * camera_parameters_.get_py()))));
            }

            
            /**
             * @brief Get desired depth
             *
             * @return desired_depth
             */
            double getDesiredDepth() const
            {
                return(desired_depth_);
            }


            /**
             * @brief Get blobs
             *
             * @return blobs
             */
            const std::vector<vpDot2>& getBlobs() const
            {
                return(blobs_);
            }


        private:
            /**
             * @brief Initialize
             */
            void initialize()
            {
                image_points_.resize(number_of_features_);
                blobs_.resize(number_of_features_);
            }


            /**
             * @brief Compute depth to area coefficient
             */
            void computeDepthToAreaCoeff()
            {
                depth_to_area_ = desired_depth_ / (1.0 / sqrt(blobs_[number_of_features_ - 1].getArea() /
                                       (camera_parameters_.get_px() * camera_parameters_.get_py())));
            }


        private:
            std::size_t number_of_features_;
            // desired distance in m
            double      desired_depth_;
            // depth/area ration coeff which needs calibration 
            double      depth_to_area_; 
            
            vpCameraParameters        camera_parameters_;
            
            std::vector<vpImagePoint> image_points_;
            std::vector<vpDot2>       blobs_;
            
    };
}//pepper_visp
