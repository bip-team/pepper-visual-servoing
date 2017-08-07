/**
    @file
    @author Jan Michalczyk 

    @brief  Blob(s) tracker with depth reconstruction
*/

#pragma once

#include "pepper_visp.h"

#include "vp_pepper_visp_config.h"

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
            BlobsWithDepthTracker(const std::size_t  number_of_features,
                                  const double       desired_depth,
                                  vpCameraParameters camera_parameters) : number_of_features_(number_of_features),
                                                                          desired_depth_(desired_depth),
                                                                          camera_parameters_(camera_parameters)                              
            {
                initialize();
            }


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


            void track(const vpImage<unsigned char>& image)
            {
                for(std::size_t i = 0; i < blobs_.size(); ++i)
                {
                    blobs_[i].track(image);
                }
            }


            double getCurrentDepth() const
            {
                double current_depth = depth_to_area_ * (1.0 / sqrt(blobs_[number_of_features_ - 1].getArea() /
                                       (camera_parameters_.get_px() * camera_parameters_.get_py())));
                
                return(current_depth);
            }

            
            double getDesiredDepth() const
            {
                return(desired_depth_);
            }


            const std::vector<vpDot2>& getBlobs() const
            {
                return(blobs_);
            }


        private:
            void initialize()
            {
                image_points_.resize(number_of_features_);
                blobs_.resize(number_of_features_);
            }


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