/**
    @file
    @author Jan Michalczyk 

    @brief
*/

#pragma once

#include <visp/vpCameraParameters.h>
#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/vision/vpPose.h>

namespace pepper_visp
{
    /**
     * @brief Class used for pose estimation 
     *        from four blobs of given size and
     *        distribution
     */
    class FourBlobsPatternTracker 
    {
        public:
            /**
             * @brief Constructor
             *
             * @param[in] camera_parameters
             */
            FourBlobsPatternTracker(const vpCameraParameters& camera_parameters) : camera_parameters_(camera_parameters)
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
                std::size_t clicks            = 0;
                bool        blobs_initialized = false;
                while(!blobs_initialized)
                {
                    vpDisplay::display(image);
                    vpDisplay::displayText(image, vpImagePoint(10, 10), "Click four blobs to initialize the tracker", vpColor::red);
                    
                    vpDisplay::flush(image);
                    
                    blobs_[clicks].initTracking(image);
                    clicks++;

                    if(clicks == blobs_.size())
                    {
                        blobs_initialized = true;
                    }
                }
            }

            
            /**
             * @brief Get object pose in the camera frame
             *
             * @param[in, out] cMo object pose in the camera frame 
             * @param[in]      image
             */
            void getObjectPose(vpHomogeneousMatrix&          cMo, 
                               const vpImage<unsigned char>& image)
            {
                for(std::size_t i = 0; i < blobs_.size(); ++i)
                {
                    blobs_[i].setGraphics(true);
                    blobs_[i].track(image);
                }
                
                vpPose pose;
                double cog_x, cog_y;
                for(std::size_t i = 0; i < reference_points_.size(); ++i)
                {
                    vpPixelMeterConversion::convertPoint(camera_parameters_, blobs_[i].getCog(), cog_x, cog_y);
                    reference_points_[i].set_x(cog_x);
                    reference_points_[i].set_y(cog_y);
                    pose.addPoint(reference_points_[i]);
                }

                if(!tracker_initialized_)
                {
                    vpHomogeneousMatrix cMo_dem;
                    vpHomogeneousMatrix cMo_lag;
                    pose.computePose(vpPose::DEMENTHON, cMo_dem);
                    pose.computePose(vpPose::LAGRANGE, cMo_lag);
                    double residual_dem = pose.computeResidual(cMo_dem);
                    double residual_lag = pose.computeResidual(cMo_lag);
                    if(residual_dem < residual_lag)
                    {
                        cMo = cMo_dem;
                    }
                    else
                    {
                        cMo = cMo_lag;
                    }
                    
                    tracker_initialized_ = true;
                }
                
                pose.computePose(vpPose::VIRTUAL_VS, cMo);
            }
            

            /**
             * @brief Display frame (pose) of tracked object
             *
             * @param[in] cMo
             * @param[in] image
             */
            void displayObjectFrame(const vpHomogeneousMatrix&    cMo,
                                    const vpImage<unsigned char>& image) const
            {
                vpDisplay::displayFrame(image, cMo, camera_parameters_, 0.05, vpColor::none);
            }


        private:
            /**
             * @brief Initialize
             */
            void initialize()
            {
                std::size_t number_of_blobs = 4;
                blobs_.resize(number_of_blobs);
                reference_points_.resize(number_of_blobs);
                tracked_points_.resize(number_of_blobs);
                
                // order important when initializing
                reference_points_[0] = vpPoint(-0.03, -0.03, 0);
                reference_points_[1] = vpPoint( 0.03, -0.03, 0);
                reference_points_[2] = vpPoint( 0.03,  0.03, 0);
                reference_points_[3] = vpPoint(-0.03,  0.03, 0);

                tracker_initialized_ = false;
            }


        private:
            bool                 tracker_initialized_;
            std::vector<vpPoint> reference_points_;
            std::vector<vpPoint> tracked_points_;
            std::vector<vpDot2>  blobs_;
            vpCameraParameters   camera_parameters_;
    };
}//pepper_visp
