/**
    @file
    @author Jan Michalczyk 
    @copyright 2016-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Face tracker
*/

#pragma once

#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp3/detection/vpDetectorFace.h>

namespace pepper_visp
{
    /**
     * @brief Class used for face tracking 
     */
    class FaceTracker 
    {
        public:
            /**
             * @brief Constructor
             *
             * @param[in] classifier_filename
             */
            FaceTracker(const std::string& classifier_filename, 
                        const vpCameraParameters& camera_parameters,
                        const double desired_depth) : camera_parameters_(camera_parameters), desired_depth_(desired_depth)
            {
                initialize(std::string(PEPPER_VISP_DATA_PATH) + classifier_filename);
            }


            /**
             * @brief Detect face
             *
             * @param[in] image
             * @return    face detected
             */
            const bool detectFace(const vpImage<unsigned char>& image)
            {
                return(face_detector_.detect(image));
            }


            /**
             * @brief Initialize tracker
             *
             * @param[in] image
             * @return    initialized
             */
            const bool initializeTracker(const vpImage<unsigned char>& image)
            {
                if(detectFace(image))
                {
                    computeDepthToAreaCoeff();

                    return(true);
                }

                return(false);
            }
            
            
            /**
             * @brief Get current depth
             *
             * @return current_depth
             */
            double getCurrentDepth() const
            {
                return(depth_to_area_ * (1.0 / sqrt((face_detector_.getBBox(0).getWidth() *
                                               face_detector_.getBBox(0).getHeight()) / (camera_parameters_.get_px() *
                                                          camera_parameters_.get_py()))));
            }
            
            
            /**
             * @brief Get face center of gravity
             *
             * @return    face cog
             */
            const vpImagePoint getFaceCog() const
            {
                return(face_detector_.getCog(0));
            }
            

            /**
             * @brief Display rectangle around detected face
             *
             * @param[in] image
             */
            void displayFace(const vpImage<unsigned char>& image) const
            {
                vpDisplay::displayRectangle(image, face_detector_.getBBox(0), vpColor::green, false, 4);
            }


        private:
            /**
             * @brief Initialize
             *
             * @param[in] classifier_filename
             */
            void initialize(const std::string& classifier_filename)
            {
                face_detector_.setCascadeClassifierFile(classifier_filename);
            }
            
            
            /**
             * @brief Compute depth to area coefficient
             */
            void computeDepthToAreaCoeff()
            {
                depth_to_area_ = desired_depth_ / (1.0 / sqrt((face_detector_.getBBox(0).getWidth() *
                                               face_detector_.getBBox(0).getHeight()) / (camera_parameters_.get_px() *
                                                          camera_parameters_.get_py())));
            }


        private:
            vpDetectorFace     face_detector_;
            vpCameraParameters camera_parameters_;
            double             desired_depth_;
            double             depth_to_area_;
    };
}//pepper_visp
