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
            FaceTracker(const std::string& classifier_filename)
            {
                initialize(std::string(PEPPER_VISP_DATA_PATH) + classifier_filename);
            }


            /**
             * @brief Detect face
             *
             * @param[in] image
             * @return    face detected
             */
            bool detectFace(const vpImage<unsigned char>& image)
            {
                bool face_detected = face_detector_.detect(image);
                if(face_detector_.getNbObjects() > 1)
                {
                    throw(std::runtime_error("More than one face detected"));
                }

                return(face_detected);
            }
            

            /**
             * @brief Display rectangle around detected face
             *
             * @param[in] image
             */
            void displayFace(const vpImage<unsigned char>& image)
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


        private:
            vpDetectorFace face_detector_;
    };
}//pepper_visp
