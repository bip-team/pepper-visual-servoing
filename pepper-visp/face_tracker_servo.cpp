/**
    @file
    @author Jan Michalczyk 
    @copyright 2016-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Visual servo loop using face tracker
*/

#include <string>

#include "vp_pepper_visp_config.h"

#include "pepper_visp.h"
#include "face_tracker.h"
#include "face_with_depth_task.h"

#include "people_detector.h"

/**
 * @brief Visual servo control loop using face tracking
 *
 * @param[in] argc number of arguments
 * @param[in] argv arguments
 *
 * @return status
 */
int main(int argc, char** argv)
{
    try
    {
        // connect to the robot
        pepper_visp::PepperVS pepper_vs(PEPPER_VISP_FOREHEAD_CONFIG_FILE);
        
        // get people detector
        pepper_visp::PeopleDetector people_detector(PEPPER_VISP_FOREHEAD_CONFIG_FILE); 

        // get image display
        vpImage<unsigned char> image(pepper_vs.getImageHeight(), pepper_vs.getImageWidth());
        vpDisplayX d(image);
        vpDisplay::setTitle(image, "ViSP viewer");

        vpCameraParameters camera_parameters = pepper_vs.getIntrinsicCameraParameters();
        
        pepper_vs.getImage(image);
        
        // get tracker
        pepper_visp::FaceTracker face_tracker("haarcascade_frontalface_alt.xml");

        // get task
        const double desired_distance = 0.5;
        pepper_visp::FaceWithDepthTask face_depth_task(camera_parameters, "face_tracker_servo.yaml");
        face_depth_task.initializeTask(image, desired_distance);

        vpColVector velocity(6);
        while(true)
        {
            try
            {
                double t = vpTime::measureTimeMs();
               
                pepper_vs.getImage(image);
                vpDisplay::display(image);
            
                if(face_tracker.detectFace(image) && people_detector.personDetected())
                {
                    face_depth_task.update(face_tracker.getFaceCog(), 
                                           desired_distance,
                                           people_detector.getDistanceFromPerson());
                    face_depth_task.getVelocity(velocity);

                    face_depth_task.displayServo(image);
                    face_tracker.displayFace(image);

#ifdef PEPPER_VISP_LOG_VELOCITY
                pepper_vs.writeVelocityToFile(velocity);
#endif

#ifdef PEPPER_VISP_USE_PEPPER_CONTROLLER
                pepper_vs.callPepperController(velocity, "CameraTop_optical_frame");
#endif
                }
                else
                {

#ifdef PEPPER_VISP_LOG_VELOCITY
                pepper_vs.writeVelocityToFile(velocity);
#endif

#ifdef PEPPER_VISP_USE_PEPPER_CONTROLLER
                pepper_vs.callPepperControllerZeroVelocity("CameraTop_optical_frame");
#endif

                }

                vpDisplay::flush(image);
                
                if(vpDisplay::getClick(image, false))
                {
                    break;
                }
                
                std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
            }
            catch(const vpException& e)
            {

#ifdef PEPPER_VISP_USE_PEPPER_CONTROLLER
                pepper_vs.callPepperControllerZeroVelocity("CameraTop_optical_frame");
#endif

            }
        }
    }
    catch(const std::exception& e)
    {
        std::cout << "Exception: " << e.what() << std::endl;
    }
    
    return(0);
}
