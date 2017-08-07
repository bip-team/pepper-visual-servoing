/**
    @file
    @author Jan Michalczyk 

    @brief Visual servo loop using blob tracker and
           depth estimation
*/

#include "pepper_visp.h"

#include "vp_pepper_visp_config.h"

#include "blobs_with_depth_tracker.h"
#include "blobs_with_depth_task.h"

/**
 * @brief Visual servo control loop using blob tracking
 *        and depth estimation
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

        // get image display
        vpImage<unsigned char> image(pepper_vs.getImageHeight(), pepper_vs.getImageWidth());
        vpDisplayX d(image);
        vpDisplay::setTitle(image, "ViSP viewer");
        
        // initialize image points, features etc.
        const std::size_t number_of_blobs = 1;
        const double      desired_depth   = 0.40;

        vpCameraParameters camera_parameters = pepper_vs.getIntrinsicCameraParameters();

        pepper_vs.getImage(image);
        
        // get tracker
        pepper_visp::BlobsWithDepthTracker blobs_with_depth_tracker(number_of_blobs,
                                                                    desired_depth,
                                                                    camera_parameters);
        blobs_with_depth_tracker.initializeByClick(image);

        // get task
        pepper_visp::BlobsWithDepthTask blobs_with_depth_task(camera_parameters);
        blobs_with_depth_task.initializeTask(blobs_with_depth_tracker.getBlobs(), 
                                             blobs_with_depth_tracker.getDesiredDepth());

        // servo loop
        vpColVector velocity;
        while(true)
        {
            try
            {
                double t = vpTime::measureTimeMs();
               
                pepper_vs.getImage(image);
                vpDisplay::display(image);
            
                blobs_with_depth_tracker.track(image);

                blobs_with_depth_task.update(blobs_with_depth_tracker.getBlobs(),      
                                             blobs_with_depth_tracker.getDesiredDepth(),
                                             blobs_with_depth_tracker.getCurrentDepth());

                blobs_with_depth_task.getVelocity(velocity);
                blobs_with_depth_task.displayServo(image);

#ifdef PEPPER_VISP_LOG_VELOCITY
                pepper_vs.writeVelocityToFile(velocity);
#endif

#ifdef PEPPER_VISP_USE_PEPPER_CONTROLLER
                pepper_vs.callPepperController(velocity);
#endif

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
                pepper_vs.callPepperControllerZeroVelocity();
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
