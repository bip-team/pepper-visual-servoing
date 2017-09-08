/**
    @file
    @author Jan Michalczyk 
    @copyright 2016-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Visual servo loop using model-based tracking
*/

#include "pepper-visp/vp_pepper_visp_config.h"

#include "pepper-visp/pepper_visp.h"
#include "pepper-visp/model_based_tracker.h"
#include "pepper-visp/four_blobs_pattern_tracker.h"
#include "pepper-visp/frame_aligner_task.h"

/**
 * @brief Control loop for aligning two estimated
 *        frames using PBVS
 *        (ex. tracked hand on tracked object)
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
        pepper_visp::PepperVS pepper_vs(PEPPER_VISP_CHIN_CONFIG_FILE);

        // object pose in camera frame
        vpHomogeneousMatrix cMo_box, cMo_blobs;
        vpCameraParameters  camera_parameters = pepper_vs.getIntrinsicCameraParameters();

        // get image display
        vpImage<unsigned char> image(pepper_vs.getImageHeight(), pepper_vs.getImageWidth());
        vpDisplayX d(image);
        vpDisplay::setTitle(image, "ViSP viewer");
        
        pepper_vs.getImage(image);

        // get trackers
        pepper_visp::FourBlobsPatternTracker blobs_tracker(camera_parameters);
        blobs_tracker.initializeByClick(image);

        pepper_visp::ModelBasedTracker box_tracker(PEPPER_VISP_DATA_PATH,
                                                   camera_parameters,
                                                   "greenbox.cao");
        box_tracker.initializeByClick(image, "greenbox.init");

        // servo task initialization
        double lambda = 0.1;
        pepper_visp::FrameAlignerTask frame_aligner_task(lambda);

        vpColVector velocity(6);
        while(true)
        {
            pepper_vs.getImage(image);
            vpDisplay::display(image);
            
            blobs_tracker.getObjectPose(cMo_blobs, image);  
            blobs_tracker.displayObjectFrame(cMo_blobs, image);

            box_tracker.track(image);
            box_tracker.getObjectPose(cMo_box);
            box_tracker.displayObjectFrame(cMo_box, image);

            // fix frame for testing
            //cMo_box = cMo_blobs;
            //vpTranslationVector t = cMo_box.getTranslationVector();
            //t[0] = t[0] - 10.0;
            //t[1] = t[1] - 10.0;
            //cMo_box.insert(t);

            frame_aligner_task.update(cMo_blobs, cMo_box);
            frame_aligner_task.getVelocity(velocity);

#ifdef PEPPER_VISP_LOG_VELOCITY
            pepper_vs.writeVelocityToFile(velocity);
#endif

            vpDisplay::displayText(image, 10, 10, "A click to exit...", vpColor::red);
            vpDisplay::flush(image);

            if(vpDisplay::getClick(image, false))
            {
                break;    
            }
        } 
    }
    catch(const std::exception& e)
    {
        std::cout << "Exception: " << e.what() << std::endl;
    }
    
    return(0);
}
