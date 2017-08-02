/**
    @file
    @author Jan Michalczyk 

    @brief Simple servo loop for model-based tracking
*/

#include "pepper_visp.h"
#include "model_based_tracker.h"
#include "four_blobs_pattern_tracker.h"

int main(int argc, char** argv)
{
    try
    {
        // connect to the robot
        pepper_visp::PepperVS pepper_vs(PEPPER_VISP_FOREHEAD_CONFIG_FILE);

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

        while(true)
        {
            pepper_vs.getImage(image);
            vpDisplay::display(image);
            
            blobs_tracker.getObjectPose(cMo_blobs, image);  
            blobs_tracker.displayObjectFrame(cMo_blobs, image);
            
            box_tracker.track(image);
            box_tracker.getObjectPose(cMo_box);
            box_tracker.displayObjectFrame(cMo_box, image);

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
