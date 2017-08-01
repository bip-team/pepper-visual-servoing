/**
    @file
    @author Jan Michalczyk 

    @brief Simple servo loop for model-based tracking
*/

#include "pepper_visp.h"
#include "model_based_tracker.h"

int main(int argc, char** argv)
{
    try
    {
        // connect to the robot
        pepper_visp::PepperVS pepper_vs(PEPPER_VISP_FOREHEAD_CONFIG_FILE);

        // object pose in camera frame
        vpHomogeneousMatrix cMo;
        vpCameraParameters  camera_parameters =
                                pepper_vs.getIntrinsicCameraParameters();

        // get image display
        vpImage<unsigned char> image(pepper_vs.getImageHeight(), pepper_vs.getImageWidth());
        vpDisplayX d(image);
        vpDisplay::setTitle(image, "ViSP viewer");
        
        pepper_vs.getImage(image);
        vpDisplay::display(image);

        // get tracker
        pepper_visp::ModelBasedTracker tracker(PEPPER_VISP_DATA_PATH,
                                               camera_parameters,
                                               "greenbox.cao");
        
        tracker.initializeByClick(image, "greenbox.init");

        while(true)
        {
            pepper_vs.getImage(image);
            vpDisplay::display(image);
            
            tracker.track(image);
            tracker.getObjectPose(cMo);
            tracker.displayObjectFrame(image, cMo);
            
            vpDisplay::displayFrame(image, cMo, camera_parameters, 0.025, vpColor::none, 3);
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
