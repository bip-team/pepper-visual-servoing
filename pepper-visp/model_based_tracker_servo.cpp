/**
    @file
    @author Jan Michalczyk 

    @brief Simple servo loop for model-based tracking
*/

#include "pepper_visp.h"

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/mbt/vpMbEdgeTracker.h>
#include <visp3/mbt/vpMbEdgeKltTracker.h>
#include <visp3/io/vpVideoReader.h>

#include <boost/pointer_cast.hpp>

int main(int argc, char** argv)
{
    try
    {
        // connect to the robot
        pepper_visp::PepperVS pepper_vs(PEPPER_VISP_FOREHEAD_CONFIG_FILE);

        // get intrinsic parameters
        vpCameraParameters camera_parameters = pepper_vs.getIntrinsicCameraParameters();

        // object pose in camera frame
        vpHomogeneousMatrix cMo;

        // get image display
        vpImage<unsigned char> image(pepper_vs.getImageHeight(), pepper_vs.getImageWidth());
        vpDisplayX d(image);
        vpDisplay::setTitle(image, "ViSP viewer");
        
        // get tracker
        boost::shared_ptr<vpMbEdgeKltTracker> tracker(new vpMbEdgeKltTracker);

            pepper_vs.getImage(image);
            vpDisplay::display(image);
        
        // tracker settings
        vpMe moving_edge;
        moving_edge.setMaskSize(5);
        moving_edge.setMaskNumber(180);
        moving_edge.setRange(8);
        moving_edge.setThreshold(10000);
        moving_edge.setMu1(0.5);
        moving_edge.setMu2(0.5);
        moving_edge.setSampleStep(4);
        boost::dynamic_pointer_cast<vpMbEdgeTracker>(tracker)->setMovingEdge(moving_edge);        

        vpKltOpencv klt_settings;
        klt_settings.setMaxFeatures(300);
        klt_settings.setWindowSize(5);
        klt_settings.setQuality(0.015);
        klt_settings.setMinDistance(8);
        klt_settings.setHarrisFreeParameter(0.01);
        klt_settings.setBlockSize(3);
        klt_settings.setPyramidLevels(3);
        boost::dynamic_pointer_cast<vpMbKltTracker>(tracker)->setKltOpencv(klt_settings);
        boost::dynamic_pointer_cast<vpMbKltTracker>(tracker)->setKltMaskBorder(5);
      
        camera_parameters.initPersProjWithoutDistortion(839, 839, 325, 243);
        tracker->setCameraParameters(camera_parameters);
        
        tracker->loadModel(std::string(PEPPER_VISP_DATA_PATH) + "greenbox.cao");
        tracker->setDisplayFeatures(true);
        tracker->initClick(image, std::string(PEPPER_VISP_DATA_PATH) + "greenbox.init", true);

        while(true)
        {
            pepper_vs.getImage(image);
            vpDisplay::display(image);
            tracker->track(image);
            tracker->getPose(cMo);
            tracker->getCameraParameters(camera_parameters);
            tracker->display(image, cMo, camera_parameters, vpColor::red, 2, true);
            vpDisplay::displayFrame(image, cMo, camera_parameters, 0.025, vpColor::none, 3);
            vpDisplay::displayText(image, 10, 10, "A click to exit...", vpColor::red);
            vpDisplay::flush(image);

            if(vpDisplay::getClick(image, false))
            {
                break;    
            }
        } 
        
        vpDisplay::getClick(image);
    }
    catch(const std::exception& e)
    {
        std::cout << "Exception: " << e.what() << std::endl;
    }
    
    return(0);
}
