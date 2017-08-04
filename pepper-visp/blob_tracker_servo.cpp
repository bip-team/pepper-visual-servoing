/**
    @file
    @author Jan Michalczyk 

    @brief Simple servo loop using blob tracker to track blobs
*/

#include "pepper_visp.h"

#include "vp_pepper_visp_config.h"

#include <visp/vpImage.h>
#include <visp3/blob/vpDot2.h>
#include <visp/vpDisplayX.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/core/vpPoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/visual_features/vpFeatureDepth.h>

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
        const std::size_t           number_of_features = 1;
        
        std::vector<vpFeaturePoint> s_current(number_of_features);
        std::vector<vpFeaturePoint> s_desired(number_of_features);
        std::vector<vpImagePoint>   image_points(number_of_features);
        std::vector<vpDot2>         blobs(number_of_features);

        vpCameraParameters camera_parameters = pepper_vs.getIntrinsicCameraParameters();
        
        // depth feature
        vpFeatureDepth current_depth_feature;
        vpFeatureDepth desired_depth_feature;

        // desired distance
        double desired_depth = 0.40; // 40[cm]
        
        // depth/area ration coeff which needs calibration 
        double depth_to_area; 

        // servo task
        vpServo task;
        task.setServo(vpServo::EYEINHAND_CAMERA);
        task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
        //task.setLambda(0.01);
        task.setLambda(0.1);
      
        // initialize trackers
        std::size_t i = 0; 
        while(true)
        {
            pepper_vs.getImage(image);
            vpDisplay::display(image);
            
            std::cout << "Choose " << number_of_features << " blob(s) to track" << std::endl;
            vpDisplay::displayText(image, vpImagePoint(10, 10), "Click in the blob to initialize the tracker", vpColor::red);
            if(vpDisplay::getClick(image, image_points[i], false))
            {
                blobs[i].initTracking(image, image_points[i]);
                vpFeatureBuilder::create(s_current[i], camera_parameters, blobs[i]);
                vpFeatureBuilder::create(s_desired[i], camera_parameters, blobs[i]);
                task.addFeature(s_current[i], s_desired[i]);
                i++;
            }

            if(i == number_of_features)
            {
                current_depth_feature.buildFrom(s_current[number_of_features - 1].get_x(),
                                                s_current[number_of_features - 1].get_y(), desired_depth, 0);
                desired_depth_feature.buildFrom(s_desired[number_of_features - 1].get_x(),
                                                s_desired[number_of_features - 1].get_y(), desired_depth, 0);

                // get depth/area coeff 
                depth_to_area = desired_depth / (1.0 / sqrt(blobs[number_of_features - 1].getArea() /
                                       (camera_parameters.get_px() * camera_parameters.get_py())));

                task.addFeature(current_depth_feature, desired_depth_feature);
                break;
            }
                
            vpDisplay::flush(image);
        }

        // servo loop
        vpColVector velocity;
        while(true)
        {
            try
            {
                double t = vpTime::measureTimeMs();
               
                pepper_vs.getImage(image);
                vpDisplay::display(image);
            
                for(i = 0; i < blobs.size(); ++i)
                {
                    blobs[i].setGraphics(true);
                    blobs[i].setGraphicsThickness(2);
                    blobs[i].track(image);
                    vpFeatureBuilder::create(s_current[i], camera_parameters, blobs[i]);
                }

                double current_depth = depth_to_area * (1.0 / sqrt(blobs[number_of_features - 1].getArea() /
                                       (camera_parameters.get_px() * camera_parameters.get_py())));
                
                current_depth_feature.buildFrom(s_current[number_of_features - 1].get_x(), s_current[number_of_features - 1].get_y(),
                                                current_depth, log(current_depth / desired_depth));

                velocity = task.computeControlLaw();
                vpServoDisplay::display(task, camera_parameters, image);

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
            }
        }

        task.kill();
    }
    catch(const std::exception& e)
    {
        std::cout << "Exception: " << e.what() << std::endl;
    }
    
    return(0);
}
