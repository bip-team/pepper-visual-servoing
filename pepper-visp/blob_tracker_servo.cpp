/**
    @file
    @author Jan Michalczyk 

    @brief Simple servo loop using blob tracker to track blobs
*/

#include "pepper_visp.h"

int main(int argc, char** argv)
{
    try
    {
        // connect to the robot
        pepper_visp::PepperVS pepper_vs("10.42.0.61", 9559, pepper_visp::CameraId::FOREHEAD);

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

        // servo task
        vpServo task;
        task.setServo(vpServo::EYEINHAND_CAMERA);
        // corresponds to ~300ms loop delay
        task.setLambda(3.0);
      
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
                vpFeatureBuilder::create(s_current[i], pepper_vs.getIntrinsicCameraParameters(), blobs[i]);
                vpFeatureBuilder::create(s_desired[i], pepper_vs.getIntrinsicCameraParameters(), blobs[i]);
                task.addFeature(s_current[i], s_desired[i]);
                i++;
            }

            if(i == number_of_features)
            {
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
                    vpFeatureBuilder::create(s_current[i], pepper_vs.getIntrinsicCameraParameters(), blobs[i]);
                }
                
                velocity = task.computeControlLaw();
                vpServoDisplay::display(task, pepper_vs.getIntrinsicCameraParameters(), image);

#ifdef PEPPER_VISP_LOG_VELOCITY
                pepper_vs.writeVelocityToFile(velocity, pepper_visp::VelocityType::FULL);
#endif

#ifdef PEPPER_VISP_USE_PEPPER_CONTROLLER
                pepper_vs.callPepperController(velocity, pepper_visp::VelocityType::ANGULAR);
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
