/**
    @file
    @author Jan Michalczyk 
    @copyright 2016-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <visp/vpCameraParameters.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/mbt/vpMbEdgeTracker.h>
#include <visp3/mbt/vpMbEdgeKltTracker.h>

#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

namespace pepper_visp
{
    /**
     * @brief Class used for tracking an object 
     *        defined .cao file (ex. box)
     */
    class ModelBasedTracker 
    {
        public:
            /**
             * @brief Constructor
             *
             * @param[in] data_path 
             * @param[in] camera_parameters
             * @param[in] model_filename
             */
            ModelBasedTracker(const std::string&        data_path, 
                              const vpCameraParameters& camera_parameters,
                              const std::string&        model_filename) : data_path_(data_path),
                                                                          camera_parameters_(camera_parameters)
            {
                tracker_.reset(new vpMbEdgeKltTracker);
                initializeTrackerSettings();
                tracker_->setCameraParameters(camera_parameters_);
                tracker_->loadModel(data_path_ + model_filename);
                tracker_->setDisplayFeatures(true);
            }


            /**
             * @brief Initialize tracker by clicking on four
             *        points defined in .init file
             *
             * @param[in] image
             * @param[in] init_filename
             */
            void initializeByClick(const vpImage<unsigned char>& image, 
                                   const std::string& init_filename) const
            {
                tracker_->initClick(image, data_path_ + init_filename, true);
            }


            /**
             * @brief Track object
             *
             * @param[in] image
             */
            void track(const vpImage<unsigned char>& image) const
            {
                tracker_->track(image);
            }

            
            /**
             * @brief Get object pose in camera frame
             *
             * @param[in, out] cMo
             */
            void getObjectPose(vpHomogeneousMatrix& cMo) const
            {
                tracker_->getPose(cMo);
            }
            

            /**
             * @brief Display object frame (pose)
             *
             * @param[in] cMo
             * @param[in] image
             */
            void displayObjectFrame(const vpHomogeneousMatrix&    cMo, 
                                    const vpImage<unsigned char>& image) const
            {
                tracker_->display(image, cMo, camera_parameters_, vpColor::red, 2, true);
                vpDisplay::displayFrame(image, cMo, camera_parameters_, 0.025, vpColor::none, 3);
            }


        private:  
            /**
             * @brief Initialize tracker settings
             */
            void initializeTrackerSettings()
            {
                vpMe moving_edge;
                moving_edge.setMaskSize(5);
                moving_edge.setMaskNumber(180);
                moving_edge.setRange(8);
                moving_edge.setThreshold(10000);
                moving_edge.setMu1(0.5);
                moving_edge.setMu2(0.5);
                moving_edge.setSampleStep(4);
                
                boost::dynamic_pointer_cast<vpMbEdgeTracker>(tracker_)->setMovingEdge(moving_edge);        

                vpKltOpencv klt_settings;
                klt_settings.setMaxFeatures(300);
                klt_settings.setWindowSize(5);
                klt_settings.setQuality(0.015);
                klt_settings.setMinDistance(8);
                klt_settings.setHarrisFreeParameter(0.01);
                klt_settings.setBlockSize(3);
                klt_settings.setPyramidLevels(3);
                
                boost::dynamic_pointer_cast<vpMbKltTracker>(tracker_)->setKltOpencv(klt_settings);
                boost::dynamic_pointer_cast<vpMbKltTracker>(tracker_)->setKltMaskBorder(5);
            }


        private:
            std::string                           data_path_;
            vpCameraParameters                    camera_parameters_;
            boost::shared_ptr<vpMbEdgeKltTracker> tracker_;
    };
}//pepper_visp
