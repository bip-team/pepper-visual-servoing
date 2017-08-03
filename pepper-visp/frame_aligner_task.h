/**
    @file
    @author Jan Michalczyk 

    @brief
*/

#pragma once

#include <visp/vpCameraParameters.h>
#include <visp/vpImagePoint.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpServo.h>
#include <visp/vpServoDisplay.h>
#include <visp/vpGenericFeature.h>
#include <visp3/core/vpMatrix.h>

namespace pepper_visp
{
    /**
     * @brief Class used for PBVS task aligning
     *        two frames expressed in camera frame
     */
    class FrameAlignerTask 
    {
        public:
            /**
             * @brief Constructor
             *
             * @param[in] lambda 
             */
            FrameAlignerTask(const double lambda) : lambda_(lambda)
            {
                initialize();
            }


            /**
             * @brief Update task
             *
             * @param[in] cMe 
             * @param[in] cMo
             */
            void update(const vpHomogeneousMatrix& cMe, const vpHomogeneousMatrix& cMo)
            {
                // update twist between camera and end-effector
                task_.set_cVe(cMe);

                vpHomogeneousMatrix eMo;
                getObjectToEndEffectorTransform(eMo, cMe, cMo);

                // update features using M between end-effector and object
                translation_feature_.buildFrom(eMo);
                rotation_feature_.buildFrom(eMo);
            }


            /**
             * @brief Get velocity
             *
             * @param[in] velocity
             */
            void getVelocity(vpColVector& velocity)
            {
                velocity = task_.computeControlLaw();
            }


            /**
             * @brief Destructor
             */
            ~FrameAlignerTask()
            {
                task_.kill();
            }


        private:
            /**
             * @brief Get translational/rotational offset of object
             *        in end-effector frame
             *
             * @param[in, out] eMo 
             * @param[in]      cMe
             * @param[in]      cMo
             */
            void getObjectToEndEffectorTransform(vpHomogeneousMatrix&       eMo, 
                                                 const vpHomogeneousMatrix& cMe,
                                                 const vpHomogeneousMatrix& cMo) const
            {
                eMo = cMe.inverse() * cMo;
            }
            
            
            /**
             * @brief Initialize
             */
            void initialize()
            {
                translation_feature_.setFeatureTranslationType(vpFeatureTranslation::cdMc);
                rotation_feature_.setFeatureThetaURotationType(vpFeatureThetaU::cdRc);

                // regulate both features to zero
                task_.addFeature(translation_feature_);
                task_.addFeature(rotation_feature_);
                task_.setServo(vpServo::EYETOHAND_L_cVe_eJe);
                task_.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
                task_.setLambda(lambda_);

                // set end-effector jacobian to identity
                std::size_t dofs = 6;
                vpMatrix I(dofs, dofs);
                I.eye();
                task_.set_eJe(I);
            }


        private:
             double               lambda_;
             vpServo              task_;
             vpFeatureTranslation translation_feature_;
             vpFeatureThetaU      rotation_feature_;
    };
}//pepper_visp
