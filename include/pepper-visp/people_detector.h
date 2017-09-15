/**
    @file
    @author Jan Michalczyk 
    @copyright 2016-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <sstream> 

#include <boost/shared_ptr.hpp>

#include <alproxies/alpeopleperceptionproxy.h>
#include <alproxies/almemoryproxy.h>

#include "config-yaml/config_reader.h"

namespace pepper_visp
{
    /**
     * @brief Class for detecting people
     */
    class PeopleDetector
    {
        public:
            /**
             * @brief Constructor
             */
            PeopleDetector()
            {
                std::string robot_ip = "10.42.0.61";
                int robot_port       = 9559;
                setParameters(robot_ip, robot_port);
                initialize();
            }
            
            
            /**
             * @brief Constructor
             *
             * @param[in] robot_ip 
             * @param[in] robot_port 
             * @param[in] camera_id
             */
            PeopleDetector(const std::string& robot_ip, const int robot_port)
            {
                setParameters(robot_ip, robot_port);
                initialize();
            }
            
            
            /**
             * @brief Constructor
             *
             * @param[in] config_file 
             */
            PeopleDetector(const std::string& config_file)
            {
                readParameters(config_file);
                initialize();
            }

            
            /**
             * @brief Get distance from person
             *
             * @return distance from person 
             */
            double getDistanceFromPerson()
            {   
                double distance;
                try
                {
                    distance = memory_->getData(std::string("PeoplePerception/Person/") + id_ + std::string("/Distance"));
                }
                catch(const std::exception& e)
                {
                    std::cerr << "Exception in getDistanceFromPerson()." << e.what() << std::endl;
                    throw;
                }
                
                return(distance);
            }


            /**
             * @brief Is person visible
             *
             * @return person visible 
             */
            bool isPersonVisible()
            {
                try
                {
                    return(memory_->getData(std::string("PeoplePerception/Person/") + id_ + std::string("/IsVisible")));
                }
                catch(const std::exception& e)
                {
                    std::cerr << "Exception in isPersonVisible()." << e.what() << std::endl;
                    throw;
                }
            }


            /**
             * @brief Initialize detector
             *
             * @return tracker initialized 
             */
            bool initializeDetector()
            {
                try
                {
                    people_ = memory_->getData("PeoplePerception/VisiblePeopleList");

                    if(people_.getSize())
                    {
                        person_info_ = memory_->getData("PeoplePerception/PeopleDetected");

                        std::ostringstream id;                       
                        id << person_info_[1][0][0];
                        id_ = id.str();
                        
                        return(true);
                    }
                }
                catch(const std::exception& e)
                {
                    std::cerr << "Exception in initializeDetector()." << e.what() << std::endl;
                    throw;
                }

                return(false);
            }


            /**
             * @brief Destructor
             */
            ~PeopleDetector()
            {
                people_detection_->unsubscribe("People");
            }
            

        private:
            /**
             * @brief Initialize
             */
            void initialize()
            {
                try
                {
                    people_detection_.reset(new AL::ALPeoplePerceptionProxy(robot_ip_, robot_port_));  
                    people_detection_->subscribe("People");
                    people_detection_->setFastModeEnabled(false);
                    people_detection_->setMovementDetectionEnabled(false);
                    people_detection_->setMaximumDetectionRange(5.0);
                    people_detection_->setTimeBeforePersonDisappears(1000.0);
                    people_detection_->setTimeBeforeVisiblePersonDisappears(1000.0);
                    
                    memory_.reset(new AL::ALMemoryProxy(robot_ip_, robot_port_));  
                }
                catch(const std::exception& e)
                {
                    std::cerr << "Exception in initialization: " << e.what() << std::endl;
                    throw;
                }
            }


            /**
             * @brief Set parameters
             *
             * @param[in] robot_ip
             * @param[in] robot_port
             * @param[in] camera_id
             */
            void setParameters(const std::string& robot_ip, const int robot_port)
            {
                robot_ip_   = robot_ip;
                robot_port_ = robot_port;
            }
            
            
            /**
             * @brief Read parameters from config file
             *
             * @param[in] config_file
             */
            void readParameters(const std::string& config_file)
            {
                yaml_config::ConfigReader config_reader(config_file);
                try
                {
                    config_reader.readScalar("pepper_visp", "robot_ip",   robot_ip_);
                    config_reader.readScalar("pepper_visp", "robot_port", robot_port_);
                }
                catch(const std::exception& e)
                {
                    std::cerr << "Exception in reading configuration file: " << e.what() << std::endl;
                    throw;
                }
            }
            

        private:
            boost::shared_ptr<AL::ALPeoplePerceptionProxy> people_detection_;
            boost::shared_ptr<AL::ALMemoryProxy>           memory_;
            
            AL::ALValue people_;
            AL::ALValue person_info_;
            std::string robot_ip_;
            int         robot_port_;
            std::string id_;
    };
}//pepper_visp
