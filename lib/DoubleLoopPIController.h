/* -------------------------------------------------------------------------- *
 *                DoubleLoopControllers: DoubleLoopPIController.h             *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Copyright 2015 Luca Tagliapietra, Elena Ceseracciu, Monica Reggiani        *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License");            *
 * you may not use this file except in compliance with the License.           *
 * You may obtain a copy of the License at                                    *
 *                                                                            *
 *     http://www.apache.org/licenses/LICENSE-2.0                             *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * ---------------------------------------------------------------------------*/

#ifndef DoubleLoopPIController_h
#define DoubleLoopPIController_h

#include <OpenSim/OpenSim.h>

namespace OpenSim{
    class DoubleLoopPIController : public OpenSim::Controller
    {
        OpenSim_DECLARE_CONCRETE_OBJECT(DoubleLoopPIController, OpenSim::Controller);
    public:
        OpenSim_DECLARE_PROPERTY(kpp, double,
            "Proportional gain for position errors.");
        OpenSim_DECLARE_PROPERTY(kpv, double,
            "Proportional gain for velocity error.");
        OpenSim_DECLARE_PROPERTY(kip, double,
            "Integral gain for position errors.");
        OpenSim_DECLARE_PROPERTY(kiv, double,
            "Integral gain for velocity error.");
        OpenSim_DECLARE_PROPERTY(coordinate_name, std::string,
            "Name of coordinate to track.");
        OpenSim_DECLARE_PROPERTY(actuator_name, std::string,
            "Name of actuator to control.");
        OpenSim_DECLARE_PROPERTY(desired_states_file, std::string,
            "Storage file containing the desired trajecttory (position, velocity) for the coordinate of interest.");

        DoubleLoopPIController();
        DoubleLoopPIController(const std::string &coordinateName, const std::string &actuatorName);
        DoubleLoopPIController(const std::string &aFileName, bool aUpdateFromXMLNode = true);
        DoubleLoopPIController(const DoubleLoopPIController &aController);
        virtual ~DoubleLoopPIController();
        /**
        * Constructor
        * @param motorCoord coordinate to control
        * @param motorId numerical id of the actuator that performs the control action
        */
        DoubleLoopPIController(OpenSim::Coordinate & motorCoord, int motorId);
#ifndef SWIG
        DoubleLoopPIController& operator=(const DoubleLoopPIController& aController);
#endif

        /**
         * This function is called at every time step for every actuator.
         *
         * @param s Current state of the system
         * @param index Index of the current actuator whose control is being calculated
         * @return Control value to be assigned to the current actuator at the current time
         */
        void computeControls(const SimTK::State& s, SimTK::Vector &controls) const;

        void initializeController(double startTime);

        void setVelocityLoopPIGains(std::vector<double>& gains);
        void setDoubleLoopPIGains(const std::vector<double>& gains);
        virtual void setDesiredStatesStorage(const OpenSim::Storage* aYDesStore);
        static void registerType();
    protected:
        void copyData(const DoubleLoopPIController &aController);
        void connectToModel(OpenSim::Model& model) OVERRIDE_11;
        void initStateFromProperties(SimTK::State& s) const OVERRIDE_11;

    private:
        void setNull();
        void constructProperties();
        void addToSystem(SimTK::MultibodySystem& system) const;
        SimTK::Vector computeStateVariableDerivatives(const SimTK::State& s) const;

        OpenSim::SimmSpline refSplinePosition_;
        OpenSim::SimmSpline refSplineVelocity_;
    };
}

#endif

