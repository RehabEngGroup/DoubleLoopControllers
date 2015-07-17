// Authors: Luca Tagliapietra, Elena Ceseracciu, Monica Reggiani

#ifndef DoubleLoopPIController_h
#define DoubleLoopPIController_h

#include <OpenSim/OpenSim.h>

class SimulationManager;

namespace OpenSim{
    class DoubleLoopPIController : public OpenSim::TrackingController
    {
        OpenSim_DECLARE_CONCRETE_OBJECT(DoubleLoopPIController, OpenSim::TrackingController);
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
        virtual const OpenSim::Storage& getDesiredStatesStorage() const;
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

