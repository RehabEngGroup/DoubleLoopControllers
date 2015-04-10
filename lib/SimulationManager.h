// Authors: Luca Tagliapietra, Elena Ceseracciu, Monica Reggiani

#ifndef SimulationManager_h
#define SimulationManager_h

#include <OpenSim/OpenSim.h>

#include <string>
#include <map>

class SimulationManager{
public:
    SimulationManager(SimTK::State& initialStateFaked, OpenSim::Model & model,
        const std::map<std::string, double>& parametersMap,
        const std::string& integratorName, const std::string& outDir);

    SimulationManager(const OpenSim::Storage& expKinematic, const OpenSim::Storage& expTorque,
        OpenSim::Model & model, const std::map<std::string, double>& parametersMap,
        const std::string& integratorName, const std::string& outDir);

    void simulate();
    double getParameter(const std::string& key) const;
    double getStartingTime(){ return initialTime_; };
    double getFinalTime(){ return finalTime_; };
    double getReferencePosition(const double time) const;
    double getReferencePositionDerivatives(const double time, int order) const;
    double getReferenceVelocity(const double time) const;
    double getReferenceTorque(const double time) const;
    void getSimulationResults(OpenSim::Storage& states, OpenSim::Storage& torqueStates);
    const OpenSim::Model& getOsimModel(){ return osimModel_; };
    void setControllerStorage(const OpenSim::Storage& expKinematic);

private:
    void initializeModel();
    void setIntegrator();
    void setParameters();
    void initializeState();
    void saveSimulationResults(const OpenSim::Manager* manager);
    void createExperimentalDataSpline(const OpenSim::Storage& expKinematic, const OpenSim::Storage& expTorque);

    double initialTime_;
    double finalTime_;
    double accuracy_;
    double tolerance_;
    double minStepSize_;
    double maxStepSize_;

    OpenSim::Model& osimModel_;
    SimTK::Integrator* integrator_;
    OpenSim::Manager* integrationManager_;
    OpenSim::SimmSpline refSplinePosition_;
    OpenSim::SimmSpline refSplineVelocity_;
    OpenSim::SimmSpline refSplineTorque_;
    OpenSim::Array<double> expTime_;
    SimTK::State initialState_;

    std::map <std::string, double> parametersMap_;

    std::string integratorName_;
    std::string outDir_;
};

#endif
