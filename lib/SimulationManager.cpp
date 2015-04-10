// Authors: Luca Tagliapietra, Elena Ceseracciu, Monica Reggiani

#include <OpenSim/OpenSim.h>
#include <string>
using std::string;
#include <map>
using std::map;
#include <iostream>
using std::cout;
using std::endl;
#include "SimulationManager.h"

#include "DoubleLoopPIController.h"
using OpenSim::DoubleLoopPIController;

//***************************************************
//    Public Constructors
//***************************************************

SimulationManager::SimulationManager(const OpenSim::Storage& expKinematic, const OpenSim::Storage& expTorque,
    OpenSim::Model & model, const std::map<std::string, double>& parametersMap,
    const std::string& integratorName, const std::string& outDir) :
    osimModel_(model), parametersMap_(parametersMap), integratorName_(integratorName), outDir_(outDir), integrator_(NULL){
    setControllerStorage(expKinematic);
    // Add Force reporter to the model
    OpenSim::ForceReporter *torqueReporter = new OpenSim::ForceReporter(&osimModel_);
    torqueReporter->setName(std::string("torqueReporter"));
    osimModel_.addAnalysis(torqueReporter);

    initializeState(); // Need to initialize state, otherwise integration manager
                       // will not set column labels correctly in output files
    integrationManager_ = new OpenSim::Manager(osimModel_);
    createExperimentalDataSpline(expKinematic, expTorque);
    initializeModel();
    setParameters();
    setIntegrator();
    //initializeState();
}

//***************************************************
//    Public Functions
//***************************************************
void SimulationManager::setControllerStorage(const OpenSim::Storage& expKinematic){

    try
    {
        DoubleLoopPIController& controller = dynamic_cast<OpenSim::DoubleLoopPIController&>(osimModel_.updControllerSet().get("PIBiodexcontroller"));
        controller.setDesiredStatesStorage(&expKinematic);
    }
    catch (std::bad_cast& e)
    {
        std::cout << e.what() << std::endl;
        throw OpenSim::Exception("Could not find a DoubleLoopPiController in model file");
    }
}

double SimulationManager::getReferencePosition(const double aTime) const{
    SimTK::Vector time(1);
    time[0] = aTime;
    return refSplinePosition_.calcValue(time);
};

double SimulationManager::getReferencePositionDerivatives(const double aTime, int order) const{
    SimTK::Vector time(1);
    time[0] = aTime;
    std::vector<int> orderVec;
    for (int i = 0; i < order; ++i)
        orderVec.push_back(1);
    return refSplinePosition_.calcDerivative(orderVec, time);
};

double SimulationManager::getReferenceVelocity(const double aTime) const{
    SimTK::Vector time(1);
    time[0] = aTime;
    return refSplineVelocity_.calcValue(time);
};

double SimulationManager::getReferenceTorque(const double aTime) const{
    SimTK::Vector time(1);
    time[0] = aTime;
    return refSplineTorque_.calcValue(time);
};

double SimulationManager::getParameter(const string& key) const {
    map<string, double>::const_iterator it = parametersMap_.find(key);
    return it->second;
}

void SimulationManager::simulate() {
    initializeState();
    osimModel_.equilibrateMuscles(initialState_);
    setIntegrator();

    integrationManager_->resetTimeAndDTArrays(initialTime_);

    std::clock_t startTime = clock();

    cout << "\n\nIntegrating from " << initialTime_ << " to " << finalTime_ << endl;

    try{
        integrationManager_->integrate(initialState_);
    }
    catch (SimTK::Exception::Base &e) {
        throw OpenSim::Exception(e.what());
    }

    cout << "Integrate routine time = " << 1.e3*(clock() - startTime) / CLOCKS_PER_SEC << " ms" << endl;
    cout << "The maximum allowed error was " << integrator_->getAccuracyInUse();
    cout << " measured with " << integrator_->getNumStepsTaken() << " output steps and";
    cout << " a treshold value of " << integrator_->getAccuracyInUse() << endl;

    saveSimulationResults(integrationManager_);

}

void SimulationManager::getSimulationResults(OpenSim::Storage& states, OpenSim::Storage& torqueStates){
    states = integrationManager_->getStateStorage();

    try{
        const OpenSim::ForceReporter& forceReporter = dynamic_cast<const OpenSim::ForceReporter&>(osimModel_.getAnalysisSet().get(string("torqueReporter")));
        torqueStates = forceReporter.getForceStorage();
    }
    catch (std::bad_cast)
    {
        throw OpenSim::Exception("Could not get information from \"torqueReporter\" force reporter analysis");
    }


    delete(integrationManager_);
    integrationManager_ = new OpenSim::Manager(osimModel_);
}

//***************************************************
//    Private Functions
//***************************************************

void SimulationManager::initializeModel(){
    const std::string motorLabel("motor");
    SimTK::Vector time(1);
    time[0] = expTime_[0];
    OpenSim::Coordinate& motorJoint = osimModel_.updCoordinateSet().get(motorLabel);
    motorJoint.setValue(initialState_, refSplinePosition_.calcValue(time));
    motorJoint.setValue(initialState_, refSplineVelocity_.calcValue(time));
    osimModel_.getMultibodySystem().realize(initialState_, SimTK::Stage::Velocity);
}

void SimulationManager::createExperimentalDataSpline(const OpenSim::Storage& expKinematic, const OpenSim::Storage& expTorque){
    OpenSim::Array<double> expPos, expVel, expTor;
    const string expPosLabel("motor");
    const string expVelLabel("motor_u");
    const string expTorLabel("DCmotor");
    expKinematic.getTimeColumn(expTime_);
    expKinematic.getDataColumn(expKinematic.getStateIndex(expPosLabel), expPos);
    expKinematic.getDataColumn(expKinematic.getStateIndex(expVelLabel), expVel);
    expTorque.getDataColumn(expTorque.getStateIndex(expTorLabel), expTor);

    for (int i = 0; i < expTime_.getSize(); i++) {
        refSplinePosition_.addPoint(expTime_[i], SimTK::convertDegreesToRadians(expPos[i]));
        refSplineVelocity_.addPoint(expTime_[i], SimTK::convertDegreesToRadians(expVel[i]));
        refSplineTorque_.addPoint(expTime_[i], expTor[i]);
    }
}

void SimulationManager::setIntegrator(){
    if (!integratorName_.compare(string("RungeKuttaMerson")))
        integrator_ = new SimTK::RungeKuttaMersonIntegrator(osimModel_.getMultibodySystem());
    else
        integrator_ = new SimTK::RungeKuttaFeldbergIntegrator(osimModel_.getMultibodySystem());
    integrator_->setMinimumStepSize(minStepSize_);
    integrator_->setMaximumStepSize(maxStepSize_);
    integrator_->setAccuracy(accuracy_);
    integrator_->setConstraintTolerance(tolerance_);
    integrator_->setProjectEveryStep(true);

    integrationManager_->setIntegrator(integrator_);
    integrationManager_->setInitialTime(initialTime_);
    integrationManager_->setFinalTime(finalTime_);

}

void SimulationManager::setParameters(){
    initialTime_ = parametersMap_.find("Initial Time")->second;
    finalTime_ = parametersMap_.find("Final Time")->second;
    accuracy_ = parametersMap_.find("Accuracy")->second;
    tolerance_ = parametersMap_.find("Tolerance")->second;
    minStepSize_ = parametersMap_.find("Minimum Step Size")->second;
    maxStepSize_ = parametersMap_.find("Maximum Step Size")->second;
}

void SimulationManager::initializeState() {
    initialState_ = osimModel_.initSystem();
    osimModel_.getMultibodySystem().realize(initialState_, SimTK::Stage::Dynamics);
}

void SimulationManager::saveSimulationResults(const OpenSim::Manager* manager){
    cout << "Saving files..." << endl;

    OpenSim::Storage statesDegrees(manager->getStateStorage());
    OpenSim::Storage statesRadians(manager->getStateStorage());
    statesDegrees.print(outDir_ + "/StatesResults.sto");

    osimModel_.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
    statesDegrees.print(outDir_ + "/StatesResults_Degree.mot");

    // Save the forces
    try{
        const OpenSim::ForceReporter& forceReporter = dynamic_cast<const OpenSim::ForceReporter&>(osimModel_.getAnalysisSet().get(string("torqueReporter")));
        forceReporter.getForceStorage().print(outDir_ + "/forcesResults.mot");
    }
    catch (std::bad_cast)
    {
        throw OpenSim::Exception("Could not get information from \"torqueReporter\" force reporter analysis");

    }
}
