// Authors: Luca Tagliapietra, Elena Ceseracciu, Monica Reggiani

#include "DoubleLoopPIController.h"
#include "SimulationManager.h"
#include <stdio.h>
using std::cout;
using std::endl;
#include <string>
using std::string;
#include <vector>
using std::vector;

void OpenSim::DoubleLoopPIController::registerType()
{
    Object::registerType(OpenSim::DoubleLoopPIController());
}

OpenSim::DoubleLoopPIController::DoubleLoopPIController() :
TrackingController()
{
    setNull();
}

OpenSim::DoubleLoopPIController::DoubleLoopPIController(const std::string &coordinateName, const std::string &actuatorName) :
TrackingController()
{
    set_coordinateName(coordinateName);
    set_actuatorName(actuatorName);
}

OpenSim::DoubleLoopPIController::DoubleLoopPIController(const std::string &aFileName, bool aUpdateFromXMLNode) :
TrackingController()
{
    setNull();
    if (aUpdateFromXMLNode) updateFromXMLDocument();
}

OpenSim::DoubleLoopPIController::DoubleLoopPIController(const DoubleLoopPIController &aController) :
TrackingController()
{
    setNull();
    copyData(aController);
}

OpenSim::DoubleLoopPIController::~DoubleLoopPIController(){}

void OpenSim::DoubleLoopPIController::copyData(const DoubleLoopPIController &aController)
{
    set_kpp(aController.get_kpp());
    set_kpv(aController.get_kpv());
    set_kip(aController.get_kip());
    set_kiv(aController.get_kiv());

    set_coordinateName(aController.get_coordinateName());
    set_actuatorName(aController.get_actuatorName());

    refSplinePosition_ = aController.refSplinePosition_;
    refSplineVelocity_ = aController.refSplineVelocity_;

    _desiredStatesStorage = aController._desiredStatesStorage;

}

void OpenSim::DoubleLoopPIController::initStateFromProperties(SimTK::State& s) const
{
    Super::initStateFromProperties(s);
}

void OpenSim::DoubleLoopPIController::setNull()
{
    constructProperties();
    _desiredStatesStorage = NULL;
}

void OpenSim::DoubleLoopPIController::constructProperties()
{
    constructProperty_kpp(1.0);
    constructProperty_kpv(1.0);
    constructProperty_kip(1.0);
    constructProperty_kiv(1.0);

    constructProperty_coordinateName("motor");
    constructProperty_actuatorName("DCmotor");
}
void OpenSim::DoubleLoopPIController::computeControls(const SimTK::State& s, SimTK::Vector &controls) const
{
    double time = s.getTime();
    const OpenSim::Coordinate& motorCoordinate = getModel().getCoordinateSet().get(get_coordinateName());
    double curPosition = motorCoordinate.getValue(s);
    double curSpeed = motorCoordinate.getSpeedValue(s);

    SimTK::Vector timeV(1);
    timeV[0] = time;
    double refPosition = refSplinePosition_.calcValue(timeV);

    // Position error, input for the P controller of position
    double posError = refPosition - curPosition;
    // Integral of position error, input for the I controller of position
    double integralPosError = getStateVariable(s, "integral_error_position");

    // Control signal based on position PI
    double cp = get_kpp()*posError + get_kip()* integralPosError;

    // Speed error, input for P controller of velocity/speed
    double speedError = cp - curSpeed;
    //Integral of speed error, input for I controller of velocity/speed
    double integralVelError = getStateVariable(s, "integral_error_velocity");

    double cv = get_kiv()*integralVelError + get_kpv()*speedError;
    int actuatorId = getModel().getForceSet().getIndex(get_actuatorName());
    controls[actuatorId] = cv;
}


OpenSim::DoubleLoopPIController& OpenSim::DoubleLoopPIController::operator=(const OpenSim::DoubleLoopPIController& aController)
{
    TrackingController::operator=(aController);
    copyData(aController);
    return(*this);
}

void OpenSim::DoubleLoopPIController::setDoubleLoopPIGains(const std::vector<double> &x){
    if (x.size() != 4){
        cout << "\n check your parameters vector! It must have 4 elements!!" << endl;
        exit(EXIT_FAILURE);
    }
    set_kpp(x.at(0));
    set_kip(x.at(1));
    set_kpv(x.at(2));
    set_kiv(x.at(3));
}

void OpenSim::DoubleLoopPIController::setVelocityLoopPIGains(std::vector<double> &x){
    if (x.size() != 2){
        cout << "\n check your parameters vector! It must have 2 elements!!" << endl;
        exit(EXIT_FAILURE);
    }
    set_kpv(x.at(0));
    set_kiv(x.at(1));
}

void OpenSim::DoubleLoopPIController::addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);
    // Auxiliary states
    addStateVariable("integral_error_position");
    addStateVariable("integral_error_velocity");
    addCacheVariable("integral_error_position_deriv", 0.0, SimTK::Stage::Velocity);
    addCacheVariable("integral_error_velocity_deriv", 0.0, SimTK::Stage::Velocity);
}

void OpenSim::DoubleLoopPIController::connectToModel(OpenSim::Model& model)
{
    Super::connectToModel(model);
}

SimTK::Vector OpenSim::DoubleLoopPIController::computeStateVariableDerivatives(const SimTK::State& s) const
{
    SimTK::Vector derivs(getNumStateVariables(), 0.);
    // same as computeControls():
    double time = s.getTime();
    const OpenSim::Coordinate& motorCoordinate = getModel().getCoordinateSet().get(get_coordinateName());
    double curPosition = motorCoordinate.getValue(s);
    double curSpeed = motorCoordinate.getSpeedValue(s);
    SimTK::Vector timeV(1);
    timeV[0] = time;
    double refPosition = refSplinePosition_.calcValue(timeV);
    double posError = refPosition - curPosition;
    double integralPosError = getStateVariable(s, "integral_error_position");
    double cp = get_kpp()*posError + get_kip()* integralPosError;
    double speedError = cp - curSpeed;

    setCacheVariable<double>(s, "integral_error_position_deriv", posError);
    setCacheVariable<double>(s, "integral_error_velocity_deriv", speedError);

    derivs[0] = posError; // integral position
    derivs[1] = speedError; //integral velocity

    return derivs;
}

void OpenSim::DoubleLoopPIController::setDesiredStatesStorage(const OpenSim::Storage* aYDesStore)
{
    _desiredStatesStorage = aYDesStore;
    OpenSim::Array<double> expPos, expVel, expTor;
    const string expPosLabel("motor");
    const string expVelLabel("motor_u");
    OpenSim::Array<double> expTime;
    _desiredStatesStorage->getTimeColumn(expTime);
    // cannot use getDataColumn(string, Array) overload as it is not const :(
    _desiredStatesStorage->getDataColumn(_desiredStatesStorage->getStateIndex(expPosLabel), expPos);
    _desiredStatesStorage->getDataColumn(_desiredStatesStorage->getStateIndex(expVelLabel), expVel);

    for (int i = 0; i < expTime.getSize(); i++) {
        refSplinePosition_.addPoint(expTime[i], SimTK::convertDegreesToRadians(expPos[i]));
        refSplineVelocity_.addPoint(expTime[i], SimTK::convertDegreesToRadians(expVel[i]));
    }
}

const OpenSim::Storage& OpenSim::DoubleLoopPIController::getDesiredStatesStorage() const
{
    return *_desiredStatesStorage;
}

