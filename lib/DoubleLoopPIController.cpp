/* -------------------------------------------------------------------------- *
 *               DoubleLoopControllers: DoubleLoopPIController.cpp            *
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

#include "DoubleLoopPIController.h"
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
Controller()
{
    setNull();
}

OpenSim::DoubleLoopPIController::DoubleLoopPIController(const std::string &coordinateName, const std::string &actuatorName) :
Controller()
{
    set_coordinate_name(coordinateName);
    set_actuator_name(actuatorName);
}

OpenSim::DoubleLoopPIController::DoubleLoopPIController(const std::string &aFileName, bool aUpdateFromXMLNode) :
Controller()
{
    setNull();
    if (aUpdateFromXMLNode) updateFromXMLDocument();
}

OpenSim::DoubleLoopPIController::DoubleLoopPIController(const DoubleLoopPIController &aController) :
Controller()
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

    set_coordinate_name(aController.get_coordinate_name());
    set_actuator_name(aController.get_actuator_name());
    set_desired_states_file(aController.get_desired_states_file());

    refSplinePosition_ = aController.refSplinePosition_;
    refSplineVelocity_ = aController.refSplineVelocity_;

}

void OpenSim::DoubleLoopPIController::initStateFromProperties(SimTK::State& s) const
{
    Super::initStateFromProperties(s);
}

void OpenSim::DoubleLoopPIController::setNull()
{
    constructProperties();
}

void OpenSim::DoubleLoopPIController::constructProperties()
{
    constructProperty_kpp(1.0);
    constructProperty_kpv(1.0);
    constructProperty_kip(1.0);
    constructProperty_kiv(1.0);

    constructProperty_coordinate_name("motor");
    constructProperty_actuator_name("DCmotor");
    constructProperty_desired_states_file("");
}
void OpenSim::DoubleLoopPIController::computeControls(const SimTK::State& s, SimTK::Vector &controls) const
{
    double time = s.getTime();
    const OpenSim::Coordinate& motorCoordinate = getModel().getCoordinateSet().get(get_coordinate_name());
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
    int actuatorId = getModel().getForceSet().getIndex(get_actuator_name());
    controls[actuatorId] = cv;
}


OpenSim::DoubleLoopPIController& OpenSim::DoubleLoopPIController::operator=(const OpenSim::DoubleLoopPIController& aController)
{
    Controller::operator=(aController);
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
    if (get_desired_states_file() != ""){
        Storage desiredStorage(get_desired_states_file());
        setDesiredStatesStorage(&desiredStorage);
    }
}

SimTK::Vector OpenSim::DoubleLoopPIController::computeStateVariableDerivatives(const SimTK::State& s) const
{
    SimTK::Vector derivs(getNumStateVariables(), 0.);
    // same as computeControls():
    double time = s.getTime();
    const OpenSim::Coordinate& motorCoordinate = getModel().getCoordinateSet().get(get_coordinate_name());
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
    OpenSim::Array<double> expPos, expVel;
    const string expPosLabel(get_coordinate_name());
    const string expVelLabel(get_coordinate_name()+ "_u");
    OpenSim::Array<double> expTime;
    aYDesStore->getTimeColumn(expTime);
    // cannot use getDataColumn(string, Array) overload as it is not const :(
    //TODO : throw exception if columns are not found!
    aYDesStore->getDataColumn(aYDesStore->getStateIndex(expPosLabel), expPos);
    aYDesStore->getDataColumn(aYDesStore->getStateIndex(expVelLabel), expVel);

    //Super ugly hack to reset the spline values, since I don't see any "clear" method
    refSplinePosition_ = OpenSim::SimmSpline();
    refSplineVelocity_ = OpenSim::SimmSpline();
    for (int i = 0; i < expTime.getSize(); i++) {
        refSplinePosition_.addPoint(expTime[i], SimTK::convertDegreesToRadians(expPos[i]));
        refSplineVelocity_.addPoint(expTime[i], SimTK::convertDegreesToRadians(expVel[i]));
    }
}

