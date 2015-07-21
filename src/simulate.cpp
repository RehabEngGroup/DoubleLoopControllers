// Authors: Luca Tagliapietra, Elena Ceseracciu, Monica Reggiani

#include <iostream>
using std::cout;
using std::endl;

#include <vector>
using std::vector;

#include <map>
using std::map;

#include <string>
using std::string;

#include <OpenSim/Common/LoadOpenSimLibrary.h>

#include "ConfigurationInterpreter.h"
#include "SimulationManager.h"

int main(int argc, char**argv) {
    cout << "---------------------------------------------------------\n";
    cout << " Biodex controller simulator\n";
    cout << " Copyright (C) L. Tagliapietra, E.Ceseracciu, M. Reggiani\n";
    cout << "---------------------------------------------------------\n";

    if (argc != 5) {
        cout << "Usage: ankleAttachmentOptimizer model datadirectory experimentalkinematic experimentalTorque\n";
        exit(EXIT_FAILURE);
    }

    const string modelName = argv[1];
    const string dataDir = argv[2];

    // Read the configuration Parameter File
    map<string, double> parametersMap;
    try {
        const string cfgFilename = (dataDir + "/simulationParameters.txt");
        ConfigurationInterpreter cfg(cfgFilename.c_str());
        cfg.getMap(parametersMap);
    }
    catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        return -1;
    }

    // Load experimental data to track
    OpenSim::Storage experimentalKinematic(argv[3]);
    OpenSim::Storage experimentalTorque(argv[4]);

    // Set output directory
    const string outputDir = dataDir + "/SimulationResults";
    cout << "Output directory: " << outputDir << endl;

    const string integratorName = "notRungeKuttaMerson";

    OpenSim::LoadOpenSimLibrary("DoubleLoopController");

    // ----- Load the Opensim Model
    OpenSim::Model biodexModel(modelName);

    //----------- Create and run Simulation Manager
    try
    {
        SimulationManager manager(experimentalKinematic, experimentalTorque, biodexModel, parametersMap, integratorName, outputDir);
        manager.simulate();
    }
    catch (OpenSim::Exception&e)
    {
        std::cout << e.what() << std::endl;
    }
    exit(EXIT_SUCCESS);
}

