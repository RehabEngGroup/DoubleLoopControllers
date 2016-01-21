# DoubleLoopControllers plugin for OpenSim

This repository currently hosts a DoubleLoop PI controller plugin for OpenSim 3.2 and 3.3.

A simple pendulum model is provided as an example in the *data* folder, together with an XML setup file for the `forward` OpenSim tool.

Usage:

    forward -L DoubleLoopController -S pendulum_DLPI_forward.xml

Please make sure that the *DoubleLoopController* shared library is in your system's PATH.

Also, please note that the example does not work *AS IS* in the GUI because of issues in reading files with relative paths in the XML setups.
