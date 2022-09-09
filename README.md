# Bimodal Skin
This code accompanies the paper Sensorized Skin with Biomimetic Tactility Features based on Bimodal Soft Resistive Composites.

This repository contains the data for all tests, the code used to collect this data via a _National Instruments USB-6210_ + _Universal Robots UR5_, and the code to train and test neural networks to predict tactile stimuli.

Written using MATLAB 2021b (Deep Learning Toolbox) & Python 3 ([Generic UR5 Controller: kg398/Generic_ur5_controller: new version of ur5 python controller](https://github.com/kg398/Generic_ur5_controller)).

The parent repository can be found [here](https://github.com/DSHardman/TemperatureSensors).

## Data Availability
Data can be downloaded from this repository's v1.0 release.

The characterization responses of Figures 2-4 are stored in _Responses/SinglePresses__ as **SingleTest** objects. Variables are named with the convention x_repYD_T, where x indicates the network size (small, medium, or large), Y indicates the press location (A, B, or C), D indicates the pressed depth in mm (1 or 4), and T indicates the temperature (50 or 100C).

The 4500 responses of each network size (Figures 5-9) are stored in _Responses/_ as **SingleTest** objects which also contain the position, depth, and temperature data. Networks trained for each size can be found in _TrainedNetworks/_.

The circular test of Figure 6 is stored in _Responses/Patterns_Large.mat_ as a **SingleTest** object, which contains similar tests for additional circles and crosses. _Responses/Patterns_Medium.mat_ contains the same tests for the medium network.

## Data Collection
**RepeatsSynchronised.py**, **RandomSynchronised.py**, and **PatternsSynchronised.py** contain the python code used to collect the three datasets described above, which synchronizes data collection between a DAQ and robot arm. The thousands of raw output .npy files are not included in this repository, but are converted into the **SingleTest** MATLAB objects described above.

## Functions
**sensorTrain.m** creates and trains the feedforward neural networks using the given parameters, returning the trained network, the mean errors of the training, validation, and test sets, and the predictions/targets/errors of the test set. These are calculated by calling **calculateErrors.m**, and described in the function comments.
