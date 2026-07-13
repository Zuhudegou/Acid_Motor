clear;
close all;
clc;

result = build_pmsm_svpwm_model("OpenModel", true, "RunSimulation", true);

fprintf("Simulink model: %s\n", result.modelFile);
fprintf("Step-response PNG: %s\n", result.plotFile);
fprintf("Step-response FIG: %s\n", result.figureFile);
