function tests = test_pmsm_svpwm_model
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.ProjectDir = fileparts(fileparts(mfilename("fullpath")));
addpath(testCase.TestData.ProjectDir);
end

function teardownOnce(testCase)
rmpath(testCase.TestData.ProjectDir);
end

function testBuilderCreatesConfiguredSimulinkModel(testCase)
projectDir = testCase.TestData.ProjectDir;
modelName = "pmsm_dq_svpwm_cascade";
modelFile = fullfile(projectDir, modelName + ".slx");

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

if isfile(modelFile)
    delete(modelFile);
end

result = build_pmsm_svpwm_model("OpenModel", false, "RunSimulation", false);

verifyTrue(testCase, isfile(modelFile));
verifyEqual(testCase, result.modelName, modelName);
verifyEqual(testCase, result.params.Vdc, 36);
verifyEqual(testCase, result.params.Rs, 0.6);
verifyEqual(testCase, result.params.Ld, 0.82e-3);
verifyEqual(testCase, result.params.Ld, result.params.Lq);
verifyEqual(testCase, result.params.PolePairs, 14);
verifyEqual(testCase, result.params.MaxSpeedRpm, 1500);
verifyEqual(testCase, result.params.RatedCurrent, 3.8);
verifyEqual(testCase, result.params.PeakCurrent, 19.5);
verifyEqual(testCase, result.params.KvRpmPerVolt, 6.71);
verifyEqual(testCase, result.params.GearStage1Ratio, 3.71428571428);
verifyEqual(testCase, result.params.GearStage2Ratio, 14);
verifyEqual(testCase, result.params.GearTotalRatio, 52, "RelTol", 1e-10);
verifyEqual(testCase, result.params.OutputMaxSpeedRpm, ...
    result.params.MaxSpeedRpm / result.params.GearTotalRatio, "RelTol", 1e-12);
verifyEqual(testCase, result.params.PsiF, ...
    voltageLimitedFluxLinkage( ...
    result.params.Vdc, result.params.MaxSpeedRpm, result.params.PolePairs), ...
    "RelTol", 1e-12);
verifyEqual(testCase, result.params.Kt, ...
    1.5 * result.params.PolePairs * result.params.PsiF, "RelTol", 1e-12);
verifyEqual(testCase, result.params.RatedTorqueMotor, 0.7);
verifyEqual(testCase, result.params.StallTorqueMotor, 2.2);
verifyEqual(testCase, result.params.RatedTorqueOutput, ...
    result.params.RatedTorqueMotor * result.params.GearTotalRatio, "RelTol", 1e-12);
verifyEqual(testCase, result.params.StallTorqueOutput, ...
    result.params.StallTorqueMotor * result.params.GearTotalRatio, "RelTol", 1e-12);
verifyEqual(testCase, result.params.LoadTorqueStallFraction, 0.9);
verifyEqual(testCase, result.params.LoadTorqueFinal, ...
    result.params.LoadTorqueStallFraction * result.params.StallTorqueOutput, ...
    "RelTol", 1e-12);
verifyEqual(testCase, result.params.RatedCurrentFromTorque, ...
    result.params.RatedTorqueMotor / result.params.Kt, "RelTol", 1e-12);
verifyEqual(testCase, result.params.StallCurrentFromTorque, ...
    result.params.StallTorqueMotor / result.params.Kt, "RelTol", 1e-12);
verifyEqual(testCase, result.params.IqLimit, ...
    min(result.params.PeakCurrent, result.params.StallCurrentFromTorque), ...
    "RelTol", 1e-12);

load_system(modelName);
cleanup = onCleanup(@() close_system(modelName, 0));

verifyEqual(testCase, string(get_param(modelName, "Solver")), "FixedStepDiscrete");
verifyEqual(testCase, str2double(get_param(modelName, "FixedStep")), result.params.Ts);
verifyNotEmpty(testCase, find_system(modelName, "Name", "Cascaded PI SVPWM PMSM"));
verifyNotEmpty(testCase, find_system(modelName, "Name", "load_torque_step"));
verifyNotEmpty(testCase, find_system(modelName, "Name", "iq"));
verifyNotEmpty(testCase, find_system(modelName, "Name", "omega_rpm"));
verifyNotEmpty(testCase, find_system(modelName, "Name", "theta_m"));
verifyNotEmpty(testCase, find_system(modelName, "Name", "omega_stage1_rpm"));
verifyNotEmpty(testCase, find_system(modelName, "Name", "theta_stage1"));
verifyNotEmpty(testCase, find_system(modelName, "Name", "omega_out_rpm"));
verifyNotEmpty(testCase, find_system(modelName, "Name", "theta_out"));
verifyNotEmpty(testCase, find_system(modelName, "Name", "load_torque"));
verifyNotEmpty(testCase, find_system(modelName, "Name", "load_torque_motor"));
verifyNotEmpty(testCase, find_system(modelName, "Name", "torque_e"));
verifyNotEmpty(testCase, find_system(modelName, "Name", "torque_motor"));
verifyNotEmpty(testCase, find_system(modelName, "Name", "torque_stage1"));
verifyNotEmpty(testCase, find_system(modelName, "Name", "torque_output"));
end

function testPositionStepResponseSettlesNearReference(testCase)
projectDir = testCase.TestData.ProjectDir;
modelName = "pmsm_dq_svpwm_cascade";
modelFile = fullfile(projectDir, modelName + ".slx");

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

if isfile(modelFile)
    delete(modelFile);
end

result = build_pmsm_svpwm_model("OpenModel", false, "RunSimulation", false);
simOut = sim(result.modelName, "ReturnWorkspaceOutputs", "on");

theta = simOut.get("theta_out");
omegaRpm = simOut.get("omega_out_rpm");
thetaMotor = simOut.get("theta_m");
omegaMotorRpm = simOut.get("omega_rpm");
thetaStage1 = simOut.get("theta_stage1");
omegaStage1Rpm = simOut.get("omega_stage1_rpm");
iq = simOut.get("iq");
torqueE = simOut.get("torque_e");
torqueMotor = simOut.get("torque_motor");
torqueStage1 = simOut.get("torque_stage1");
torqueOutput = simOut.get("torque_output");
loadTorque = simOut.get("load_torque");
loadTorqueMotor = simOut.get("load_torque_motor");

preLoadMask = theta.Time > result.params.LoadStepTime - 0.1 & ...
    theta.Time < result.params.LoadStepTime;

verifyTrue(testCase, all(isfinite(theta.Data)));
verifyTrue(testCase, all(isfinite(omegaRpm.Data)));
verifyTrue(testCase, all(isfinite(iq.Data)));
verifyTrue(testCase, all(isfinite(torqueE.Data)));
verifyLessThanOrEqual(testCase, max(abs(omegaMotorRpm.Data)), result.params.MaxSpeedRpm + 1);
verifyLessThan(testCase, ...
    abs(theta.Data(find(preLoadMask, 1, "last")) - result.params.PositionStep), 0.05);
verifyLessThanOrEqual(testCase, max(abs(iq.Data)), result.params.IqLimit + 0.25);
verifyLessThanOrEqual(testCase, max(abs(torqueE.Data)), result.params.StallTorqueMotor + 0.05);
verifyEqual(testCase, torqueMotor.Data, torqueE.Data, "AbsTol", 1e-12);
verifyEqual(testCase, torqueStage1.Data, ...
    torqueMotor.Data * result.params.GearStage1Ratio * result.params.GearEfficiency, ...
    "AbsTol", 1e-12);
verifyEqual(testCase, torqueOutput.Data, ...
    torqueMotor.Data * result.params.GearTotalRatio * result.params.GearEfficiency, ...
    "AbsTol", 1e-12);
verifyEqual(testCase, thetaMotor.Data(end) / result.params.GearTotalRatio, ...
    theta.Data(end), "RelTol", 1e-8);
verifyEqual(testCase, omegaMotorRpm.Data(end) / result.params.GearTotalRatio, ...
    omegaRpm.Data(end), "AbsTol", 1e-8);
verifyEqual(testCase, thetaMotor.Data(end) / result.params.GearStage1Ratio, ...
    thetaStage1.Data(end), "RelTol", 1e-8);
verifyEqual(testCase, omegaMotorRpm.Data(end) / result.params.GearStage1Ratio, ...
    omegaStage1Rpm.Data(end), "AbsTol", 1e-8);
verifyEqual(testCase, thetaStage1.Data(end) / result.params.GearStage2Ratio, ...
    theta.Data(end), "RelTol", 1e-8);
verifyEqual(testCase, omegaStage1Rpm.Data(end) / result.params.GearStage2Ratio, ...
    omegaRpm.Data(end), "AbsTol", 1e-8);

preLoadSamples = loadTorque.Data(loadTorque.Time < result.params.LoadStepTime);
postLoadSamples = loadTorque.Data(loadTorque.Time > result.params.LoadStepTime + 5 * result.params.Ts);
postMotorLoadSamples = loadTorqueMotor.Data( ...
    loadTorqueMotor.Time > result.params.LoadStepTime + 5 * result.params.Ts);

verifyEqual(testCase, preLoadSamples(1), result.params.LoadTorqueInitial);
verifyEqual(testCase, postLoadSamples(end), result.params.LoadTorqueFinal, "RelTol", 1e-10);
verifyEqual(testCase, postMotorLoadSamples(end), ...
    result.params.LoadTorqueFinal / result.params.GearTotalRatio, "RelTol", 1e-11);
verifyEqual(testCase, postMotorLoadSamples(end), ...
    result.params.LoadTorqueStallFraction * result.params.StallTorqueMotor, ...
    "RelTol", 1e-11);
end

function psiF = voltageLimitedFluxLinkage(vdc, maxSpeedRpm, polePairs)
maxMechanicalSpeed = maxSpeedRpm * 2 * pi / 60;
voltageLimit = vdc / sqrt(3);
psiF = voltageLimit / (polePairs * maxMechanicalSpeed);
end
