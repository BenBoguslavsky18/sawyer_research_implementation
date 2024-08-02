
"use strict";

let HeadState = require('./HeadState.js');
let CameraSettings = require('./CameraSettings.js');
let IODeviceStatus = require('./IODeviceStatus.js');
let HomingState = require('./HomingState.js');
let InteractionControlState = require('./InteractionControlState.js');
let AnalogOutputCommand = require('./AnalogOutputCommand.js');
let HeadPanCommand = require('./HeadPanCommand.js');
let IOStatus = require('./IOStatus.js');
let EndpointStates = require('./EndpointStates.js');
let IOComponentCommand = require('./IOComponentCommand.js');
let URDFConfiguration = require('./URDFConfiguration.js');
let JointLimits = require('./JointLimits.js');
let DigitalOutputCommand = require('./DigitalOutputCommand.js');
let IONodeStatus = require('./IONodeStatus.js');
let AnalogIOState = require('./AnalogIOState.js');
let SEAJointState = require('./SEAJointState.js');
let AnalogIOStates = require('./AnalogIOStates.js');
let EndpointNamesArray = require('./EndpointNamesArray.js');
let IOComponentConfiguration = require('./IOComponentConfiguration.js');
let IODeviceConfiguration = require('./IODeviceConfiguration.js');
let EndpointState = require('./EndpointState.js');
let CollisionDetectionState = require('./CollisionDetectionState.js');
let NavigatorState = require('./NavigatorState.js');
let NavigatorStates = require('./NavigatorStates.js');
let CameraControl = require('./CameraControl.js');
let IODataStatus = require('./IODataStatus.js');
let CollisionAvoidanceState = require('./CollisionAvoidanceState.js');
let HomingCommand = require('./HomingCommand.js');
let RobotAssemblyState = require('./RobotAssemblyState.js');
let JointCommand = require('./JointCommand.js');
let IONodeConfiguration = require('./IONodeConfiguration.js');
let DigitalIOStates = require('./DigitalIOStates.js');
let IOComponentStatus = require('./IOComponentStatus.js');
let DigitalIOState = require('./DigitalIOState.js');
let InteractionControlCommand = require('./InteractionControlCommand.js');
let CalibrationCommandActionFeedback = require('./CalibrationCommandActionFeedback.js');
let CalibrationCommandGoal = require('./CalibrationCommandGoal.js');
let CalibrationCommandActionGoal = require('./CalibrationCommandActionGoal.js');
let CalibrationCommandActionResult = require('./CalibrationCommandActionResult.js');
let CalibrationCommandResult = require('./CalibrationCommandResult.js');
let CalibrationCommandAction = require('./CalibrationCommandAction.js');
let CalibrationCommandFeedback = require('./CalibrationCommandFeedback.js');

module.exports = {
  HeadState: HeadState,
  CameraSettings: CameraSettings,
  IODeviceStatus: IODeviceStatus,
  HomingState: HomingState,
  InteractionControlState: InteractionControlState,
  AnalogOutputCommand: AnalogOutputCommand,
  HeadPanCommand: HeadPanCommand,
  IOStatus: IOStatus,
  EndpointStates: EndpointStates,
  IOComponentCommand: IOComponentCommand,
  URDFConfiguration: URDFConfiguration,
  JointLimits: JointLimits,
  DigitalOutputCommand: DigitalOutputCommand,
  IONodeStatus: IONodeStatus,
  AnalogIOState: AnalogIOState,
  SEAJointState: SEAJointState,
  AnalogIOStates: AnalogIOStates,
  EndpointNamesArray: EndpointNamesArray,
  IOComponentConfiguration: IOComponentConfiguration,
  IODeviceConfiguration: IODeviceConfiguration,
  EndpointState: EndpointState,
  CollisionDetectionState: CollisionDetectionState,
  NavigatorState: NavigatorState,
  NavigatorStates: NavigatorStates,
  CameraControl: CameraControl,
  IODataStatus: IODataStatus,
  CollisionAvoidanceState: CollisionAvoidanceState,
  HomingCommand: HomingCommand,
  RobotAssemblyState: RobotAssemblyState,
  JointCommand: JointCommand,
  IONodeConfiguration: IONodeConfiguration,
  DigitalIOStates: DigitalIOStates,
  IOComponentStatus: IOComponentStatus,
  DigitalIOState: DigitalIOState,
  InteractionControlCommand: InteractionControlCommand,
  CalibrationCommandActionFeedback: CalibrationCommandActionFeedback,
  CalibrationCommandGoal: CalibrationCommandGoal,
  CalibrationCommandActionGoal: CalibrationCommandActionGoal,
  CalibrationCommandActionResult: CalibrationCommandActionResult,
  CalibrationCommandResult: CalibrationCommandResult,
  CalibrationCommandAction: CalibrationCommandAction,
  CalibrationCommandFeedback: CalibrationCommandFeedback,
};
