
"use strict";

let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let SyncModeInfo = require('./SyncModeInfo.js');
let ObjectStatus = require('./ObjectStatus.js');
let IntscnTL = require('./IntscnTL.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let SyncModeCmd = require('./SyncModeCmd.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let ReplayInfo = require('./ReplayInfo.js');
let VehicleSpec = require('./VehicleSpec.js');
let ObjectStatusList = require('./ObjectStatusList.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let DdCtrlCmd = require('./DdCtrlCmd.js');
let MapSpecIndex = require('./MapSpecIndex.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let VehicleCollision = require('./VehicleCollision.js');
let WaitForTick = require('./WaitForTick.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let GhostMessage = require('./GhostMessage.js');
let SyncModeRemoveObj = require('./SyncModeRemoveObj.js');
let Lamps = require('./Lamps.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');
let CtrlCmd = require('./CtrlCmd.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let RadarDetections = require('./RadarDetections.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let EventInfo = require('./EventInfo.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let PREvent = require('./PREvent.js');
let GPSMessage = require('./GPSMessage.js');
let SyncModeAddObj = require('./SyncModeAddObj.js');
let SaveSensorData = require('./SaveSensorData.js');
let ERP42Info = require('./ERP42Info.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let IntersectionControl = require('./IntersectionControl.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');
let RadarDetection = require('./RadarDetection.js');
let SensorPosControl = require('./SensorPosControl.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let PRStatus = require('./PRStatus.js');
let TrafficLight = require('./TrafficLight.js');
let CollisionData = require('./CollisionData.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let MapSpec = require('./MapSpec.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');

module.exports = {
  MoraiSimProcStatus: MoraiSimProcStatus,
  SyncModeInfo: SyncModeInfo,
  ObjectStatus: ObjectStatus,
  IntscnTL: IntscnTL,
  GetTrafficLightStatus: GetTrafficLightStatus,
  SyncModeCmd: SyncModeCmd,
  SyncModeResultResponse: SyncModeResultResponse,
  ReplayInfo: ReplayInfo,
  VehicleSpec: VehicleSpec,
  ObjectStatusList: ObjectStatusList,
  ScenarioLoad: ScenarioLoad,
  MoraiSrvResponse: MoraiSrvResponse,
  DdCtrlCmd: DdCtrlCmd,
  MapSpecIndex: MapSpecIndex,
  PRCtrlCmd: PRCtrlCmd,
  SetTrafficLight: SetTrafficLight,
  VehicleSpecIndex: VehicleSpecIndex,
  VehicleCollision: VehicleCollision,
  WaitForTick: WaitForTick,
  VehicleCollisionData: VehicleCollisionData,
  IntersectionStatus: IntersectionStatus,
  GhostMessage: GhostMessage,
  SyncModeRemoveObj: SyncModeRemoveObj,
  Lamps: Lamps,
  MoraiTLIndex: MoraiTLIndex,
  CtrlCmd: CtrlCmd,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  RadarDetections: RadarDetections,
  SyncModeCmdResponse: SyncModeCmdResponse,
  EventInfo: EventInfo,
  EgoVehicleStatus: EgoVehicleStatus,
  PREvent: PREvent,
  GPSMessage: GPSMessage,
  SyncModeAddObj: SyncModeAddObj,
  SaveSensorData: SaveSensorData,
  ERP42Info: ERP42Info,
  NpcGhostCmd: NpcGhostCmd,
  IntersectionControl: IntersectionControl,
  SyncModeSetGear: SyncModeSetGear,
  RadarDetection: RadarDetection,
  SensorPosControl: SensorPosControl,
  MoraiSimProcHandle: MoraiSimProcHandle,
  PRStatus: PRStatus,
  TrafficLight: TrafficLight,
  CollisionData: CollisionData,
  MultiEgoSetting: MultiEgoSetting,
  WaitForTickResponse: WaitForTickResponse,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  MoraiTLInfo: MoraiTLInfo,
  MapSpec: MapSpec,
  NpcGhostInfo: NpcGhostInfo,
};
