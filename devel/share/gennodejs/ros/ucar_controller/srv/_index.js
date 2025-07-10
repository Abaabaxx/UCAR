
"use strict";

let GetMaxVel = require('./GetMaxVel.js')
let SetMaxVel = require('./SetMaxVel.js')
let GetSensorTF = require('./GetSensorTF.js')
let SetLEDMode = require('./SetLEDMode.js')
let SetSensorTF = require('./SetSensorTF.js')
let GetBatteryInfo = require('./GetBatteryInfo.js')

module.exports = {
  GetMaxVel: GetMaxVel,
  SetMaxVel: SetMaxVel,
  GetSensorTF: GetSensorTF,
  SetLEDMode: SetLEDMode,
  SetSensorTF: SetSensorTF,
  GetBatteryInfo: GetBatteryInfo,
};
