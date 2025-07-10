
"use strict";

let GetWaypointByName = require('./GetWaypointByName.js')
let GetChargerByName = require('./GetChargerByName.js')
let GetNumOfWaypoints = require('./GetNumOfWaypoints.js')
let GetWaypointByIndex = require('./GetWaypointByIndex.js')
let AddNewWaypoint = require('./AddNewWaypoint.js')
let SaveWaypoints = require('./SaveWaypoints.js')

module.exports = {
  GetWaypointByName: GetWaypointByName,
  GetChargerByName: GetChargerByName,
  GetNumOfWaypoints: GetNumOfWaypoints,
  GetWaypointByIndex: GetWaypointByIndex,
  AddNewWaypoint: AddNewWaypoint,
  SaveWaypoints: SaveWaypoints,
};
