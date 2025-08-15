import React from "react";
import "../css/locations.css";

const Locations = ({ aprilTagPosition, sensorPosition }) => {
  if (!aprilTagPosition || !sensorPosition) {
    return (
      <div className="loading-container">VEHICLE LOCATION IS LOADING..</div>
    );
  }
  console.log("🔍 Locations'a gelen aprilTagPosition:", aprilTagPosition);
  console.log("🔍 Locations'a gelen sensorPosition:", sensorPosition);
  return (
    <div className="locations">
      <div>
        <h3>AprilTag Based Position</h3>
        <p>X: {aprilTagPosition.x.toFixed(2)} m</p>
        <p>Y: {aprilTagPosition.y.toFixed(2)} m</p>
        <p>Z: {aprilTagPosition.z.toFixed(2)} m</p>
      </div>
      <div>
        <h3>Wheel Speed Sensor Based Position</h3>
        <p>X: {sensorPosition.x.toFixed(2)} m</p>
        <p>Y: {sensorPosition.y.toFixed(2)} m</p>
        <p>Z: {sensorPosition.z.toFixed(2)} m</p>
      </div>
    </div>
  );
};

export default Locations;
