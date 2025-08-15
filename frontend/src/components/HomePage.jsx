import React, { useState } from "react";
import Header from "./Header";
import Camera from "./Camera";
import Buttons from "./Buttons";
import VehicleTracking3D from "./VehicleTracking3D";
import Locations from "./Locations";

function HomePage() {
  const [aprilTagPosition, setAprilTagPosition] = useState(null);
  const [sensorPosition, setSensorPosition] = useState(null);

  return (
    <div>
      <Header />
      <div className="camera-container">
        <Camera
          aprilTagPosition={aprilTagPosition}
          sensorPosition={sensorPosition}
        />
        <VehicleTracking3D
          setAprilTagPosition={setAprilTagPosition}
          setSensorPosition={setSensorPosition}
        />
      </div>
      <div className="button-container">
        <Buttons />
        <Locations
          aprilTagPosition={aprilTagPosition}
          sensorPosition={sensorPosition}
        />
      </div>
    </div>
  );
}

export default HomePage;
