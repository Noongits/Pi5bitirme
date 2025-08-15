import React, { useRef, useEffect, useState } from "react";
import "../css/VehicleTracking3D.css";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader";

const VehicleTracking3D = ({ setAprilTagPosition, setSensorPosition }) => {
  const mountRef = useRef(null);
  const [logData, setLogData] = useState([]);
  const [tags, setTags] = useState({});
  const [initialTagPositions, setInitialTagPositions] = useState(null);

  const jsonToCSV = (data) => {
    if (data.length === 0) return "";
    const headers = Object.keys(data[0])
      .map((key) => {
        if (typeof data[0][key] === "object") {
          return Object.keys(data[0][key]).map((subKey) => `${key}.${subKey}`);
        }
        return key;
      })
      .flat();
    const rows = data.map((row) =>
      headers
        .map((header) => {
          const [mainKey, subKey] = header.split(".");
          const value =
            subKey && row[mainKey] ? row[mainKey][subKey] : row[mainKey];
          return typeof value === "number" ? value : `"${value || ""}"`;
        })
        .join(",")
    );
    return [headers.join(","), ...rows].join("\n");
  };

  useEffect(() => {
    const THREE = window.THREE;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf0f0f0);

    const camera = new THREE.PerspectiveCamera(
      60,
      mountRef.current.clientWidth / mountRef.current.clientHeight,
      0.1,
      1000
    );
    camera.position.set(0, 10, -10);
    camera.lookAt(0, 1, 0);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(
      mountRef.current.clientWidth,
      mountRef.current.clientHeight
    );
    renderer.shadowMap.enabled = true;
    mountRef.current.appendChild(renderer.domElement);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
    directionalLight.position.set(5, 10, 5);
    directionalLight.castShadow = true;
    scene.add(directionalLight);

    const ambientLight = new THREE.AmbientLight(0x404040, 1);
    scene.add(ambientLight);

    const gridHelper = new THREE.GridHelper(30, 30);
    scene.add(gridHelper);

    const material = new THREE.MeshStandardMaterial({ color: 0x888888 });
    const loader = new STLLoader();
    loader.load(
      "eyfelkulesi.stl",
      (geometry) => {
        const mesh = new THREE.Mesh(geometry, material);
        mesh.scale.set(0.02, 0.02, 0.02);
        mesh.position.set(-2, 1, 6,5);      //mesh.position.set(1.8, 0, 3);
        mesh.rotation.x = -Math.PI / 2;
        mesh.castShadow = true;
        mesh.receiveShadow = true;

        
        scene.add(mesh);
      },
      undefined,
      (error) => console.log(error)
    );
    /*loader.load(
      "truck.stl",
      (geometry) => {
        const mesh = new THREE.Mesh(geometry, material);
        mesh.scale.set(0.4, 0.4, 0.4);
        mesh.position.set(1.8, 0, 3);
        mesh.rotation.z = -Math.PI / 2;
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        scene.add(mesh);
      },
      undefined,
      (error) => console.log(error)
    );*/

    const createCar = (color) => {
      const group = new THREE.Group();
      const body = new THREE.Mesh(
        new THREE.BoxGeometry(1.5, 0.5, 3),
        new THREE.MeshStandardMaterial({ color })
      );
      body.castShadow = true;
      body.receiveShadow = true;
      group.add(body);

      const wheelGeometry = new THREE.CylinderGeometry(0.3, 0.3, 0.5, 16);
      const wheelMaterial = new THREE.MeshStandardMaterial({ color: 0x333333 });

      [-0.7, 0.7].forEach((x) => {
        [-1, 1].forEach((z) => {
          const wheel = new THREE.Mesh(wheelGeometry, wheelMaterial);
          wheel.rotation.z = Math.PI / 2;
          wheel.position.set(x, -0.3, z);
          wheel.castShadow = true;
          wheel.receiveShadow = true;
          group.add(wheel);
        });
      });

      group.scale.set(0.5, 0.5, 0.5);
      return group;
    };

    const redCar = createCar("red");
    const blueCar = createCar("blue");
    scene.add(redCar);
    scene.add(blueCar);

    const tagCubes = {};
    for (let i = 0; i <= 5; i++) {
      const tagGeometry = new THREE.BoxGeometry(0.8, 0.8, 0.8);
      const tagMaterial = new THREE.MeshStandardMaterial({ color: 0x00ff00 });
      const cube = new THREE.Mesh(tagGeometry, tagMaterial);
      cube.visible = false;
      scene.add(cube);
      tagCubes[`tag_${i}`] = cube;
    }

    const legendDiv = document.createElement("div");
    legendDiv.style.position = "absolute";
    legendDiv.style.top = "10px";
    legendDiv.style.left = "10px";
    legendDiv.style.padding = "6px 12px";
    legendDiv.style.background = "#eee";
    legendDiv.style.borderRadius = "8px";
    legendDiv.innerHTML = `
      <span style="color: red; font-weight: bold">🚗</span> AprilTag Based Position (Red Car) &nbsp;&nbsp;
      <span style="color: blue; font-weight: bold">🚘</span> Wheel Speed Sensor Based Position (Blue Car) &nbsp;&nbsp;
      <span style="color: green">■</span> April Tags
    `;
    mountRef.current.appendChild(legendDiv);

    const animate = () => {
      requestAnimationFrame(animate);

      if (window.latestAprilTagPosition && window.latestSensorPosition) {
        const { x: ax, y: ay, z: az } = window.latestAprilTagPosition;
        const { x: sx, y: sy, z: sz } = window.latestSensorPosition;
        const aDir = window.latestDirection;

        redCar.position.set(ax * 4, ay, az * 4);
        blueCar.position.set(sx * 4, sy, sz * 4);

        if (typeof aDir === "number") {
          const target = THREE.MathUtils.degToRad(aDir);
          redCar.rotation.y = THREE.MathUtils.lerp(redCar.rotation.y, target, 0.1);
        }
        if (typeof aDir === "number") {
          const target = THREE.MathUtils.degToRad(aDir);
          blueCar.rotation.y = THREE.MathUtils.lerp(redCar.rotation.y, target, 0.1);
        }
 
        if (initialTagPositions) {
          Object.entries(initialTagPositions).forEach(([key, value]) => {
            const cube = tagCubes[key];
            if (cube && value) {
              const isZero = value.x === 0 && value.y === 0 && value.z === 0;
              cube.visible = !isZero;
              if (!isZero) {
                cube.position.set(value.x * 4, value.y, value.z * 4);
              }
            }
          });
        }
      }

      renderer.render(scene, camera);
    };

    animate();
    return () => {
      mountRef.current.removeChild(renderer.domElement);
      if (legendDiv) mountRef.current.removeChild(legendDiv);
    };
  }, [initialTagPositions]);

  useEffect(() => {
    const fetchData = async () => {
      try {
        const response = await fetch("http://raspberrypi.local:5000/positions");
        const data = await response.json();
        
        console.log("AprilTag direction:", data.direction);
        console.log("Sensor direction:", data.direction);

        setAprilTagPosition(data.aprilTagPosition);
        setSensorPosition(data.sensorPosition);

        window.latestAprilTagPosition = data.aprilTagPosition;
        window.latestSensorPosition = data.sensorPosition;
        window.latestDirection = data.direction;

        if (!initialTagPositions) {
          const tagData = {};
          for (let i = 0; i <= 5; i++) {
            tagData[`tag_${i}`] = data[`tag_${i}`];
          }
          setTags(tagData);
          setInitialTagPositions(tagData);
        }

        setLogData((prevLog) => [
          ...prevLog,
          {
            timestamp: new Date().toISOString(),
            aprilTagPosition: data.aprilTagPosition,
            sensorPosition: data.sensorPosition,
            direction: data.direction,
            ...tags,
          },
        ]);
      } catch (error) {
        console.error("Error fetching data:", error);
      }
    };

    fetchData();
    const interval = setInterval(fetchData, 1000);
    return () => clearInterval(interval);
  }, [setAprilTagPosition, setSensorPosition, initialTagPositions, tags]);

  return (
    <div className="vehicle-3d-wrapper">
      <div ref={mountRef} />
      <button
        onClick={() => {
          const csv = jsonToCSV(logData);
          const blob = new Blob([csv], { type: "text/csv" });
          const url = URL.createObjectURL(blob);
          const a = document.createElement("a");
          a.href = url;
          a.download = "logData.csv";
          a.click();
        }}
        className="csv-download-button"
      >
        Log Data
      </button>
    </div>
  );
};

export default VehicleTracking3D;
