# PRE-SAFE: Precise Self-localization and Improved Obstacle Detection for Autonomous Vehicles

PRE-SAFE is an advanced autonomous vehicle system designed for precise self-localization and obstacle detection using multiple sensor fusion techniques. The system combines computer vision, inertial measurement, wheel odometry, and landmark-based navigation to achieve robust autonomous navigation.

## 🎯 Project Overview

PRE-SAFE implements a multi-layered localization approach that prioritizes accuracy through sensor fusion:

1. **AprilTag Localization** (Highest Priority) - Visual marker-based positioning
2. **Landmark-based Localization** - Eiffel Tower model detection and ranging
3. **Sensor-based Localization** - IMU and wheel odometry fusion

The system features a **full-stack architecture** with a Raspberry Pi backend for sensor processing and vehicle control, paired with a modern React frontend for real-time monitoring and control.

## 🏗️ System Architecture

### Backend (Raspberry Pi)
Located in `/backend/src/`:

- **Main Controller** (`main.py`) - Multi-threaded system orchestrator
- **Navigation System** (`navigation.py`) - Path planning and execution
- **Localization Engine** (`localisation.py`) - Multi-sensor position estimation
- **Motor Controller** (`motor_controller.py`) - 4-wheel drive control system
- **Flask API** (`web.py`) - RESTful API for frontend communication

### Frontend (React Application)
Located in `/frontend/src/`:

- **Vehicle Control Interface** - Real-time vehicle operation dashboard
- **3D Visualization** (`VehicleTracking3D.jsx`) - Three.js-based real-time tracking
- **Live Camera Feed** (`Camera.jsx`) - Streaming video display
- **Position Monitoring** (`Locations.jsx`) - Multi-sensor position display
- **Authentication System** (`LoginPage.jsx`) - Secure access control

### Sensor Systems (Backend)

#### Visual Localization
- **AprilTag Detection** (`apriltag_detection.py`) - Fiducial marker recognition
- **Stereo Vision** (`stereo_distance.py`) - Dual-camera distance measurement
- **Monocular Distance** (`mono_distance.py`) - Single-camera ranging
- **Landmark Detection** (`detect_and_crop.py`) - Eiffel Tower identification

#### Inertial & Odometry
- **IMU Integration** (`gyroscope.py`, `mpu6050.py`) - MPU6050-based orientation tracking
- **Wheel Sensors** (`wheel_sensor.py`) - Encoder-based odometry

### Navigation Algorithms (Backend)
- **Manhattan Navigation** (`manhattan_navigation.py`) - Grid-based path planning
- **Road Network Simulation** (`simulate_road_network.py`) - Virtual road generation

## 🚀 Features

### Backend Capabilities

#### Multi-Modal Localization
- **AprilTag System**: 8cm fiducial markers with known world coordinates
- **Visual Landmarks**: Eiffel Tower model detection with angle classification
- **Sensor Fusion**: IMU + wheel odometry with drift correction
- **Hierarchical Estimation**: Automatic fallback between localization methods

#### Advanced Navigation
- **Manhattan Grid Navigation**: Structured path planning on virtual road networks
- **Real-time Path Planning**: Dynamic destination setting and route calculation
- **Precision Turning**: IMU-guided angular control with tolerance-based stopping

#### API Services
- **RESTful API**: Flask-based backend serving sensor data and control endpoints
- **Live Video Streaming**: Real-time camera feed at `/video_feed`
- **Position Tracking**: JSON API for all sensor data at `/positions`
- **Multi-threaded Architecture**: Concurrent sensor processing and control

### Frontend Features

#### Interactive Dashboard
- **Secure Login System**: Username/password authentication for system access
- **Real-time Control Panel**: Intuitive directional control buttons (Forward/Backward/Left/Right)
- **Live Camera Integration**: Streaming video feed from vehicle cameras
- **Multi-sensor Position Display**: Real-time coordinates from AprilTag and wheel sensors

#### 3D Visualization System
- **Three.js Integration**: Real-time 3D scene rendering with vehicle tracking
- **Dual Vehicle Representation**: 
  - Red car for AprilTag-based positioning
  - Blue car for wheel sensor-based positioning
- **Interactive 3D Environment**: Grid-based coordinate system with landmark models
- **AprilTag Visualization**: Green cubes representing detected AprilTag positions
- **Data Logging**: CSV export functionality for position and sensor data

#### Modern UI/UX
- **Material-UI Components**: Professional interface with consistent design
- **Responsive Layout**: Optimized for various screen sizes
- **Real-time Updates**: Live position updates every second
- **Visual Feedback**: Color-coded position indicators and status displays

## 📋 Hardware Requirements

### Computing Platform
- Raspberry Pi 4B (recommended) or compatible SBC
- MicroSD card (32GB+, Class 10)

### Sensors & Actuators
- **Cameras**: Dual Raspberry Pi Camera modules (stereo configuration)
- **IMU**: MPU6050 6-axis accelerometer/gyroscope
- **Wheel Encoders**: Optical/magnetic encoders (20 pulses/revolution)
- **Motors**: 4x DC motors with H-bridge drivers
- **GPIO**: Motor control pins configured for 4-wheel drive

### Physical Setup
- **Camera Baseline**: 51mm stereo separation
- **Wheel Diameter**: 65mm
- **AprilTag Size**: 8cm (tag25h9 family)
- **Test Environment**: Indoor with known AprilTag positions

## 🛠️ Installation & Setup

### Backend Setup (Raspberry Pi)

#### System Dependencies
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Python dependencies
sudo apt install python3-pip python3-opencv python3-flask python3-numpy

# Install specialized libraries
pip3 install picamera2 apriltag RPi.GPIO flask-cors pillow
```

#### Hardware Configuration
```bash
# Enable camera interface
sudo raspi-config
# Navigate to Interface Options > Camera > Enable

# Enable I2C for IMU
sudo raspi-config
# Navigate to Interface Options > I2C > Enable

# Reboot system
sudo reboot
```

#### Calibration Setup
1. **Camera Calibration**: Run scripts in `backend/src/cam_calibration/`
2. **IMU Calibration**: System auto-calibrates on startup (2000 samples)
3. **AprilTag Placement**: Position tags according to coordinates in `backend/src/variables.py`

### Frontend Setup (Development Machine)

#### Prerequisites
- Node.js 16+ and npm
- Modern web browser with WebGL support

#### Installation
```bash
cd frontend
npm install
```

#### Dependencies
- **React 19**: Modern UI framework
- **Material-UI**: Professional component library
- **Three.js**: 3D visualization and rendering
- **React Router**: Client-side routing
- **React Icons**: Icon components

#### Configuration
Update the Raspberry Pi IP address in:
- `frontend/package.json` - proxy setting
- `frontend/src/components/` - API endpoint URLs

## 🏃‍♂️ Running the System

### Backend (Raspberry Pi)
```bash
cd backend
python3 src/main.py
```

#### System Components
The main script launches multiple concurrent threads:
- **Camera Capture**: Dual camera frame acquisition
- **Flask API Server**: RESTful API on `http://[PI_IP]:5000`
- **Navigation**: Autonomous path planning and execution
- **Sensor Processing**: IMU, encoders, and visual localization
- **Output Management**: Data logging and status reporting

#### Navigation Modes
Configure in `backend/src/variables.py`:
- `nav_mode = 0`: Straight line navigation
- `nav_mode = 1`: Manhattan grid navigation (default)
- `nav_mode = 2`: AprilTag-based navigation
- `nav_mode = 3`: Landmark-based navigation

### Frontend (Development)
```bash
cd frontend
npm start
```
Access the dashboard at `http://localhost:3000`

#### Production Build
```bash
cd frontend
npm run build
# Deploy the build/ folder to your web server
```

### Full System Operation

1. **Start Backend**: Launch the Python backend on Raspberry Pi
2. **Start Frontend**: Run the React development server or deploy production build
3. **Access Dashboard**: Navigate to the frontend URL
4. **Login**: Use credentials (default: `sinan/1234` or `admin/admin123`)
5. **Control Vehicle**: Use the intuitive control interface

#### Key URLs
- **Frontend Dashboard**: `http://localhost:3000` (development)
- **Backend API**: `http://[RASPBERRY_PI_IP]:5000`
- **Live Camera Feed**: `http://[RASPBERRY_PI_IP]:5000/video_feed`
- **Position Data API**: `http://[RASPBERRY_PI_IP]:5000/positions`

### Authentication
Default login credentials:
- Username: `sinan` | Password: `1234`
- Username: `admin` | Password: `admin123`

Update credentials in `frontend/src/components/LoginPage.jsx`

## 📊 Localization Hierarchy

The system implements a three-tier localization hierarchy:

### 1. AprilTag Localization (Primary)
- **Accuracy**: Highest precision (±2cm typical)
- **Method**: PnP pose estimation from known fiducial markers
- **Coverage**: Limited to marker visibility range
- **Fallback**: When no tags detected

### 2. Landmark Localization (Secondary)
- **Target**: Eiffel Tower scale model
- **Method**: Object detection + stereo/mono distance measurement
- **Angle Classification**: Neural network-based orientation estimation
- **Range**: Medium-distance landmark navigation

### 3. Sensor Fusion (Baseline)
- **Components**: MPU6050 IMU + wheel encoders
- **Method**: Dead reckoning with drift correction
- **Reliability**: Continuous availability, accumulates error over time
- **Correction**: Reset by higher-priority localization methods

## 🔧 Configuration

### Backend Configuration (`backend/src/variables.py`)
```python
# AprilTag world coordinates (meters)
APRILTAG_COORDS = {
    0: [0, 0.0, 2.2],      # Origin tag
    2: [-0.6, 0.0, 2.2],   # Left reference
    3: [1.2, 0.0, 1.32],   # Right reference
    # ... additional tags
}

# Navigation settings
nav_mode = 1  # Manhattan navigation
roadnet_height, roadnet_width = 20, 20  # Grid dimensions

# Camera parameters
resolution = (1280, 960)
stereo_baseline = 51  # mm
```

### Motor Control Pins (GPIO BCM)
```python
# Front motors
FRONT_LEFT_PIN1, FRONT_LEFT_PIN2 = 26, 21
FRONT_RIGHT_PIN1, FRONT_RIGHT_PIN2 = 20, 16

# Rear motors  
BACK_LEFT_PIN1, BACK_LEFT_PIN2 = 5, 6
BACK_RIGHT_PIN1, BACK_RIGHT_PIN2 = 1, 7
```

### Frontend Configuration
Update Raspberry Pi IP addresses in:

#### `frontend/package.json`
```json
{
  "proxy": "http://192.168.6.62:5000"
}
```

#### Component API Endpoints
```javascript
// Camera feed
const imageSrc = "http://raspberrypi.local:5000/video_feed";

// Position data
const response = await fetch("http://raspberrypi.local:5000/positions");

// Control commands
const BASE_URL = "http://raspberrypi.local:5000";
```

## 📁 Project Structure

```
Pi5bitirme/
├── backend/                      # Raspberry Pi Backend
│   ├── src/                      # Core system modules
│   │   ├── main.py               # System orchestrator
│   │   ├── navigation.py         # Path planning & execution
│   │   ├── localisation.py       # Multi-sensor position estimation
│   │   ├── motor_controller.py   # 4WD motor control
│   │   ├── web.py                # Flask API server
│   │   ├── apriltag_detection.py # Fiducial marker processing
│   │   ├── stereo_distance.py    # Dual-camera ranging
│   │   ├── mono_distance.py      # Single-camera distance
│   │   ├── gyroscope.py          # IMU orientation tracking
│   │   ├── wheel_sensor.py       # Encoder odometry
│   │   ├── detect_and_crop.py    # Landmark detection
│   │   ├── manhattan_navigation.py # Grid path planning
│   │   ├── variables.py          # Global configuration
│   │   └── cam_calibration/      # Camera calibration tools
│   │       ├── camera_calibration.py
│   │       └── capture_dual_cam.py
│   └── scripts/                  # Testing & calibration utilities
│       ├── tagtest_localization.py
│       ├── dual_camera_capture.py
│       └── distance_test.py
├── frontend/                     # React Frontend Application
│   ├── public/                   # Static assets
│   │   ├── index.html           # Main HTML template
│   │   ├── eyfelkulesi.stl      # 3D Eiffel Tower model
│   │   └── truck.stl            # 3D vehicle model
│   ├── src/                     # React source code
│   │   ├── components/          # React components
│   │   │   ├── HomePage.jsx     # Main dashboard
│   │   │   ├── LoginPage.jsx    # Authentication
│   │   │   ├── VehicleTracking3D.jsx # 3D visualization
│   │   │   ├── Camera.jsx       # Live video feed
│   │   │   ├── Buttons.jsx      # Control interface
│   │   │   ├── Locations.jsx    # Position display
│   │   │   └── Header.jsx       # Navigation header
│   │   ├── css/                 # Component styles
│   │   ├── assets/              # Images and resources
│   │   ├── App.jsx              # Main App component
│   │   └── index.jsx            # Entry point
│   ├── package.json             # Dependencies & scripts
│   └── README.md                # Frontend documentation
└── README.md                    # Main project documentation
```

## 🧪 Testing & Validation

### Backend Testing
```bash
cd backend

# AprilTag precision test
python3 scripts/tagtest_localization.py

# Distance measurement validation
python3 scripts/distance_test.py

# Dual camera stereo verification
python3 scripts/dual_camera_capture.py

# Manual control testing
python3 scripts/manual_stop.py
```

### Frontend Testing
```bash
cd frontend

# Run test suite
npm test

# Component testing
npm run test -- --coverage

# Build verification
npm run build
```

### System Integration Testing
1. **Backend-Frontend Communication**: Verify API endpoints respond correctly
2. **Real-time Data Flow**: Ensure position updates appear in 3D visualization
3. **Control Responsiveness**: Test vehicle commands through web interface
4. **Camera Streaming**: Validate live video feed integration
5. **Data Logging**: Confirm CSV export functionality works properly

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improvement`)
3. Commit changes (`git commit -am 'Add new localization method'`)
4. Push to branch (`git push origin feature/improvement`)
5. Create Pull Request

## 📄 License

This project is part of academic research. Please refer to the accompanying thesis document (`PRE-SAFE_Thesis.pdf`) for detailed technical information and research methodology.

## 🎓 Academic Context

PRE-SAFE represents a comprehensive approach to autonomous vehicle localization, combining multiple sensor modalities for robust navigation in structured environments. The system demonstrates practical implementation of:

- **Computer Vision**: AprilTag detection and landmark recognition
- **Sensor Fusion**: Multi-modal localization with hierarchical estimation
- **Real-time Systems**: Concurrent processing and control
- **Full-stack Development**: Modern web technologies integrated with embedded systems
- **3D Visualization**: Interactive real-time vehicle tracking and monitoring

### Key Innovations
- **Multi-tier Localization Hierarchy**: Intelligent fallback between localization methods
- **Real-time 3D Visualization**: Three.js integration for live vehicle tracking
- **Modern Web Interface**: React-based dashboard with Material-UI components
- **Comprehensive Data Logging**: CSV export for research and analysis

For detailed technical analysis, experimental results, and theoretical background, please refer to the complete thesis documentation (`PRE-SAFE_Thesis.pdf`).

## 📞 Support

For technical questions or implementation assistance:
- Refer to the thesis documentation for detailed technical information
- Check the frontend README (`frontend/README.md`) for React-specific guidance
- Create an issue in the project repository for bug reports or feature requests

### System Requirements Summary
- **Backend**: Raspberry Pi 4B, Python 3.8+, dual cameras, IMU, motor controllers
- **Frontend**: Node.js 16+, modern browser with WebGL support
- **Network**: Local network connectivity between frontend and Raspberry Pi

---

**PRE-SAFE** - Advancing autonomous vehicle technology through precise localization, intelligent sensor fusion, and modern web-based interfaces.
