import { useState, useEffect } from "react";
import "../css/Camera.css";


function Camera() {
  const [imageSrc, setImageSrc] = useState("http://raspberrypi.local:5000/video_feed?width=320&height=240");

  useEffect(() => {
    const interval = setInterval(() => {
      setImageSrc(`http://raspberrypi.local:5000/video_feed?timestamp=${new Date().getTime()}`);
    }, 100);

    return () => clearInterval(interval);
  }, []);

  return (
    <div className="camera-container">
      <div className="video-section">
        <img 
          src={imageSrc} 
          alt="Live Camera Feed" 
          width="640"   
          height="360"  
          style={{ objectFit: 'cover' }}  
        />
      </div>

      
    </div>
  );
}

export default Camera;
