import { Button } from "@mui/material";
import React from "react";
import "../css/button.css";
import ArrowDownwardIcon from "@mui/icons-material/ArrowDownward";
import ArrowForwardIcon from "@mui/icons-material/ArrowForward";
import ArrowUpwardIcon from "@mui/icons-material/ArrowUpward";
import ArrowBackIcon from "@mui/icons-material/ArrowBack";

const BASE_URL = "http://raspberrypi.local:5000";
function Buttons() {
  const sendCommand = async (command) => {
    try {
      const response = await fetch(`${BASE_URL}/${command}`, { method: "GET" });
      if (!response.ok) {
        throw new Error(`Error: ${response.statusText}`);
      }
      console.log(`${command} command sent successfully`);
    } catch (error) {
      console.error("Failed to send command", error);
    }
  };
  return (
  <div className="button-container">
    <div className="button-group">
      <div className="forward-wrapper">
        <Button
          className="forward-button"
          color="inherit"
          size="large"
          variant="contained"
          startIcon={<ArrowUpwardIcon />}
          onClick={() => sendCommand("forward")}
        >
          Forward
        </Button>
      </div>
      <div className="horizontal-button">
        <Button
          color="inherit"
          size="large"
          variant="contained"
          startIcon={<ArrowBackIcon />}
          onClick={() => sendCommand("left")}
        >
          Left
        </Button>
        <Button
          color="inherit"
          size="large"
          variant="contained"
          startIcon={<ArrowDownwardIcon />}
          onClick={() => sendCommand("backward")}
        >
          Backward
        </Button>
        <Button
          color="inherit"
          size="large"
          variant="contained"
          startIcon={<ArrowForwardIcon />}
          onClick={() => sendCommand("right")}
        >
          Right
        </Button>
      </div>
    </div>
  </div>
);
}

export default Buttons;
