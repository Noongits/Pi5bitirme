import React, { useState } from "react";
import "../css/LoginPage.css";
import { FaUser, FaLock } from "react-icons/fa";
import { useNavigate } from "react-router-dom";

export const users = [
  { username: "sinan", password: "1234" },
  { username: "admin", password: "admin123" },
];

const LoginPage = () => {
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const navigate = useNavigate();

  const handleLogin = (e) => {
    e.preventDefault();

    const isValidUser = users.some(
      (user) => user.username === username && user.password === password
    );

    if (isValidUser) {
      navigate("/home");
    } else {
      alert("Invalid username or password");
    }
  };

  return (
    <div className="wrapper">
      <form onSubmit={handleLogin}>
        <h1>Vehicle Control System</h1>
        <div className="input-box">
          <input
            type="text"
            placeholder="Username"
            required
            value={username}
            onChange={(e) => setUsername(e.target.value)}
          />
          <FaUser className="icon" />
        </div>
        <div className="input-box">
          <input
            type="password"
            placeholder="Password"
            required
            value={password}
            onChange={(e) => setPassword(e.target.value)}
          />
          <FaLock className="icon" />
        </div>

        <button type="submit">Enter</button>
      </form>
    </div>
  );
};

export default LoginPage;
