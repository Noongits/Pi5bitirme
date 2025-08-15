import React from 'react'
import '../css/Header.css';
import logo from '../assets/logo.jpg';
function Header() {
  return (
    <div className='header'>
         <div className="title">PRE-SAFE</div>
         <div className="title2"> VEHICLE CONTROL SYSTEM </div>
         <img src={logo} alt="Logo" className="logo" />
    </div>
  )
}
export default Header;