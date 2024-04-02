# DeerProject
### 🎓 Problem

🦌 White-tailed deer munching on the crops in fields of Delaware. I'm building a deer-chasing drone system. 

💡 Idea: a static camera spots an animal and summons a buzzing drone.

🤞 If all goes well, my tenants will have one less thing to worry about with deer unharmed.

### Hardware
- Drone Tello Ryze - for tests
- Raspberry Pi 4B
- PureThermal 3
- FLIR Lepton 3.1R - IR camera 160x120

### Set up VNC server on RPi:

To get an access to GUI of RPi you can use VNC. 

1. Install the VNC server on your Raspberry Pi:
    
    ```
    sudo apt-get update
    sudo apt-get install realvnc-vnc-server
    ```
    
2. Configure the VNC server on your Raspberry Pi:Navigate to "Interfacing Options" > "VNC" and enable the VNC server.
    
    ```
    sudo raspi-config
    ```
    
3. Start the VNC server on your Raspberry Pi:
    
    ```
    sudo systemctl start vncserver-x11-serviced.service
    ```
    
- Install VNC client: https://www.realvnc.com/en/connect/download/viewer/raspberrypi/
