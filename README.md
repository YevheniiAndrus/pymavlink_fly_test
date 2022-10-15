# pymavlink_fly_test

OS: Ubuntu 20.04 TLS

1. Clone ardupilot repository <br>
   ```
   git clone https://github.com/ArduPilot/ardupilot.git <br>
   cd ardupilot <br>
   git submodule update --init --recursive <br>
   ```

2. Install all needed dependencies <br>
   From repo root run <br>
   ```Tools/environment_install/install-prereqs-ubuntu.sh -y``` <br>
   
   Reload the path (log-out and log-in to make permanent): <br>
   ```. ~/.profile```
   

3. Configuring autopilot: <br>
   From repo root <br>
   ```./waf configure --board sitl``` # software-in-the-loop simulator <br>
   ```./waf plane```                  # Fixed wing airplanes including VTOL
   

4. Download QGroundControl <br>
   ```https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html```
   
   
5. Clone fly script <br>
   ```git clone git@github.com:YevheniiAndrus/pymavlink_fly_test.git```
   
   
6. Run simualtion and QGroundControl <br>
   Open two termonals. <br>
   In the first ```run sim_vehicle.py -v ArduPlane``` <br>
   In the second go to folder with QGroundControl and run it ```./QGroundControl```
   
   
7. Run fly.py script <br>
   Wait until Ready To fly message appear in QGroundControl in the top left corner <br>
   go to pymavlink_fly_test folder <br>
   run ```python3 fly.py```
