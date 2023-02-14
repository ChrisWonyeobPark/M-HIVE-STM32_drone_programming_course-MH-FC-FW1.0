+++++ M-HIVE Ground Station V0.9.7d +++++

°ÿ This GCS only works on Windows OS. Developed in the .NET Framework 4 environment.

< How to use >
  1. Connect 3DR Telemetry module to PC via USB. (It is recommended to install the latest CP210x driver)
  2. After setting the comport and baud rate appropriately, Open comport.
  3. Data transmission and reception follows FC°ÍGCS communication protocol V0.9.1 message frame.
    (Refer to STM32 Drone Programming course FC°ÍGCS communication protocol v0.9.1.pdf file)
  4. Three files must exist in the same folder: GMap.NET.Core.dll, GMap.NET.WindowsForms.dll, ZedGraph.dll

  5. Described how to use in the "STM32 Drone Programmin A to Z" online course.
    https://www.udemy.com/course/stm32-drone-programming/?referralCode=E24CB7B1CD9993855D45


< GCS features >
1. Comport setting and terminal
  - Port and baud rate setting
  - Available sending/receiving Ascii and Hex via terminal function

2. Googld map (Gmap.net)
  - Current coordinate display as a marker
  - Current coordinate latitude and longitude display as texts
  - Show previous 30 markers
  - Show current heading

3. PID gain setting
  - PID gain transmission
  - PID gain request
  - Automatic gain file saving by pressing the gain send button (\Gain Log folder)

4. 2D-line graph visualization (ZedGraph)
  - Roll/Pitch/Yaw/Altitude line graph
  - Simultaneous graph visualization of reference and setpoint for PID control performance validation
  - Sensor log file saving function (\Sensor Log folder)

5. Drone status display and other functions
  - GCS °Í 3DR telemetry module wired connection status indication
  - GCS °Í FC wireless connection status indication and communication fail alarm
  - Battery voltage indication and low voltage alarm
  - FS-i6 SWA, SWC status display
  - FS-i6 °Í iA6B Fail-safe status indication and alarm

  - When launching the GCS, the latest transmitted gains are automatically loaded (saved in \Gain Log\Latest.ini)


< List of known issues >
1. If the USB device is removed while the comport is connected, it is impossible to disconnect the comport.
  °Ê Need to re-launch the GCS.

2. An issue that slows down the software when activating the terminal function.
  °Ê Activate the terminal only when necessary, otherwise disable the terminal.

3. An issue when transmitting HEX using terminal, an error occurs if there is a blank in the most front and back of the data to be sent.
  °Ê When transmitting HEX, to be sure there is no blank in the most front and back of data.

4. An issue that the graph slows down when data is received too quickly.
  °Ê It's recommended that FC sends messages at up to 50 Hz

5. An issue that sometimes strange PID gain received from FC is indicated when transmitting PID gain.
  °Ê Request gain from FC and be sure to check the gain currently set in FC.

6. An issue that the graph is not drawn if the value is 0.
  °Ê If data is received but no graph is drawn, regard the value is 0.

Copyright(c) 2023, ChrisP @ M-HIVE

===== M-HIVE Ground Station Update History =====
< V0.9.7d >
  - English version first release
  + Feb 14, 2023

< V0.9.7c >
  - Fixed a bug that the map could not be loaded
  + Nov 9, 2021

< V0.9.7b >
  - Language selection function added - Korean, English
  - When changing the language to English, the STM32 Drone Programming online course link is changed - Inflearn for Korean, Udemy for English.
  - Changed the purchase link for MH-FC V2.2 and STM32F4 Evaluation kit - It is no longer connected to Naver Cafe but Smart Store.
  + 21st Apr, 2021

< V0.9.7 >
  - Changed to ZedGraph API version from 5.1.5 to 5.1.7 °Ê Refer to https://www.nuget.org/packages/ZedGraph/ for ZedGraph API updates.
  - Fixed an issue where the roll and pitch gains were loaded with the same value at the first run of GCS. (Loading \\Gain Log\Latest.ini)
  - Fixed an issue when calculating the checksum of the received message, the value of byte 18 was not used for the checksum calculation.
  + Mar 22, 2020

< V0.9.6 >
  - First release
  + Jan 28, 2020