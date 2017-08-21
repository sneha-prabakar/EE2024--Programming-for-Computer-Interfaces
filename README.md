# EE2024
# Project Description:

To create a system, known as Care Unit for The Elderly (shortly, CUTE).  CUTE is a portable and cost-effective device that elderly people from all economic backgrounds can use. It requires very less technical expertise and hence, easy for the elderly to use. Any issues that the elderly face in their homes, would be sent wirelessly to an agency known as Centralized Elderly Monitoring System (CEMS) so that they can proactively take measures to solve the issue. 

CUTE is implemented on a LPC1769 board using several external peripherals such as temperature sensor, accelerometer, light sensor, XBee, etc. to monitor the elderly’s movement and the environment in which they live in.

Some of the features of CUTE include:
  - 1.Ambient Lighting System 
  - 2.'Help' message sender to an external system via wireless network; so that the elderly can call upon help whenever they want by simply pressing a button
  - 3.Alarm system in case of:
    - 3.1. Fire
    - 3.2. Movement in darkness
    - 3.3. Regular monitoring of the elderly's movement and sending the data to an external system (which can be a doctor/relative), so that they can know when the elderly person has fallen somewhere. 
    - 3.4. Customized user-friendly interface

CUTE also simplifies the way the elderly can take control of their environment and change it to suit their own needs.All these monitoring operations can be performed only when the elderly are alone at home (i.e. in the MONITOR mode), as they are personally taken care of in the presence of a caretaker in the STABLE mode. CUTE makes use of polling at regular time intervals and interrupts to collect data and transmit it to the CEMS in the MONITOR mode.

Detailed information of the features and how they were implemented can be found in the 'Project Report' document. 
