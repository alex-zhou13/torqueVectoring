# torqueVectoring

EC463/EC464 Fall 2022 - Spring 2023

Group 9- Torque Vectoring

Team Members: Will Krska, Jonathan Ye, Nick Marchuk, Giacomo Corraluppi, Alex Zhou

The goal of this project is to create a fail-safe torque vectoring control system for motorsports teams racing in Formula SAE. This system applies a differential torque to the two independent drives of the two rear wheels of the car to improve cornering speed and performance, instead of relying on one power source to control the vehicle dynamics. Before using any of the code found in this repository, you must first make sure that all the microcontrollers and components are correctly wired up to each other, otherwise the code will not work as expected. Next, download the SrDes-aux_controller folder and upload the code to the Arduino Nano by using the Arduino IDE. Next, download the SrDes-main_controller folder and upload the code to the corresponding ESP32-S3 microcontroller located in the main controller box using ESP-IDF. Do the same for the SrDes-watchdog folder and upload it to the microcontroller located directly beneath the main controller ESP. Finally, download the SrDes-motor_controller folder and upload the code to the ESP32-S3 sitting below the demonstration vehicle. 

A few of the biggest issues we ran into when creating this system was interfacing between all the sensors and microcontrollers used in the project. We spent a long time trying to use an ESP32-S3 to interface with our Adafruit accelerometer, only to realize we needed Arduino libraries so it was much more effective to use an Arduino Nano instead. We also ran into issues trying to send two separate signals to two different DACs, and our solution to this problem was to use different registers for each of the DACs. Another problem we encountered was integrating the generated code from the Simulink model into the main controller, and the main thing to look out for here is making sure all the functions are being used, and the variable names for the input data are changed to match their corresponding parameters defined in the main controller. Lastly, the biggest suggestion we have for anyone trying to replicate or improve on our project is to start with a simpler model and slowly make it more complicated rather than starting with the most complicated system from the start because the errors will be very hard to debug given the hardware configuration of our systems. 

Currently, this system has only been tested on a 3D demonstration vehicle model prototype. Our client, Terrier Motosports of Boston University, is currently in development of their next generation formula race car that can fit a human being inside and race down a track. The ultimate end goal is to install these electronics onto any real life-sized formula race car that uses independently-controlled motors for its wheels, such as the one under development. Our hope is that this system serves as a more affordable alternative to proprietary solutions already in existence, while being modular and simpler for configuration across various formula specifications, both physically and algorithmically.

