# Protogen Ear Twitch

This project implements servo-driven ear-twitching functionality using an ESP32-S3 DevKitC-1 and two servo motors. The servos simulate random ear twitches with adjustable parameters.

Features

	•	Control of two servos using the ESP32-S3 board.
	•	Randomized ear-twitching motion with customizable timing and positions.
	•	Uses the PlatformIO environment for easy development and flashing.

Requirements

	•	ESP32-S3 WROOM-1 R8N16 board
	•	2x Servos
	•	PlatformIO
	•	ESP32Servo library

Installation

1.	Clone the repository:
   `git clone https://github.com/stef1949/Protogen-Ear-Twitch.git`


3.	Open the project in PlatformIO.
4.	Install the required libraries:
	platformio lib install ESP32Servo
5.	Upload the code to your ESP32-S3 board.

Configuration

Modify the platformio.ini file to configure the project for your specific board and adjust the parameters in the source files as needed.

Usage

	1.	Connect the servos to the correct GPIO pins as defined in the code.
	2.	Power the board and observe the ear twitching simulation.
	3.	Customize the twitch range and timing by modifying variables in the code.

License

This project is licensed under the MIT License. See the LICENSE file for details.

Let me know if you’d like to add more details!
