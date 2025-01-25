# project-annguyen9461
project-annguyen9461 created by GitHub Classroom


**Idea 1: Using AR glasses to control robot hand**
I want to use AR glasses (https://brilliant.xyz/) to control a robot hand (https://www.allegrohand.com/). The input would be my hand gestures in the field of vision of the glasses and the output is the robot hand copying my gesture. I want to try tasks like: pick and place, maybe some simple ASL letters/numbers 
The first step for me to solve this problem is using a computer vision algorithm in C++ to detect my hand gesture. I would also have to set up the control system for the hand with ROS2 and C++.

**Idea 2: Translate text visible in the user’s environment and display it in the AR glasses.**
This project aims to use augmented reality (AR) glasses that can detect text in their surroundings, translate it into users' preferred language, and overlay the translated text seamlessly onto the original context. I plan to use OpenCV for text detection and OCR (Optical Character Recognition) with Tesseract OCR, as well as Google Translate or a custom language translation API. Features I'm considering are: 
<ol>
  <li>detect and overlay translated text on signs, documents, etc.</li>
  <li>allow user to select text regions with gestures.</li>
</ol>
The first step is creating a minimally functional prototype that detects text in an image, translates it, and displays the translation in a user-friendly way. This prototype will use OpenCV to identify text regions within an input image, apply Tesseract OCR to extract the text, and integrate Google Translate to perform language translation. The output will be an annotated image where the translated text is overlaid directly on top of the original text regions.

A challenge might be the AR glasses's lack of C++ SDK so I'm trying to figure out how to code up features in C++ then port it to their Python SDK.
