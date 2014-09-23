# PERSON IDENTIFICATION 

## INTRODUCTION

Building seamless human robot collaborative environments presents a set of unique challenges. The robots not only have to understand natural commands using voice or gesture, it also has to discern the identity of the person giving commands for authentication and authorization purposes. This is makes tasks like following / working with the same person in a cluttered environment, where other humans may be present, easier.

The work will be built upon the work done by previous year Vibot students. They built a system where a Pioneer DX robot fitted with a Kinect Sensor could understand a select few pre-determined voice-cum-gesture commands and perform actions accordingly. The system was limited to one person giving commands by standing in front of the robot. The robot also lost track of the person it was following if other people intercepted in the line of sight.

The new Kinect 2 sensor gives us very high quality colour, depth and microphone array in the same package. We would be looking to combine and exploit all three modalities for person identification/re-identification and produce state of the art results on the following dataset.

RGB-D Person Re-identification Dataset [1]


### Objectives
1. Refactor the previous year’s code to work with our tool-chain comprising Point-Cloud library, QT framework, and the new Kinect 2 SDK. The skeleton code will be provided, except for the audio part. Your job will be to take the part of their code which does the gesture recognition on skeletal data and speech recognition with the audio source and make them work in this environment.

2. Use the in-built audio beam forming capability to identify the source of a command initialization phrase like “OK PIONEER” in cases where there are multiple people in the viewpoint.

3. Once the source is identified, segment that user, and extract features from Colour, Depth and Voice spectrum domains to associate with them a unique identity representation. The existing techniques in literature either uses only RGB-D [1] , or RGB-D and thermal body heat signature [2] for this task. We would be looking to use RGB-D and Audio to improve upon [1].

4. Finally,we will use the unique identifier to authenticate and authorize the user to give further commands. We will also demonstrate that we can propagate the RGB-D identifier representation when tracking and following a person even if other people come and go in the view point.

### SKILLS
Programming in C++ and may be prototyping in Matlab. A balance of team members will be required for implementing the algorithms with at least someone to build a scalable architecture to bring all the algorithms together.

### REFERENCES

[[1] Barbosa, IgorBarros; Cristani, Marco; Del Bue, Alessio; Bazzani, Loris; Murino, Vittorio, “Re-identification with RGB-D Sensors”, Computer Vision – ECCV 2012. Workshops and Demonstrations, Lecture Notes in Computer Science, 2012.](http://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=2&cad=rja&uact=8&ved=0CCkQFjAB&url=http%3A%2F%2Fwww.lorisbazzani.info%2Fpapers%2Fproceedings%2FBarbosaetal_REID12.pdf&ei=0kohVOKOJcq9ygOc8oCAAw&usg=AFQjCNEEsbJ27H_2QaWrdgQd_37ZDy8GTw&sig2=Ha1dcMGUEKnYdicGDuvEsA&bvm=bv.75775273,d.bGQ)

[[2] Mogelmose, A; Bahnsen, C.; Moeslund, T.B.; Clapes, A; Escalera, S., "Tri-modal Person Re-identification with RGB, Depth and Thermal Features," Computer Vision and Pattern Recognition Workshops (CVPRW), 2013 IEEE Conference on , vol., no., pp.301,307, 23-28 June 2013](http://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&cad=rja&uact=8&ved=0CCIQFjAA&url=http%3A%2F%2Fwww.maia.ub.es%2F~sergio%2Flinked%2Fcvpr2013trimodalpersonreid.pdf&ei=UUshVJWAJ-OBywPV64KABQ&usg=AFQjCNEWTOI-RmIAJeI-j6abQmPbrM7LNg&sig2=umYgVL5pLxsN3kooGvwcrw&bvm=bv.75775273,d.bGQ)


