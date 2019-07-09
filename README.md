# Project Perceptbot

This repository is the companion repository for the Project Perceptbot (paper forthcoming).

## Table of contents

This project has been divided into several components, organised into the directories you can see above. For information specific to each part of the project (which were developed in parallel by different team members), see the individual READMEs within each directory.

We strongly recommend acquiring a good understanding of ![ROS](https://www.ros.org/about-ros/) (Robot Operating System) before delving into the contents of these packages. The organisation of the directories reflects a typical ROS hierarchy.

## Dependencies

Some dependencies are listed in individual READMEs. Most are handled by ROS, which can be configured to automatically import dependencies. 

In addition, there are some non-ROS-related dependencies. The `vision` materials require the `opencv` and `openVINO` libraries.
The `web_interface` directory comes bundled with its dependencies. The `motor_controller` material *should* require no additional dependencies to be installed. The `navigation_stack` is really designed to be used with various diffeent ROS navigation packages (i.e. it is implementation-agnostic).
