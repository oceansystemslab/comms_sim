comms_sim
============

This repo holds the ROS package of the communications simulator used within the OSL. It provides a consolidated set of nodes, configuration and launch files to simulate basic communication capabilities for the OSL's vehicles. 

Guidelines
----------

Before attempting any modification to this repo please make sure that: 
  - you are working on an up-to-date version of the `master` branch
  - the previous working version of the repo has been tagged (`git tag --list` to check the tags)
  - you have considered the option of creating a new branch for your feature (`git checkout -b <branch_name>` to create a new branch), after all this is the recommended approach!
  - you know what are you doing!
  
Initially this repo is providing packages using the rosbuild build system until the OSL vehicles are migrated to an upgraded version of the ROS. Later the rosbuild support is going to be dropped and the master branch will offer a catkinized package format. The software development follows a couple of principles that have been proven useful during the early development of this project like [Semver][semver], for semantic versioning of releases and tags, [KISS][kiss], as a general guideline to prevent the commit of _huge_ files and _beast_ modules, and, finally, [TDD][tdd], as a principle for testing your code if you want to rely on a more pragmatic approach.

Requirements
------------
  - ROS HYDRO/INDIGO
  
Credits
-------

This work is part of **Nessie VIII AUV** project: *to boldly go where no robot has gone before!*

![nessie_logo](docs/nessie_tran.png)


[semver]: http://semver.org/
[kiss]: http://en.wikipedia.org/wiki/KISS_principle
[tdd]: http://en.wikipedia.org/wiki/Test-driven_development
[solid]: http://en.wikipedia.org/wiki/SOLID_(object-oriented_design)
