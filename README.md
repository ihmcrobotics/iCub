# iCub

[![Build Status](https://travis-ci.org/ihmcrobotics/iCub.svg?branch=master)](https://travis-ci.org/ihmcrobotics/iCub)

Repository containing work adapting the IHMC Walking Controller to the iCub robot. Currently simulations only.

This repository uses SNAPSHOT builds of the required packages from IHMC Open Robotics Software (https://github.com/ihmcrobotics/ihmc-open-robotics-software).
You don't need a working copy of IHMC Open Robotics Software to use the iCub repositories. While we publish SNAPSHOT builds for the core
software regularly, we will only update the iCub dependency graph as needed for features developed in the controller to avoid thrashing
the dependency tree.

## Getting Started

We require a Java Development Kit 8 or higher, and a fairly up-to-date version of Gradle to use our software. We also highly suggest
using a modern Java IDE. More information can be found at https://ihmcrobotics.github.io/documentation/ 
(see Software Requirements on the sidebar) for installing Java, Gradle, and an IDE.

Once you have installed the IDE of your choice, you may import the iCub repository as a Gradle project.

## Tested Platforms
We test all of our software on OS X 10.12 Sierra, Windows 7/8/10, and Ubuntu 14.04 &amp; 16.04 LTS Desktop and Server.

## Branches
This repository uses the git-flow branching model. You can find more about git-flow [here](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow).

## Licensing
This library is under the license Apache 2.0. Consult the license file for more information.

## Compatibility
This library is compatible with Java 8+.
