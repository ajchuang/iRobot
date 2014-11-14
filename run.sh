#!/bin/bash
echo cleaning
rm *.class

echo compiling
javac *.java

echo running
java PathPlanner hw4_world_and_obstacles_convex.txt hw4_start_goal.txt
