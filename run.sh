#!/bin/bash
echo -e \\033c
echo terminal reset

echo cleaning
rm *.class
rm route.txt

echo compiling
javac *.java

echo running
java PathPlanner hw4_world_and_obstacles_convex.txt hw4_start_goal.txt
