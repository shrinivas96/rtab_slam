#!/bin/sh

# A script that creates a directory given the 
# absolute path of where the directory needs to be placed

# create the directory
mkdir -p ${1}

# print the name of the directory. when used with roslaunch, 
# this can return a value and set the value of a rosparameter.
# echo ${1} | awk -F / '{print $NF}' | tr -d "\n"
# echo $dir_name | tr -d "\n"

# Optionally to return the success or faliure of mkdir command
# Credits: https://stackoverflow.com/a/16633130/6609148
if [ $? -ne 0 ] ; then
    echo "fatal" | tr -d "\n"
else
    echo "success" | tr -d "\n"
fi