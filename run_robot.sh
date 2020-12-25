#!/bin/bash
usage(){ 
	echo "Invalid option, Usage: $0 [-v|-r]"
}

if (($# == 0)); then
	usage
	exit 2
fi

while getopts "vr?h" opt; do
  case $opt in
	  v)
		  echo "Calling vision script..."
		  sudo /home/pi/.virtualenvs/cv_bot/bin/python vision.py
		  ;;
	  r)
		  echo "Starting Robot..."
		  sudo /home/pi/.virtualenvs/cv_bot/bin/python robot.py
		  ;;
	  h|?)
		  usage
		  exit 1
		  ;;
  esac
done
