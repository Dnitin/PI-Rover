#!/bin/bash
usage(){ 
	echo "Invalid option, Usage: $0 [-v|-r|-l|-e|[-?|-h]]"
}

while getopts "vtle?h" opt; do
  case $opt in
	  v)
		  echo "Calling vision script..."
		  sudo /home/pi/.virtualenvs/cv_bot/bin/python vision.py
		  ;;
	  t)
		  echo "Starting Robot in test mode..."
		  sudo /home/pi/.virtualenvs/cv_bot/bin/python robot.py tst
		  ;;
	  l)
		  echo "Starting Robot in line follower mode..."
		  sudo /home/pi/.virtualenvs/cv_bot/bin/python robot.py line_follower
		  ;;
	  e)
		  echo "Starting Robot in Grid explorer mode..."
		  sudo /home/pi/.virtualenvs/cv_bot/bin/python robot.py explore_grid
		  ;;
	  h|?)
		  usage
		  exit 1
		  ;;
  esac
done
