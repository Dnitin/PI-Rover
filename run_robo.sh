#!/bin/bash


run_robot() {
	sudo /home/pi/.virtualenvs/cv_bot/bin/python robot.py
}
run_vision(){
	sudo /home/pi/.virtualenvs/cv_bot/bin/python vision.py 
}

while getopts "v:r" opt; do
	case $opt in
		r) run_robot ;; 
		v) run_vision ;; 
		*) run_robot ;;

	esac
done

