#!/usr/bin/python
import os
import sys
import yaml
import shutil

def matlab_kernel(color,command):
    idx = 0 if color == 'blue' else 1
    command = " tmux send-keys -t ta_view:kernels."+str(idx)+" '"+command+"' && tmux send-keys -t ta_view:kernels."+str(idx)+" Enter"
    os.system(command)

def python_kernel(color,command):
    idx = 2 + (0 if color == 'blue' else 1)
    command = " tmux send-keys -t ta_view:kernels."+str(idx)+" '"+command+"' && tmux send-keys -t ta_view:kernels."+str(idx)+" Enter"
    os.system(command)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Error: expected team number and color")
        exit()

    team = str(sys.argv[1])
    color = str(sys.argv[2])

    target_folder = os.path.expanduser('~/GoogleDrive/.shared/MEAM 520 F20/FinalProjectSubmissions')

    submission_path = os.path.join(target_folder, "Team"+team)

    # try:
    py = [f for f in os.listdir(submission_path) if f.endswith('.py')]
    mat = [f for f in os.listdir(submission_path) if f.endswith('.m')]
    if py and mat:
        raise Exception('ERROR: Team '+team +' submitted in 2 languages, cannot run')
    if py:
        print("Running Team "+team+" as "+color+" in Python")

        python_kernel(color, 'cd "'+submission_path+'" ' )
        python_kernel(color, 'clear' )
        python_kernel(color, 'python final.py '+color )

    elif mat:
        print("Running Team "+team+" as "+color+" in MATLAB")

        # WARNING: YOU MUST BE RUNNING MATLAB IN THE BACKGROUND FOR BOTH TEAMS
        # USING ta.sh FOR THIS TO WORK

        matlab_kernel(color, 'cd("'+submission_path+'")' )
        matlab_kernel(color, 'clear; clc' )
        matlab_kernel(color, 'addpath("../Core")' )
        matlab_kernel(color, 'final("'+color+'")' )
        matlab_kernel(color, 'rosshutdown' )

    else:
        print("No MATLAB or Python code submitted")
