#!/usr/bin/python
import os
import sys
import yaml
import shutil

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
        os.chdir(submission_path)
        command = 'python final.py '+color
        print(command)
        os.system(command)

    elif mat:
        print("Running Team "+team+" as "+color+" in MATLAB")
        os.chdir(submission_path)
        command = "matlab nodisplay -nosplash -nodesktop -r \"addpath('../Core'); final('"+color+"'); rosshutdown; exit;\""
        print(command)
        os.system(command)

    else:
        print("No MATLAB or Python code submitted")
