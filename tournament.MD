# Final Tournament Instructions for TAs

## Installation

First install `tmux`:
```
sudo apt install tmux
```

Then install the [google drive client](https://github.com/astrada/google-drive-ocamlfuse):
```
sudo add-apt-repository ppa:alessandro-strada/ppa
sudo apt-get update
sudo apt-get install google-drive-ocamlfuse
```
Start the client with
```
google-drive-ocamlfuse
```
which will open a browser window to log into your SEAS Google account. Then, create the mount point:
```
mkdir ~/GoogleDrive
```
and connect, as
```
google-drive-ocamlfuse ~/GoogleDrive
```
Now your Google Drive folders will be accessible from your computer. The submission folder should automatically be located at `~/GoogleDrive/.shared/MEAM 520 F20/FinalProjectSubmissions/`. To forcibly unmount your Google Drive, run the command

```
sudo umount -f ~/GoogleDrive
```

To unmount the Google Drive folder run:
```
sudo umount -f ~/GoogleDrive
```
## Running Tournament

Below are the steps to run each round. Here is the [tournament spreadsheet](https://docs.google.com/spreadsheets/d/1fW7A1DUgbDhKYgeo7QxdiVG7dADTGSHZWH_2BnEvoRM/edit?usp=sharing).

 1. Randomize the blocks with
 ```
 roscd al5d_gazebo/scripts && python block_randomizer.py
 ```

 2. Launch the simulator  and ensure that all objects and robots initialized correctly before proceeding.
 ```
 roslaunch al5d_gazebo final.launch
 ```

 3. Launch the tmux session with MATLAB/Python kernels where student code will be run, so we can have MATLAB already launched and so we can see the output in real time. There will be four shells, for [MATLAB Python] x [Red Blue], but only two will be used at a time. *You may not need to relaunch this every time but you should ensure the previous teams' code exited properly and there is an empty prompt in all four terminals before proceeding.*
```
roscd al5d_gazebo/scripts && ./ta.sh
```
4. Launch the student code. Replace 1 and 2 with the appropriate team numbers for the match. This script will write to the tmux sessions and then exit immediately. Wait until both team's tmux panes say `"Waiting for starting gun..."` before proceeding.
 ```
 roslaunch al5d_gazebo match.launch red:=1 blue:=2
 ```
5. Launch the referee to start the match. This will fire the starting gun. Switch back to tmux to check that both teams printed out `Go!` and are running.
  ```
  roslaunch al5d_gazebo referee.launch
  ```
6. Wait for the match to complete. After 60 seconds, Gazebo physics will be paused, the kernels will be killed, and the score for each team will print out on the `referee.launch` command line.
7. Enter the score on the  [tournament spreadsheet](https://docs.google.com/spreadsheets/d/1fW7A1DUgbDhKYgeo7QxdiVG7dADTGSHZWH_2BnEvoRM/edit?usp=sharing).
8. Close `final.launch` to prep for next match.


*Note: it may take a few minutes for student code to update in the Google Drive folders when they make changes (for during practice rounds). It may also be necessary to unmount and remount the Google Drive to avoid syncing issues.*
