
#!/bin/sh

# SET UP TMUX SESSION FOR TA DURING COMPETITION

session="ta_view"
window="kernels"

tmux kill-session

# START SESSION
tmux new-session -d -s $session

tmux rename-window -t 0 $window
tmux split-window -h
tmux split-window -v -t 0
tmux split-window -v -t 2

# tmux select-layout tiled

# RED MATLAB
tmux send-keys -t $session:$window.0 'clear && matlab nodisplay -nosplash -nodesktop'  C-m

# BLUE MATLAB
tmux send-keys -t $session:$window.1 'clear && matlab nodisplay -nosplash -nodesktop' C-m

# RED PYTHON
tmux send-keys -t $session:$window.2 'clear'
tmux send-keys -t $session:$window.2 Enter

# BLUE PYTHON
tmux send-keys -t $session:$window.3 'clear'
tmux send-keys -t $session:$window.3 Enter

tmux -2 attach-session -d

tmux select-layout tiled


# tmux send-keys -t 0 'red_matlab'
# tmux send-keys -t 1 'red_matlab' 'echo blue' Enter # launch MATLAB


# tmux rename-window -t 0 'blue_matlab' # create shell
# tmux send-keys -t 'blue_matlab' 'matlab nodisplay -nosplash -nodesktop' C-m # launch MATLAB
#
# tmux new-window -t $SESSION:2 -n 'red_matlab'
# tmux send-keys -t 'red_matlab' 'matlab nodisplay -nosplash -nodesktop' C-m # launch MATLAB
