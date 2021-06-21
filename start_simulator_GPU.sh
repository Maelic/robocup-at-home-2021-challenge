#DISPLAY=:0 xhost si:localuser:root
#docker-compose -f docker-compose.nvidia.yml up

gnome-terminal -e "bash -c 'DISPLAY=:0 xhost si:localuser:root; docker-compose -f docker-compose.nvidia.yml up;exec $SHELL'"&
sleep 30
xdg-open http://localhost:3000
xdg-open http://localhost:3001
