#rm -rf darknet

echo  "Enter the full path to the YCB-Dataset folder (without the name of the folder)"
echo  "It should be something like: /home/username/Documents/"
read Res

cd $Res
git clone https://github.com/AlexeyAB/darknet.git
cd -
if [ "$1" = "rtx" ]; then
	cp ./RTX/Makefile $Res/darknet/Makefile
elif [ "$1" = "gtx" ]; then
	cp ./GTX_TITAN/Makefile $Res/darknet/Makefile
else 
	echo "Error, please try again"
	exit 1
fi
cd $Res/darknet
sudo make clean
sudo make

cd -
cp -r ../darknet_config/*.* $Res/darknet
