echo  "Which install do you want?"
echo  "1. CPU Only (low performance)"
echo  "2. GPU (Required a Nvidia GPU and an installed driver)"
read Res

#INSTALL DOCKER
sudo apt-get install curl
docker 2> /dev/null || curl https://get.docker.com | sh && sudo systemctl --now enable docker

if [ "$Res" = "1" ]; then
	sudo apt-get update
	sudo systemctl restart docker
elif [ "$Res" = "2" ]; then
	#INSTALL DOCKER-NVIDIA TOOLKIT
	#Source: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker
	distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
	   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
	   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

	sudo apt-get update
	sudo apt-get install -y nvidia-docker2
	sudo systemctl restart docker
	#sudo docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi | grep "CUDA Version: 11"
	# if [ $? = "" ]; then
	# 	echo -e "\n \e[42m Issue installing Nvidia-Docker, please see https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker \e[0m \n"; 
	# 	exit 1
	# fi
else 
	echo "Invalid input, please try again"
	exit 1
fi

#INSTALL DOCKER-COMPOSE
sudo apt-get remove docker-compose
COMPOSE_VERSION=$(wget https://api.github.com/repos/docker/compose/releases/latest -O - | grep 'tag_name' | cut -d\" -f4)
sudo wget https://github.com/docker/compose/releases/download/${COMPOSE_VERSION}/docker-compose-`uname -s`-`uname -m` -O /usr/local/bin/docker-compose
sudo chmod 755 /usr/local/bin/docker-compose

#PULL IMAGES AND INSTALL DEPENDENCIES (3-4GB space)
sudo sh pull-images.sh    

