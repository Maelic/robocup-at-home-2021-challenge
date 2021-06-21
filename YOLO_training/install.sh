echo  "Which GPU do you have ? (type 1 or 2)"
echo  "1. Nvidia RTX (Turing Architecture)"
echo  "2. Nvidia GTX or TITAN (Pascal Architecture)"
read Res

if [ "$Res" = "1" ]; then
	model="rtx"
elif [ "$Res" = "2" ]; then
	model="gtx"
else 
	echo "Invalid input, please try again"
	exit 1
fi

mkdir -p install/ && cd install/
sudo apt-get install python-pip
pip install gdown

################################################
###      Install CUDNN if not existing       ###
################################################
echo -e "\n \e[43m Check CUDNN version ... \e[0m \n"; 
gdown https://drive.google.com/uc?id=1mQuzRPWnNKnRtsQDoAw7X8X15GMOfd9o
sudo dpkg -i libcudnn8-samples_8.1.1.33-1+cuda11.2_amd64.deb
sudo ldconfig
cp -r /usr/src/cudnn_samples_v8/ .
cd  ./cudnn_samples_v8/mnistCUDNN
make clean && make
./mnistCUDNN | grep "Test passed!"
if [ $? = 0 ]; then
       cd ../.. 
       echo -e "\n \e[42m CUDNN 8.1 already installed, skipping ... \e[0m \n"; 
else
       cd ../.. 
       echo -e "\n \e[43m Installing CUDNN 8.1 ... \e[0m \n"; 
       gdown https://drive.google.com/uc?id=145G84LZnHNi6QsCGX_7OAKrunSoKoKIa
       gdown https://drive.google.com/uc?id=1QjexX5lXDN4Qjf8hOfbB0iV1pjC0Yp9j
       sudo dpkg -i libcudnn8_8.1.1.33-1+cuda11.2_amd64.deb
       sudo dpkg -i libcudnn8-dev_8.1.1.33-1+cuda11.2_amd64.deb
       sudo ldconfig
       echo -e "\n \e[42m Done. \e[0m \n"; 
fi
cd ..

################################################
###         Download and Install Yolo        ###
################################################
echo -e "\n \e[43m Download and Install Yolo ... \e[0m \n"; 
cd yolo
bash install.sh $model
cd ..
rm -rf install/
echo -e "\n \e[42m Done. \e[0m \n";

echo -e "\n \e[42mInstall complete, well done !!! \e[0m \n";
