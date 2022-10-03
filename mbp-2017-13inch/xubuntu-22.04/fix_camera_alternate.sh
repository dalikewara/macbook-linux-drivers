cd facetimehd-firmware &&
make clean &&
make &&
sudo make install &&
cd ../facetime/bcwc_pcie &&
make clean &&
make &&
sudo depmod &&
sudo modprobe facetimehd &&
sudo su &&
echo facetimehd >> /etc/modules
