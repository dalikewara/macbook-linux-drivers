cd facetimehd-firmware &&
make clean &&
make &&
sudo make install &&
cd .. &&
cd bcwc_pcie &&
make clean &&
make &&
sudo make install &&
sudo depmod &&
sudo modprobe -r bdc_pci &&
sudo modprobe facetimehd
