cd snd_hda_macbookpro &&
sudo ./install.cirrus.driver.sh &&
cd ../snd-hda-codec-cs8409 &&
make clean &&
make &&
sudo make install
