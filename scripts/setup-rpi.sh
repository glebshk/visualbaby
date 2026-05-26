echo "Installing pigpio library..."
git clone https://github.com/joan2937/pigpio.git
cd pigpio
make
sudo make install

echo "Enabling pigpio daemon..."
sudo pigpiod