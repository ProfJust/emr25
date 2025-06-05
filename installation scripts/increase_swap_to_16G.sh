# https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-22-04
free -h
df -h
sudo swapoff /swapfile     # alten Swap ausschalten
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo swapon --show
sudo cp /etc/fstab /etc/fstab.bak
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab