#!/usr/bin/env bash

# luke@aidrivers.ai

:'
This script:
- uninstalls some bloat, enables extra repos for apt, installs lots of useful little things that make an Ubuntu installation complete - build essentials, codecs, spell-check dictionaries, vlc, advanced power management, common dependencies, Teamviewer.
- It downloads, compiles, installs and configures ROS and Autoware including all dependencies and a hack for dealing with environment conflicts. Note: This takes a *long* time.
- It installs Eclipse and clang-format for c/c++
- Downloads and installs the Anaconda scientific computing platform and creates sample enviornment for Tensorflow (GPU-enabled) with Spyder and another for Pytorch/Spyder (GPU enabled). These can be entered with "source activate tf" and "source activate pytorch" or through the Anaconda navigator. Note: these are included for pedagogical value and for quick testing - it is recommended to create new enviornments per application for actual development.
If unfamiliar with Anaconda, it is heavily recommended to read the docs before using: https://conda.io/docs/user-guide/tasks/manage-environments.html
Note: The Nvidia GPU driver must be enabled for these to work. This is best done through the Ubuntu Additional Drivers menu.

It is recommended to read through this script before running it so that you understand what it is doing, run without arguments as a normal unpriviliged user for further instruction.

'

printf "\n\nIt is strongly recommended to bring your base ubuntu installation up to date before proceeding, if you have not already done so, please run:\n\nsudo apt-get update && sudo apt-get dist-upgrade -y && sudo systemctl reboot\n\nPlease note that this script *must* be ran with normal user permssions, not as sudo, although you will be asked for the sudo password\n\nPlease note that this script will also take a very long time to run, use a lot of bandwidth and a lot of cpu time downloading and building various packages. It will require the sudo lock for a lot of the time so it is worth first temporarily extending the retention time of the sudo lock so that you don't have to watch the script and repeatedly enter the sudo password. You can do this by running:\n\nsudo visudo\n\nand then changing the line: \"Defaults      env_reset\" to: \"Defaults      env_reset,timestamp_timeout=600\"\nThis will make the sudo lock last for 10 hours. Remember to change it back afterwards!\nIf you wish to cancel this script for now to implement the above recommendations, press ctrl+c. To continue, press Enter\n"
read
cd ~
# get the sudo password
sudo su $USER

printf "Creating a directory to store logs: ~/install_logs\nPlease review afterwards to check for errors"
mkdir -p install_logs

echo "Getting rid of some of the bloat..."
sudo apt-get purge -y unity-webapps-common aisleriot gnome-mahjongg gnome-mines gnome-sudoku -y 1> ~/install_logs/apt.log 2> ~/install_logs/apt.error.log
sudo apt-get -y autoremove 1>> ~/install_logs/apt.log 2>> ~/install_logs/apt.error.log

echo "Installing codecs, spell-check dictionaries, extra repos, vlc, advanced power management"
sudo apt-get install ubuntu-restricted-extras vlc -y 1>> ~/install_logs/apt.log 2>> ~/install_logs/apt.error.log

# enable canonical partner repos.
sudo printf "\n#Canonical Partner Repos" >> /etc/apt/sources.list
sudo printf "deb http://archive.canonical.com/ubuntu xenial partner" >> /etc/apt/sources.list

# Install UK English Dictionary (for spellcheck in libreoffice, thunderbird)
# If you want the canadian version or australian version etc. subs. 'gb' for 'ca' or 'au'
sudo apt-get install libreoffice-l10n-en-gb libreoffice-help-en-gb thunderbird-locale-en-gb hunspell-en-gb hyphen-en-gb 1>> ~/install_logs/apt.log 2>> ~/install_logs/apt.error.log

# install and enable advanced power management - useful for laptops
sudo apt-get install tlp tlp-rdw -y 1>> ~/install_logs/apt.log 2>> ~/install_logs/apt.error.log
sudo tlp start 1>> ~/install_logs/apt.log 2>> ~/install_logs/apt.error.log

# Install Eclipse IDE and C/C++ development tools
echo 'Installing Eclipse c/c++ tools and clang-format'
sudo apt-get install -y qtdeclarative5-controls-plugin qml-module-qtquick-controls clang-format
sudo apt-get install -y eclipse-cdt

# https://repo.continuum.io/archive/
# It is recommended that users take the time to understand how conda works and how to use and share conda envs
printf 'Installing Anaconda python for scientific computing - please take 15 minutes or so to read the docs and understand what it is and how it works \nInternal notes: https://3.basecamp.com/4075579/buckets/9044477/messages/1395346583 \nDocs: https://docs.anaconda.com/'
wget https://repo.continuum.io/archive/Anaconda3-5.3.0-Linux-x86_64.sh
bash Anaconda3-5.3.0-Linux-x86_64.sh -bfp ~/anaconda3
rm Anaconda3-5.3.0-Linux-x86_64.sh
# hack to avoid clashes between anaconda and ROS
wget https://gist.githubusercontent.com/StefanFabian/17fa715e783cd2be6a32cd5bbb98acd9/raw/6982a55347a047f5c6baa9a69264550dde3d7c85/.anaconda_with_ros_wrapper.bash
echo 'source .anaconda_with_ros_wrapper.bash' >> ~/.bashrc
source ~/.bashrc 
conda update conda -y 1>> ~/install_logs/conda.log 2>> ~/install_logs/conda.error.log
conda update --all -y 1>> ~/install_logs/conda.log 2>> ~/install_logs/conda.error.log

#Spyder IDE can be launched with the appropriate environment by selecting the env from the drop down menu at the top of Anaconda Navigator. This can be pinned permanently to the Ubuntu side-bar by right clicking the icon

# create a tensorflow env
# Note: you will have to install and enable the NVIDIA gpu driver for this to work
# if you do not have an nvidia gpu you can install the non-gpu versions by changing 'tensorflow-gpu' to 'tensorflow'
echo 'Creating conda tensorflow environment (gpu version) called tf'
conda create --name tf -y && source activate tf && conda install tensorflow-gpu -y && conda install -c menpo opencv3 -y && conda install spyder -y 1>> ~/install_logs/conda.log 2>> ~/install_logs/conda.error.log

# create a pytorch env
# if you do not have an nvidia gpu you can install the non-gpu versions by changing 'pytorch-gpu' to 'pytorch'
echo 'Creating conda pytorch environment (gpu version) env called pytorch'
conda create --name pytorch -y && source activate pytorch && conda install -c anaconda pytorch-gpu -y && conda install -c menpo opencv3 -y && conda install spyder -y 1>> ~/install_logs/conda.log 2>> ~/install_logs/conda.error.log

# Install ROS
echo 'downloading, building and installing ROS'
sudo echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list 1>> ~/install_logs/ros.log 2>> ~/install_logs/ros.error.log
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 1>> ~/install_logs/ros.log 2>> ~/install_logs/ros.error.log
sudo apt-get update 1>> ~/install_logs/ros.log 2>> ~/install_logs/ros.error.log
sudo apt-get install ros-kinetic-desktop-full -y 1>> ~/install_logs/ros.log 2>> ~/install_logs/ros.error.log
sudo /usr/bin/rosdep init 1>> ~/install_logs/ros.log 2>> ~/install_logs/ros.error.log
/usr/bin/rosdep update 1>> ~/install_logs/ros.log 2>> ~/install_logs/ros.error.log
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y 1>> ~/install_logs/ros.log 2>> ~/install_logs/ros.error.log

# Install and build Autoware
echo 'downloading, building, configuring and installing Autoware'
source /opt/ros/kinetic/setup.bash
sudo apt-get install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin libmosquitto-dev gksu 1>> ~/install_logs/autoware.log 2>> ~/install_logs/autoware.error.log
git clone https://github.com/CPFL/Autoware.git --recurse-submodules 1>> ~/install_logs/autoware.log 2>> ~/install_logs/autoware.error.log
cd ~/Autoware/ros/src/
catkin_init_workspace 1>> ~/install_logs/autoware.log 2>> ~/install_logs/autoware.error.log
cd ~/Autoware/ros/
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO 1>> ~/install_logs/autoware.log 2>> ~/install_logs/autoware.error.log
./catkin_make_release 1>> ~/install_logs/autoware.log 2>> ~/install_logs/autoware.error.log


echo 'downloading and installing Teamviewer - if this step fails it is probably due to an expired TLS certificate, in which case, do it manually: https://download.teamviewer.com/download/linux/teamviewer_amd64.deb'
# Install dependencies...
sudo apt-get install -y qtdeclarative5-controls-plugin qml-module-qtquick-controls

# TLS Cert - the site is trusted by firefox but not wget by default, best practice is to download securely with the cert:
echo '-----BEGIN CERTIFICATE-----
MIIG2TCCBcGgAwIBAgIRAKoBuDHRTzsRSQg0MJaRBsEwDQYJKoZIhvcNAQELBQAw
gZAxCzAJBgNVBAYTAkdCMRswGQYDVQQIExJHcmVhdGVyIE1hbmNoZXN0ZXIxEDAO
BgNVBAcTB1NhbGZvcmQxGjAYBgNVBAoTEUNPTU9ETyBDQSBMaW1pdGVkMTYwNAYD
VQQDEy1DT01PRE8gUlNBIERvbWFpbiBWYWxpZGF0aW9uIFNlY3VyZSBTZXJ2ZXIg
Q0EwHhcNMTgwNTIyMDAwMDAwWhcNMjAwNTIxMjM1OTU5WjBdMSEwHwYDVQQLExhE
b21haW4gQ29udHJvbCBWYWxpZGF0ZWQxHTAbBgNVBAsTFFBvc2l0aXZlU1NMIFdp
bGRjYXJkMRkwFwYDVQQDDBAqLnRlYW12aWV3ZXIuY29tMIIBIjANBgkqhkiG9w0B
AQEFAAOCAQ8AMIIBCgKCAQEA8usbHNFBND+Nt5SVtFJT5fAhPglbzRrv0FzX4ne2
QwsM8U7to6qzN8KvKGfmBlQrBmFOPFZb27NyxwdRHXZc9SHnS92+/puQWNLLRXYG
MUwcn2fKLYcANcn9mvy42WMDlHk8HMHX8t07KbdGLYnsHFMMf+KrbfAo4IU3PPbI
Q6Mf8EeTjrsw/RBu3RzdM21YKtOrqJ+gXA2Qebb/jsUiPG3duEPFWNoSNuNk4e2O
ngL71mRzyOFxfAnv6NnCO2dNkmzNn/c6T+s4yt77x0J1rDY3pzTfW44iuCtdYkQp
Om2QqCRnxvFF6aQuJgeqbOul3Zj7F3YJomCyZKnwopg3VwIDAQABo4IDXjCCA1ow
HwYDVR0jBBgwFoAUkK9qOpRaC9iQ6hJWc99DtDoo2ucwHQYDVR0OBBYEFMstI4ni
IJ1zsK0LNlYb8RxcBycTMA4GA1UdDwEB/wQEAwIFoDAMBgNVHRMBAf8EAjAAMB0G
A1UdJQQWMBQGCCsGAQUFBwMBBggrBgEFBQcDAjBPBgNVHSAESDBGMDoGCysGAQQB
sjEBAgIHMCswKQYIKwYBBQUHAgEWHWh0dHBzOi8vc2VjdXJlLmNvbW9kby5jb20v
Q1BTMAgGBmeBDAECATBUBgNVHR8ETTBLMEmgR6BFhkNodHRwOi8vY3JsLmNvbW9k
b2NhLmNvbS9DT01PRE9SU0FEb21haW5WYWxpZGF0aW9uU2VjdXJlU2VydmVyQ0Eu
Y3JsMIGFBggrBgEFBQcBAQR5MHcwTwYIKwYBBQUHMAKGQ2h0dHA6Ly9jcnQuY29t
b2RvY2EuY29tL0NPTU9ET1JTQURvbWFpblZhbGlkYXRpb25TZWN1cmVTZXJ2ZXJD
QS5jcnQwJAYIKwYBBQUHMAGGGGh0dHA6Ly9vY3NwLmNvbW9kb2NhLmNvbTArBgNV
HREEJDAighAqLnRlYW12aWV3ZXIuY29tgg50ZWFtdmlld2VyLmNvbTCCAX0GCisG
AQQB1nkCBAIEggFtBIIBaQFnAHYA7ku9t3XOYLrhQmkfq+GeZqMPfl+wctiDAMR7
iXqo/csAAAFjh5KmRAAABAMARzBFAiEArLo3veObBMplstjU/8tJbUiDtMZjZP04
XEvmALNQ1nECIFfeJr7km8Jp9n8LH6kvH07rte+FBryt/LqvZZK//PYtAHUAXqdz
+d9WwOe1Nkh90EngMnqRmgyEoRIShBh1loFxRVgAAAFjh5KofAAABAMARjBEAiBT
CYQwohw6rNDBD9yjs7JXqLQoUXsMqvsJgI7qaWEA9QIgQKyQGn+J8fkWJlRP7zKa
bJoEUwNkEbcBy8VadjGvvLcAdgBvU3asMfAxGdiZAKRRFf93FRwR2QLBACkGjbII
mjfZEwAAAWOHkqY5AAAEAwBHMEUCIAJ1ygD/+OCPFL3LC7QVK8tTfHXUySrpNwco
qWpWCeK6AiEAkCepeJj+TZ+AU8VoX+m8m7Zl/ErtnuabBf5BrXjvlI8wDQYJKoZI
hvcNAQELBQADggEBAFcRXFeTgXCu6Ydbg2NdZLTQ2KJgDDcM45bSbzckmPQicSxP
+GKqHBZFP3Q/e24Pq+6cYuGlPtnK3ycMwjjCRTY4PjLt6jr9N8essxQvBJ4p5hbd
q0TG+4GcfAx0g5pWD6NYhx55p7UotpNdModY54lcWja7qFyHHhJPl6dK6SyKS1uA
mDnWwbGDnHH38qbPlMUZxqHPkBbyL4u5F7jL/TEOJJ1rzRgLEa23yM8QwMtHy95M
JKbu6ou5V+VjyVtId8sXA1BAiFVI9sAA2ZPv3Cw5H4jqOAkD7oDIN43K/PKhsCGM
2tzhs4pUKyBS/mQe8eC9yOKM1yKEILvsJmEKShU=
-----END CERTIFICATE-----' > teamviewer.tls

wget --ca-certificate=teamviewer.tls https://download.teamviewer.com/download/linux/teamviewer_amd64.deb
rm teamviewer.tls
sudo dpkg -i teamviewer_amd64.deb


# To change username from 'user' to 'phil' (don't do this whilst logged in, use a TTL shell as root)
# sudo usermod -l phil user
# sudo usermod -d /home/newHomeDir -m newUsername
# to change password, simply run, passwd
# To change the hostname, edit the following:
# /etc/hostname
# /etc/hosts
#then reboot
