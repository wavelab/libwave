### Generating a .deb for OpenCV 3.2.0 ###

Requirements:
debhelper
devscripts
build-essential

1) Get .tar.gz from source (https://github.com/opencv/opencv/releases/tag/3.2.0)

2) Extract the .tar.gz
`tar xf opencv-3.2.0.tar.gz`

3) Rename the .tar.gz
`mv opencv-3.2.0.tar.gz opencv_3.2.orig.tar.gz

4) Create a "debian/" directory inside the opencv-3.2.0 directory
`mkdir debian`

5) Inside the debian directory, add a changelog, compat, control, and rules file
`dch --create -v 3.2-0 --package opencv`
`echo "9" >> compat`
- my control file and rules file are a little more complex
    - inside the rules file, I place all the CMake flags
    - inside the control file, I place all the dependencies

6) Create your GPG key if you don't have one (I highly recommend also installing rng-tools)
`sudo apt-get install rng-tools`
`/etc/init.d/rng-tools start`
`gpg --gen-key`

7) Run debuild and fix any errors
`debuild -S -k<your-GPG-key>`

8) Ensure that you register your GPG key with UbuntuOne
 https://launchpad.net/~<username>/+editpgpkeys

8) Upload to your PPA
`dput ppa:<user>/<ppa> <package-name>_<version>_source.changes`
It will take a while to process
