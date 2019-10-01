# rb-rca5
This project is a solution to the task description in the file "Project Description.pdf"

## Requirements
This project is requires following things 

1. Gazebo 10
2. opencv 2
3. fuzzylite 6.0

## Installing Requirements

### Installing Gazebo
There are many ways to install Gazebo on. It has a docker file, which allow you to install it no matter which operation system you are usering. In the folder installer contains a installer script for Gazebo 10 with only work on linux. 
#### Mapping editor
As it appears from the "Project Description.pdf" the tools make it easier 
``` bash
sudo apt install openctm-tools pstoedit potrace
```

### Installing OpenCV 2
It is done by in Ubuntu by running follow command
``` bash
sudo apt install libopencv-dev
```

### Installing fuzzylite 6
At the time Ubuntu 18.04 package manager didn't contain fuzzylite 6 therefore it is necessary to compile from source. 
You can get a copy fuzzylit on this [github site](https://github.com/fuzzylite/fuzzylite).
When you have download follow these steps:
1. Unzip the folder.
2. Enter the unzip folder and navigate to inside the folder fuzzylite.
3. Run cmake. 
4. Run `sudo make install`
..* This makes a bin folder and add all the header to "/usr/local/includes" 
5. Move the files in the bin folder to "/bin"
6. Try running the command fuzzylite and fuzzylite-test

## Running Gazebo


In this project there are two files used to run Gazebo. The first file "gazebo_server.sh" launch the server part of gazebo. The second file "gazebo_client.sh" is launching the GUI and connecting to the server.

There are to different worlds with can be launch as follow 
``` bash
bash gazebo_server.sh smallworld.world
bash gazebo_server.sh bigworld.world
```
## Git

### initialization
Go to desired git folder at own pc (cd something/something)

clone path by $git clone url (verify by ls)
Go inside folder and check status by $git status
``` bash
git checkout Development
```

to merge own code and download new code(stuff only happens at own pc), use
``` bash
git pull
```
to upload own code to Git

``` bash
git commit -m
```

