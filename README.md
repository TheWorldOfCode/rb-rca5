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
You can either download a zip file or clone the project with `git clone`
When you have download follow these steps:
1. Unzip the folder. Enter the unzip folder (If clone enter the clone folder)
2. Navigate to inside the folder fuzzylite.
3. Run cmake `cmake . -Bbuild`. 
4. Go into the build folder and run `sudo make install`
..* This makes a bin folder and copy all the header to "/usr/local/includes" 
5. copy from the local bin folder fuzzylite to "/usr/local/bin" 
`sudo cp ./bin/fuzzylite /usr/local/bin`
6. Try running fuzzylite

If you tried to run the command `fuzzylite` before coping, you would have discovered that it won't run. This is because were are somthing wrong with the program (located in "/usr/local/bin"), to  solve this we copy a working copy of the program to the folder. 
## Running Gazebo
In this project there are two files used to run Gazebo. The first file "gazebo_server.sh" launch the server part of gazebo. The second file "gazebo_client.sh" is launching the GUI and connecting to the server.

There are to different worlds with can be launch as follow 
``` bash
bash gazebo_server.sh smallworld.world
bash gazebo_server.sh bigworld.world
```
When you have launch the server, open a new terminal and navigation to the project (or press Ctrl-Shift-N), because the current is lock and will show the status of the server.  
Now you can launch the client, this is not necessay i order to run you own code. This is done by running the following command 
``` bash
bash gazebo_client.sh & 
```
The & means you would like to run the client as a background process (this means the terminal doesn't lock). 

## Compile the code with CMake
In order to compile navigate to the code folder, in this project is it called robot_control. Heer will you find the CMakeLists.txt file which is used by the cmake to setup your make file. The make file is used to compile the code. 
In order to make the make file run following command:
``` bash 
cmake . -Bbuild
```
The dot means it shall look for the CMakeLists.txt in the current directory and the option `-Bbuild` means that it should build the necessay file in the directory "build/" (If the directory is create, it will created it). 

Then you have to navigate into the build directory. The you can run the following command in order to compile: 
``` bash 
make -j4
```
The option `-j4` is not necessacy but tell the make program it shall used 4 threads. 

Now you have compile you program. Or you can run the script build.sh, which create the build folder and compile the programs, and creating test folders.

### Problems 
You should beware of the cmake used absolute path, this means if you move the build folder it vil still look for the source file and header the same place as before the move. So if you will move the project remove the build folder (`rm -r build/`) and run cmake again. 

If the compile are given you problems try to run `make clean`, this remove the compile object files. Then try to compile again.  If this doesn't work the remove the build folder and use cmake and compile again. 

## Git
This is a short introduction to the used of git. 
### initialization
Navigate to the folder you have to copy the repository to. Then in order to clone the github respository get the url from a browser. 
When you have the run the command 
``` bash
git clone url
```
where the url is the one you get from the github site. 
Now you have clone the remove repository to your computer. In order to work with the repository enter the download folder. 

### Status 
I order to see the status of the current respository you can run the command `git status`. This tell you all you wants to own about the respository. 

### Adding file
If you create a file inside the repository folder doesn't mening that git track the file. I order to track the file you have to rune the command `git add filename` where filename is the name of the file you wants to track. If a file is not track you can't uploadet to the remove respository


### Remove files
If you wants to remove a track file (this vil not delete on the local machine) the run the command `git rm filename`, where filename is the name of the file you no longer would like to track .

### Update local repository from remote
This can be done with two difference commands. 

The first command is `git fetch` this get the change from the remote repository by it doesn't update the local this can be do with the command `git merge`

The second command is `git pull`  this fetchs the change and merge them with the local repository

### Update remote repository
This is done with the command `git push`, but in order to push you changes you have to commit the first this is done with the command `git commit -m 'This is my commit'`. The option '-m' means that you write the message directly.

### Working on a difference branch 
When you clone a repository you will make change to the master branch, this can be change like this `git checkout branchname` where branchname is the name of the branch you have to changes. Now you changes are upload to the this branch instead of the master. 



