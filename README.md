1. get the groundgrid code:
```sh
git clone https://github.com/dcmlr/groundgrid.git
```
2. build the container:
```sh
docker compose up -d --build
```
3. run the container:
```sh
xhost +
docker compose exec groundgrid bash
```

4. build the groundgrid code:
```sh
# make sure you are in workspace directory /home/workspace
catkin build
```
5. run the groundgrid node:
- **Nicolas version**
```sh
. devel/setup.bash
# no need to run roscore, it will be started automatically
roslaunch groundgrid KITTIPlayback.launch directory:=/path/to/the/SemanticKITTI/dataset sequence:=0
# or
roslaunch groundgrid KITTIEvaluate.launch directory:=/path/to/the/SemanticKITTI/dataset sequence:=0
```
- **Boris version**
```sh
roscore
rostopic list
rosbag play -l bags/2025-03-17-13-37-54.bag
roslaunch groundgrid loxo.launch
python3 tools/topic_splitter.py
rviz # then open rviz config file
```