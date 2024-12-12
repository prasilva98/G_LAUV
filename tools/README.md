### Dockerfile 

This docker image contains pretty much everyhting needed to run the log the replayer and to continue its development. 

## Build 

Obviously, you should install docker beforehand. 
To build it the container:

```
docker build -t g-lauv . --progress=plain 
```

To run it with a GUI please make sure you install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). 
This, together with some extra launch arguments will allow your docker to access your GPU and produce a GUI when running Gazebo. 
To run the container: 

```
docker run -it --gpus all --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" g-lauv 
```

Inside the container you can follow the instructions to start a replayer. 

