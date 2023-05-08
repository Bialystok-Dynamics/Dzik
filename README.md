# Dzik
## Docker
### Docker installation
Please follow steps described in [installation guide](https://docs.docker.com/engine/install/ubuntu/) as well as in [post installation guide](https://docs.docker.com/engine/install/linux-postinstall/)
### Gazebo installation
`sudo apt update && sudo apt install gazebo11`
### How to run
Currently you can only run simulation inside of docker image with gazebo client on local side. Rviz is not supported yet.
In order to launch dzik simulation simply execute the script:
```sh
./docker_run.sh
```
And in another terminal:
```sh
gzclient
```
Then you should see the view of the simulation.
### TODO
- push the image to docker registry, so it won't be necessary to built it for every user
- make rviz usable
- currently image is quite big (~3GB) mainly due to gazebo overhead, make it divide an image on simulation part and "core" part
- make such changes, so workspace won't be built for every run of container
