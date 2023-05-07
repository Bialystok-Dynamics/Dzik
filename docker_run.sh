#!/usr/bin/sh
docker build -t argo:dzik .
docker run --rm --network=host -it --expose 11345 -v ${PWD}/src:${PWD}/src -v ${PWD}/run_sim.sh:${PWD}/run_sim.sh --workdir ${PWD} argo:dzik "./run_sim.sh"
