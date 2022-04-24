#! /bin/bash

set -e

source /opt/ros/noetic/setup.bash
source ../../devel/setup.bash
roscore &

cd /
# see https://github.com/Jupyter-contrib/jupyter_nbextensions_configurator/issues/72
jupyter nbextensions_configurator enable --user
jupyter nbextension enable --py --sys-prefix jupyros
jupyter notebook --allow-root --ip 0.0.0.0
