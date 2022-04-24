#! /bin/bash

cd /
# see https://github.com/Jupyter-contrib/jupyter_nbextensions_configurator/issues/72
jupyter nbextensions_configurator enable --user
jupyter nbextension enable --py --sys-prefix jupyros
jupyter notebook --allow-root --ip 0.0.0.0
