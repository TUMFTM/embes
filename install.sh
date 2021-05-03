#!/bin/bash

# clone embedded energy strategy repository and initialize submodule
git clone https://gitlab.lrz.de/roborace/modules/mod_energy_strategy.git
cd mod_energy_strategy
git submodule update --recursive --init
cd ..

# install acados modeling language
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
mkdir -p build
cd build
cmake ..
make install
cd ..
cd ..

# create virtual environment and activate it
python3 -m venv venv_es
source venv_es/bin/activate
pip install --upgrade pip

# install acados python template
cd acados
pip3 install interfaces/acados_template
wget https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux
mkdir -p bin/
cp t_renderer-v0.0.34-linux bin/t_renderer
chmod +x bin/t_renderer
cd ..

# export environment variables
export ACADOS_SOURCE_DIR=home/$USER/emb_es_root/acados
export LD_LIBRARY_PATH=home/$USER/emb_es_root/acados/lib:$LD_LIBRARY_PATH

# go to energy strategy code
cd mod_energy_strategy
pip install -r requirements.txt
