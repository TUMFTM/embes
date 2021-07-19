.. _refInstallation:

Installation
============

The code was tested using Ubuntu 20.04 and Python 3.7/3.8. The software-packages

.. code-block:: bash

    sudo apt-get install python3-dev
    sudo apt-get install build-essential
    sudo apt-get install cmake

must be available.

Quick-start
-----------

The impatient user can do the following three steps to install the code, all necessary dependencies and run the main
script of the energy strategy afterwards:

    1. In `/home/$USER/` create a folder `emb_es_root`

    .. code-block:: bash

        mkdir /home/$USER/emb_es_root

    2. Go to the folder `emb_es_root`, clone the repository, and change permissions of the install script `chmod +x install.sh` and run the
    script `./install.sh`

    .. code-block:: bash

        cd /home/$USER/emb_es_root
        git clone https://github.com/TUMFTM/embes
        cd embes
        chmod +x install.sh
        ./install.sh

    3. To run the main script of the energy strategy, run

    .. code-block:: bash

        cd /home/$USER/emb_es_root
        source venv_es/bin/activate
        cd embes
        export ACADOS_SOURCE_DIR=/home/$USER/emb_es_root/acados
        export LD_LIBRARY_PATH=/home/$USER/emb_es_root/acados/lib:$LD_LIBRARY_PATH
        python3 main_emb_es.py

    Your terminal should display the following information:

    .. image:: install_success.png
       :height: 100
       :align: center

Detailed description
--------------------

Do the following three steps, to check out the necessary code to run the energy strategy. We wish to achieve the
following structure:

    ::

        /home/$USER/emb_es_root
        ├── acados
        └── embes
            ├── external
                └── global_racetrajectory_optimization (submodule of *embedded_energy_strategy*)
            ├── requirements.txt
            └── main_emb_es.py

    1. Check out the repository `embes <https://github.com/TUMFTM/embes>`_
    and initialize its submodules to receive the necessary powertrain definitions:

    .. code-block:: bash

        git clone https://github.com/TUMFTM/embes
        cd embes
        git submodule update --recursive --init

    2. Download and install the `acados template for python <https://docs.acados.org/installation/index.html>`_
    into `/home/$USER/emb_es_root`,
    since we need it as optimization modeling language. The first lines in the following code fragment to install the
    template are copied from the acados manual:

    .. code-block:: bash

        cd /home/$USER/emb_es_root
        git clone https://github.com/acados/acados.git
        cd acados
        git submodule update --recursive --init

        mkdir -p build
        cd build
        cmake ..
        # add more optional arguments e.g. -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> above
        make install

    3. Set up a virtual environment using `virtualenv` and install the requirements given in `requirements.txt`.
    Insert a name of your choice into `<your-venv-name>`.

    .. code-block:: bash

        cd /home/$USER/emb_es_root
        python3 -m venv <your-venv-name>

    Now, activate the virtual environment and upgrade pip

    .. code-block:: bash

        source <your-venv-name>/bin/activate
        pip install --upgrade pip

    and install the `embes/requirements.txt` inside

    .. code-block:: bash

        pip3 install /home/$USER/emb_es_root/acados/interfaces/acados_template
        wget https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux
        mkdir -p /home/$USER/emb_es_root/acados/bin/
        cp t_renderer-v0.0.34-linux /home/$USER/emb_es_root/acados/bin/t_renderer
        chmod +x /home/$USER/emb_es_root/acados/bin/t_renderer
        cd /home/$USER/emb_es_root/embes
        pip install -r requirements.txt

Now you should be able to run the main script starting the energy strategy algorithm within the virtual environment
by exporting the library paths to your acados installation:

.. code-block:: bash

        export ACADOS_SOURCE_DIR=/home/$USER/emb_es_root/acados
        export LD_LIBRARY_PATH=/home/$USER/emb_es_root/acados/lib:$LD_LIBRARY_PATH

        cd /home/$USER/emb_es_root/embes
        python3 main_emb_es.py

To work with the code
---------------------
To work with the energy-strategy optimization, have a look into the :ref:`refExamples`-section of this
documentation.
