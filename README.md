# Model Predictive Controllers with Intelligent Optimizers (WIP 6/17/2022, Defense 10/10/2022)
## Updates
Upcoming project architecture modifications will integrate MATLAB to the workflow. MPC/MHE, modeling, simulation, and data analysis will be offloaded to MATLAB. The flow chart below outlines the overall intended structure.

[Controller Training Block Diagram](/assets/block_diagram.png)

## Introduction
This repo is home to all of the AI-MPC algorithms proposed in my thesis. The LaTeX [manuscript](https://www.overleaf.com/read/fnqgjjqtmtzw) is available online.
This project includes simulated control examples for
- Batch bioreactor
- Polymerization reactor
- Oscillating masses
- Inverted double pendulum

And applied control examples for 
- Inverted single pendulum ([Quanser QUBE](https://www.quanser.com/products/qube-servo-2/))
- Dual rotor ([Quanser AERO](https://www.quanser.com/products/aero-2/))

## Requirements
- PyTorch (version)
- CUDA (version)
- do-mpc (version)
- Numpy (version)

## Usage
Clone this repo

    git clone https://github.com/Catra2/thesis

It is recommended to use [Anaconda](https://www.anaconda.com/products/distribution) package manager, in which case you can run: 

    conda env create -f environment.yml
    conda activate thesis

There are two options:
- Train your own models from scratch
- Use provided pre-trained models

### Using Pre-trained Models
1. Navigate to the folder containing the project you're interested in
2. Modify 

### Training your own models
1. Navigate to the folder containing the project you're interested in
2. Modify the local config.py file manually and alter the paths related to models and checkpoints to your preference
3. To see a full list of available options, run:

    python train.py -h

## Related Work
Reference papers from the thesis 
