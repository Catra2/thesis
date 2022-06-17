# Model Predictive Controllers with Intelligent Optimizers (WIP 6/17/2022, Defense 10/10/2022)
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
There are two options:
- Train your own models from scratch
- Use provided pre-trained models

### Training your own models
Open the folder containing the project you're interested in and type:
    python train.py -h

## Related Work
Reference papers from the thesis 
