# Model Predictive Controllers with Intelligent Optimizers (WIP 6/17/2022, Defense 10/10/2022)
## Updates
Upcoming project architecture modifications will integrate MATLAB to the workflow. MPC/MHE, modeling, simulation, 
and data analysis will be offloaded to MATLAB. The flow chart below outlines the overall intended structure.

![Runtime Diagram](/assets/block_diagram_runtime.png "Block Diagram at Runtime")

## Introduction
This repo is home to all of the AI-MPC algorithms proposed in my thesis. The LaTeX [manuscript](https://www.overleaf.com/read/fnqgjjqtmtzw) is available online.  
Project includes simulated control system examples for:
- Batch bioreactor
- Polymerization reactor
- Oscillating masses
- Inverted double pendulum

And physical control system examples for:
- Inverted single pendulum ([Quanser QUBE](https://www.quanser.com/products/qube-servo-2/))
- Dual rotor ([Quanser AERO](https://www.quanser.com/products/aero-2/))

## Requirements
- MATLAB R2021a
- Python 3.10
- Model Predictive Control Toolbox

## Usage
If you have the software pre-requisites, you can run the algorithm manually by executing main.m  

Otherwise, skip to the results section.  

### Using Pre-trained Models (recommended)
1. Navigate to the folder containing the project you're interested in
2. Run the MATLAB file, which will:
   1. Start Simulink
   2. Initialize the controllers
      1. MPC is built-in to MATLAB
      2. AI model must be loaded and imported to the simulation
   3. Run the simulation

### Training your own models
- WIP: Intended use will be something along the lines of:
  1. "git clone <this repo>"
  2. "python train.py --options" from command line
     1. Include information about:
        1. Dataset format
        2. Hyperparameters 
        3. Network type
        4. GPU
        5. Modules detailed in environment.yaml
           1. Include a requirements.txt for the venv users
        6. etc.
  3. Overwrite previous model.pth file
  4. Run the MATLAB scripts as usual

## Results

## Related Work
Key reference papers from the thesis 
