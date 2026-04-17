Imitation Learning of an LTV-MPC for Real-Time Lane Keeping and Dynamic Obstacle Avoidance
This repository contains the implementation of an Autonomous Driving system that combines classical Control Theory with Deep Learning. The project focuses on Dynamic Obstacle Avoidance and Lane Keeping under critical friction conditions (\mu = 0.5, simulating rain/snow).
We compare a Linear Time-Varying Model Predictive Control (LTV-MPC) — our mathematical "expert" — with a Neural Network (NN) trained via Behavioral Cloning (Imitation Learning) to act as a computationally efficient surrogate.

🚀 Key Features
• LTV-MPC Expert: A robust controller that optimizes steering and longitudinal acceleration while respecting safety and physical constraints.
• Neural Surrogate: A Deep Neural Network that replicates the MPC's behavior with significantly lower computational overhead.
• Low-Grip Scenarios: Rigorous testing on slippery surfaces (\mu = 0.5) to evaluate safety and generalization.
• 3D MATLAB Simulation: A custom-built 3D environment for real-time visualization of vehicle dynamics and obstacle evasion.

🛠️ Installation & Usage
Prerequisites
• MATLAB (R2021b or later recommended)
• Control System Toolbox
• Deep Learning Toolbox
• Optimization Toolbox

Running the simulation
1. Clone the repository
2. Open MATLAB and navigate to the project folder.
3. Run the main script: Test_MPC_vs_RN

📂 Repository Structure
• Test_MPC_vs_RN.m: Main entry point for the simulation and comparison.
• TrainedNet.mat: Pre-trained neural network weights.
• MPC_Select_LTV.m: LTV-MPC controller implementation.
• Functions/: Helper functions for 3D rendering and vehicle dynamics.

👥 The Team
• Dario Barbato
• Carmine Maisto
• Andrea Colapinto
Supervisor: Prof. Luigi Glielmo
