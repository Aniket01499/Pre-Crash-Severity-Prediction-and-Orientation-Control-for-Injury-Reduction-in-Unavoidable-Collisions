# Pre-Crash Severity Prediction and Orientation Control for Injury Reduction

**Author:** Aniket Kakde

## 📌 Overview
Traditional active safety systems prioritize binary collision avoidance. However, they often fail to address scenarios where a collision is physically unavoidable. 

This repository contains the MATLAB codebase for a severity-aware control architecture designed to optimize vehicle impact configuration during unavoidable side collisions. By actively controlling the vehicle's pre-crash orientation, the system shifts the impact away from vulnerable areas and toward safer, energy-absorbing structural zones.

> **Intellectual Property Notice:** The concepts, algorithms, and code contained within this repository represent original research and are the exclusive Intellectual Property of the author.

---

## 🚀 The Core Problem
When a lateral collision cannot be avoided, relying solely on longitudinal braking is insufficient. The vehicle's structural integrity varies significantly along its profile. This research bridges a critical gap by shifting the objective from "avoidance at all costs" to "injury mitigation through optimized impact geometry".

---

## ⚙️ Methodology & Architecture
The unique contribution of this project is a framework that synthesizes physical and biomechanical variables into a real-time optimization objective. 

* **Biomechanical Risk Synthesis:** Mathematical formulation of injury risk factors based on impact location and kinetic energy transfer.
* **Real-Time Optimization Objective:** A custom MATLAB algorithm designed to calculate the optimal vehicle orientation required to minimize predicted severity.
* **Orientation Control Logic:** Trajectory and yaw calculation to steer the vehicle toward the optimized impact configuration.

---

## 💻 Technologies & Files
This project was developed entirely in **MATLAB**, focusing on mathematical modeling, algorithmic logic, and numerical simulation. 

* `main_simulation.m`: The primary simulation script orchestrating the pre-crash optimization logic.
* `Scenario_validation.m`: Validation script to test the control architecture against defined crash scenarios.

<img width="916" height="876" alt="6 6" src="https://github.com/user-attachments/assets/a6b1c476-c8a9-428b-a9c2-8dd6d6565306" />

<img width="1920" height="926" alt="Control inputs" src="https://github.com/user-attachments/assets/422eea96-516d-42d5-8b41-80868258e719" />

<img width="1600" height="533" alt="7 2" src="https://github.com/user-attachments/assets/645b03f6-204a-44b1-a1c5-0f2368928b08" />


---

## 📬 Contact
* **Aniket Kakde**
* Email: aniketkakde806@gmail.com
