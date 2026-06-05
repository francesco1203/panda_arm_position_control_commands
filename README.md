## Nome del Progetto / Project Name

Braccio Panda - Comandi di controllo in posizione / Panda arm - Position control commands

---

### Scopo del Progetto / Project Aim

(IT) : Realizzare i comandi in posizione, movimento giunti e movimento cartesiano, per il braccio robotica Panda (Franka Emika) e simulare

(EN) : To realize position command, joints movement and cartesian movement, for the robotic arm Panda (Franka Emika) and simulate

---

### Linguaggi e tecnologie / Languages & Technologies

- ROS2 JAZZY

---

### Usage

On LinuxUbuntu, using cmd:

- 0. Download and run your moveit and rviz simulator
(you can use: sudo apt install ros-jazzy-moveit-resources-panda)

Write your commands in src/pacchetto_nodi/config/task_config.yaml

- 1. colcon build --symlink-install

CMD1 - start ros2 nodes for executing actions and services
- 2. source install/setup.bash
- 3. ros2 launch pacchetto_nodi preparazione.launch.py

CMD2 - start task node orchestrator
- 4. source install/setup.bash
- 5. ros2 run pacchetto_nodi task_node   $(ros2 pkg prefix pacchetto_nodi)/share/pacchetto_nodi/config/task_config.yaml

### IMPORTANT

Ensure that the following topics are avaiable 
- /joint_states: where to read joint configuration
- /cmd/joint_position: where to publish the joint command to move the simulated robot

If your topics have different names than the default ones, change the file pacchetto_nodi/include/pacchetto_nodi/topic_names.hpp

---

### Struttura / Structure

- src/pacchetto_interfacce          (azioni e servizi/actions and services)
- src/pacchetto_nodi                (nodi da eseguire/nodes to execute)

---

### Esame / Exam

(IT) : studio per l'esame di: PROGRAMMAZIONE DEI ROBOT - facoltà magistrale di Ingegneria Informatica, ramo Automazione

(EN) : Studying for exam: ROBOT PROGRAMMINGS - Master's degree in Computer Engineering, Automation branch
