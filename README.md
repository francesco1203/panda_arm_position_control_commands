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

### Utilizzo / Usage

On LinuxUbuntu, using cmd:

- 1. colcon build --symlink-install

Write your commands in src/pacchetto_nodi/config/task_config.yaml

CMD1 - start RViz simulator and MoveIt:
- 2. source install/setup.bash
- 3. ros2 launch panda_description_and_moveit demo.launch.py

CMD2 - start ros2 nodes for executing actions
- 3. source install/setup.bash
- 4. ros2 launch pacchetto_nodi preparazione.launch.py

CMD3 - start task node orchestrator
- 5. source install/setup.bash
- 6. ros2 run pacchetto_nodi task_node   $(ros2 pkg prefix pacchetto_nodi)/share/pacchetto_nodi/config/task_config.yaml


---

### Struttura / Structure

- src/pacchetto_interfacce          (azioni e servizi/actions and services)
- src/pacchetto_nodi                (nodi da eseguire/nodes to execute)
- src/panda_description_and_moveit  (urfd, Rviz config, launch file for simulator)

---

### Esame / Exam

(IT) : studio per l'esame di: PROGRAMMAZIONE DEI ROBOT - facoltà magistrale di Ingegneria Informatica, ramo Automazione

(EN) : Studying for exam: ROBOT PROGRAMMINGS - Master's degree in Computer Engineering, Automation branch
