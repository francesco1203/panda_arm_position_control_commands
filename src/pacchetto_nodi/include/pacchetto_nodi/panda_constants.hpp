#pragma once   // evita inclusioni doppie — alternativa moderna agli include guard

#include <vector>
#include <string>


// Costanti specifiche del robot (Panda)
constexpr int N_JOINTS              = 7;
const std::string PLANNING_GROUP    = "panda_arm";
const std::string LAST_LINK         = "panda_hand";
const std::string BASE_LINK         = "panda_link0";        //coincidente con world
const std::vector<std::string> PANDA_JOINT_NAMES = {
    "panda_joint1", "panda_joint2", "panda_joint3",
    "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
};

//per controllo di singolarità panda
constexpr double SINGULARITY_THRESHOLD_WARNING = 0.01;  // soglia per considerare una configurazione di warning
constexpr double SINGULARITY_THRESHOLD_ERROR = 0.001;   // soglia per considerare una configurazione di errore
