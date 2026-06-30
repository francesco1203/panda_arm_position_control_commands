#include <memory>
#include <vector>
#include <cmath>
#include <functional>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

//messaggi
#include "sensor_msgs/msg/joint_state.hpp"


#include "pacchetto_interfacce/action/move_joint_lin.hpp"

//mie librerie
#include "pacchetto_nodi/panda_constants.hpp"
#include "pacchetto_nodi/topic_action_service_names.hpp"


using namespace std::placeholders;




class JointTrajGenerator : public rclcpp::Node
{
  public:
    /* Alias */

    // Messaggi
    using JointStateMsg  = sensor_msgs::msg::JointState;

    //publisher e subscriber
    using JointStateSubPtr = rclcpp::Subscription<JointStateMsg>::SharedPtr;
    using JointStatePubPtr = rclcpp::Publisher<JointStateMsg>::SharedPtr;

    //azione MoveJointLin
    using MoveJointLinAct = pacchetto_interfacce::action::MoveJointLin;
    using ServerPtr = rclcpp_action::Server<MoveJointLinAct>::SharedPtr;
    using GoalHandleMoveJointLin = rclcpp_action::ServerGoalHandle<MoveJointLinAct>;
    using GoalHandlePtr = std::shared_ptr<GoalHandleMoveJointLin>;
    using GoalResponse = rclcpp_action::GoalResponse;
    using CancelResponse = rclcpp_action::CancelResponse;  
    using GoalUUID = rclcpp_action::GoalUUID;
    using GoalPtr = std::shared_ptr<const MoveJointLinAct::Goal>;

    //altro
    using joint_config  = std::vector<double>;
    

    /* Costruttore */
    JointTrajGenerator() : Node("joint_traj_generator"),
        n_joints(N_JOINTS),                                     //da panda_constants.hpp
        joint_names_(PANDA_JOINT_NAMES),                        //da panda_constants.hpp
        joint_states_topic_name_(READING_JOINT_STATES_TOPIC),   // da topic_action_service_names.hpp
        command_topic_name_(PUBLISH_JOINT_COMMAND_TOPIC),       // da topic_action_service_names.hpp
        joint_action_name_(MOVE_JOINT_ACTION),                 // da topic_action_service_names.hpp
        q0_received_(false)
    {
         // parametri da launch_file
        this->declare_parameter<double>("T_camp", 0.1);     //default 0.1s -> 10Hz pubblicazione traiettoria
        this->declare_parameter<double>("qdd_c", 1.5);      //default 1.5 rad/s^2 accel in gen.trapezoidale

        T_ = this->get_parameter("T_camp").as_double();
        qdd_c_ = this->get_parameter("qdd_c").as_double();
        

        // Subscriber per leggere la configurazione attuale del robot in tempo reale
        joint_states_sub_ = this->create_subscription<JointStateMsg>(
            joint_states_topic_name_, 10, 
            std::bind(&JointTrajGenerator::joint_states_callback, this, _1)
        );
        

        // TODO : far leggere q0 una sola volta ogni volta che parte l'azione


        // Publisher per inviare i comandi di posizione al robot
        joint_cmd_pub_ = this->create_publisher<JointStateMsg>(
            command_topic_name_, 10
        );


        // Creazione dell'Action Server 
        action_server_ = rclcpp_action::create_server<MoveJointLinAct>(
            this,
            joint_action_name_,

            // 1. Lambda per HANDLE GOAL
            [this](const GoalUUID & uuid, GoalPtr goal) {
                (void)uuid;

                /* CHECK DI VALIDITÀ DELLA RICHIESTA */

                // Rifiuta il goal se il subscriber non ha ancora letto la posizione del robot (errore di sincronizzazione)
                if (!q0_received_) {
                    RCLCPP_ERROR(this->get_logger(), "Impossibile partire: posizione attuale /joint_states non ancora ricevuta.");
                    return GoalResponse::REJECT;
                }

                // Rifiuta se la configurazione desiderata non ha 7 giunti del Panda (errore di comando)
                if (goal->q_desired.size() != static_cast<size_t>(n_joints)) {
                    RCLCPP_ERROR(this->get_logger(), "q_desired deve avere esattamente %d elementi.", n_joints);
                    return GoalResponse::REJECT;
                }

                // Rifiuta se la durata del moto è negativa (errore di comando)
                if (goal->duration <= 0.0) {
                    RCLCPP_ERROR(this->get_logger(), "Il tempo finale duration deve essere maggiore di zero.");
                    return GoalResponse::REJECT;
                }

                // Rifiuta se sbagliato a indicare profilo cinematico (errore di comando)
                if (goal->cinematic_profile!='t' && goal->cinematic_profile!='q') {
                    RCLCPP_ERROR(this->get_logger(), "Il profilo cinematico dev'essere scelto tra 't' e 'q'");
                    return GoalResponse::REJECT;
                }

                //TODO:
                // - rifiuta se q fuori dai limiti di giunto
                // - rifiuta se q_dot e q_dotdot fuori dai limiti dinamici di giunto

                RCLCPP_INFO(this->get_logger(), "Goal accettato! Il robot eseguirà il MoveJointLin in duration=%.2f secondi.", goal->duration);
                return GoalResponse::ACCEPT_AND_EXECUTE;
            },


            // 2. Lambda per HANDLE CANCEL
            [this](const GoalHandlePtr goal_handle) {
                (void)goal_handle;
                RCLCPP_INFO(this->get_logger(), "Richiesta di cancellazione traiettoria ricevuta.");
                return CancelResponse::ACCEPT;
            },


            // 3. Lambda per HANDLE ACCEPTED
            [this](const GoalHandlePtr goal_handle) {
                // Esecuzione asincrona in un thread dedicato
                std::thread{[this, goal_handle]() { this->execute(goal_handle); }}.detach();
            }
        );


        // inizializzo a 0 prima di leggerla
        q0_.resize(n_joints, 0.0);
    

        RCLCPP_INFO(this->get_logger(), "JointTrajGenerator (Azione: move_joint_lin) pronto.");
    }

  private:
    /* Attributi privati */

    // SOTTO-COMPONENTI NODO
    JointStateSubPtr joint_states_sub_;
    JointStatePubPtr joint_cmd_pub_;
    ServerPtr action_server_;

    //parametri del robot e della rete ROS (passati da .hpp)
    int n_joints;
    std::vector<std::string> joint_names_;
    std::string joint_states_topic_name_;
    std::string command_topic_name_;
    std::string joint_action_name_;

    //iper-parametri algoritmi del nodo 
    double T_;                  // Periodo di campionamento traiettoria generata
    double qdd_c_;              // accel. per profili trapezoidali

    //altro
    joint_config q0_;           // Configurazione iniziale
    bool q0_received_;          // Flag di ricezione q0


    /* Callback del subscriber */ 
    //serve a leggere q0
    void joint_states_callback(const JointStateMsg::SharedPtr msg)
    {
        for (int i = 0; i < n_joints; i++) {
            for (size_t j = 0; j < msg->name.size(); j++) {
                if (msg->name[j] == joint_names_[i]) {
                    q0_[i] = msg->position[j];
                }
            }
        }
        q0_received_ = true;
    }


    // Profilo cinematico polinomio quintico: restituisce q_hat(t) ∈ [0,1]
    double quintic(double tau)
    {
        return 6.0 * std::pow(tau, 5) - 15.0 * std::pow(tau, 4) + 10.0 * std::pow(tau, 3);
    }


    // Profilo cinematico trapezoidale: restituisce q_hat(t) ∈ [0,1]
    // t: tempo attuale,tf: durata totale, qdd_c: accelerazione costante, delta_q: spostamento giunto (qf - qi)
    double trapezoidal(double t, double tf, double qdd_c, double delta_q)
    {
        // Lavoriamo esclusivamente con i moduli per la generazione della forma del profilo
        qdd_c = std::abs(qdd_c);
        double abs_delta_q = std::abs(delta_q);

        // Se non deve muoversi, il fattore di avanzamento è già al 100% della (non) traiettoria
        if (abs_delta_q < 1e-9) return 1.0; 

        // Calcolo l'accelerazione minima assoluta
        double qdd_c_min = 4.0 * abs_delta_q / (tf * tf); 

        if (qdd_c < qdd_c_min) {
            qdd_c = qdd_c_min;
        }

        // Argomento della radice protetto da valori negativi dovuti a precisione numerica
        double root_arg = (tf * tf * qdd_c) - (4.0 * abs_delta_q);
        if (root_arg < 0.0) root_arg = 0.0; // Protezione NaN

        // Calcolo del tempo di raccordo
        double tc = tf / 2.0 - 0.5 * std::sqrt(root_arg / qdd_c);
        if (tc < 0.0) tc = 0.0;

        // Generazione profilo normalizzato [0, 1] usando abs_delta_q
        if (t < tc) {
            // Fase 1: accelerazione costante
            return (0.5 * qdd_c * t * t) / abs_delta_q;
        }
        else if (t <= tf - tc) {
            // Fase 2: velocità costante
            return (qdd_c * tc * (t - tc / 2.0)) / abs_delta_q;
        }
        else {
            // Fase 3: decelerazione costante
            double t_inv = tf - t;
            if (t_inv < 0.0) t_inv = 0.0; // Evita sforamenti oltre tf
            return 1.0 - (0.5 * qdd_c * t_inv * t_inv) / abs_delta_q;
        }
    }

    
    /* EXECUTE DELL'AZIONE */
    void execute(const GoalHandlePtr goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Esecuzione MoveJointLin...");

        //estraggo i dati della richiesta, del goal
        const auto goal = goal_handle->get_goal();
        joint_config q_desired = goal->q_desired;      
        double duration = goal->duration;   
        char cinematic_profile = goal->cinematic_profile;                        


        // CATTURA DELLO STATO INIZIALE: cattura istantanea di dove si trova il robot ORA
        joint_config q_start = q0_;


        //numero di campioni traiettoria
        int N = static_cast<int>(duration / T_); 

        
        // loop di esecuzione
        auto feedback = std::make_shared<MoveJointLinAct::Feedback>();
        auto result = std::make_shared<MoveJointLinAct::Result>();
        rclcpp::Rate loop_rate(1.0 / T_); 

        for (int k = 0; k <= N && rclcpp::ok(); k++)
        {
            if (goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Traiettoria MoveJointLin interrotta.");
                return;
            }


            double t = k * T_;              //tempo attuale algoritmico
            double tau = t / duration;      //percentuale completamento
                if (tau > 1.0) tau = 1.0;   //clamp

            joint_config q_k(n_joints);     //obiettivo in var. giunto (prealloc)

            
            if(cinematic_profile == 'q')  // uso profilo cinematico del polinomio quintico
            {
                //genero il profilo (dipende solo dal tempo, mi basta generarlo una sola volta per tutti i giunti)
                double q_hat = quintic(tau);    //quintic usa tau, percentuale completamento (asse tempi normalizzato)


                //aggiorno variabili di giunto - obiettivo

                for (int i = 0; i < n_joints; i++) {

                    //così mi muovo linearmente, con spostamento totale guidato da q_hat
                    q_k[i] = q_start[i] + (q_desired[i] - q_start[i]) * q_hat;
                }
            }
            else    //cinematic profile = 't' -> uso trapezoidale
            {
                //aggiorno variabili di giunto - obiettivo
                
                double delta_q, q_hat;  //inizializzazione

                for (int i = 0; i < n_joints; i++) 
                {
                    delta_q = q_desired[i] - q_start[i];
                    q_hat   = trapezoidal(t, duration, qdd_c_, delta_q);

                    //così mi muovo linearmente, con spostamento totale guidato da q_hat
                    q_k[i]  = q_start[i] + delta_q * q_hat;
                }
            }
            
            
            // Posizione calcolata: invio comando di posizione giunti
            JointStateMsg cmd_msg;
            cmd_msg.header.stamp = this->now();
            cmd_msg.name = joint_names_;
            cmd_msg.position = q_k;                 //comando di posizione giunto
            joint_cmd_pub_->publish(cmd_msg);

            // Feedback dell'azione
            feedback->q_current = q_k;
            feedback->progress = tau;
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }

        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Movimento lineare nello spazio giunti completato.");
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointTrajGenerator>());
  rclcpp::shutdown();
  return 0;
}