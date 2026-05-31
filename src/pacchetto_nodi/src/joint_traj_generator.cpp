#include <memory>
#include <vector>
#include <cmath>
#include <functional>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "pacchetto_interfacce/action/move_joint_lin.hpp"

//costanti
#include "pacchetto_nodi/message_alias.hpp" 
#include "pacchetto_nodi/panda_constants.hpp"


using namespace std::placeholders;


constexpr double T_CAMP      = 0.1;  // 10 Hz  campionamento della traiettoria


class JointTrajGenerator : public rclcpp::Node
{
  public:
    /* Alias */

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
    
    /* dai .hpp 
      Alias Messaggi --> Pose, PoseStamped, JointState + joint_config
      Costanti N_JOINTS, PLANNING_GROUP, LAST_LINK, PANDA_JOINT_NAMES
    */



    /* Costruttore */
    JointTrajGenerator() : Node("joint_traj_generator"),
        joint_names_(PANDA_JOINT_NAMES),
        q0_received_(false),
        T_(T_CAMP)
    {
        // Subscriber per leggere la configurazione attuale del robot in tempo reale
        joint_states_sub_ = this->create_subscription<JointStateMsg>(
            "/joint_states", 10, std::bind(&JointTrajGenerator::joint_states_callback, this, _1));
        

        // TODO : far leggere q0 una sola volta ogni volta che parte l'azione


        // Publisher per inviare i comandi di posizione al robot
        joint_cmd_pub_ = this->create_publisher<JointStateMsg>("/cmd/joint_position", 10);


        // Creazione dell'Action Server con CALLBACK LAMBDA (Stile ROS 2 Jazzy)
        action_server_ = rclcpp_action::create_server<MoveJointLinAct>(
            this,
            "move_joint_lin_action", // Nome dell'azione nella rete ROS


            // 1. Lambda per HANDLE GOAL
            [this](const GoalUUID & uuid, GoalPtr goal) {
                (void)uuid;

                // Rifiuta il goal se il subscriber non ha ancora letto la posizione del robot
                if (!q0_received_) {
                    RCLCPP_ERROR(this->get_logger(), "Impossibile partire: posizione attuale /joint_states non ancora ricevuta.");
                    return GoalResponse::REJECT;
                }

                // Rifiuta se la configurazione desiderata non ha i 7 giunti del Panda
                if (goal->q_desired.size() != 7) {
                    RCLCPP_ERROR(this->get_logger(), "q_desired deve avere esattamente 7 elementi.");
                    return GoalResponse::REJECT;
                }

                if (goal->duration <= 0.0) {
                    RCLCPP_ERROR(this->get_logger(), "Il tempo finale duration deve essere maggiore di zero.");
                    return GoalResponse::REJECT;
                }

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
        q0_.resize(N_JOINTS, 0.0);
    

        RCLCPP_INFO(this->get_logger(), "JointTrajGenerator (Azione: move_joint_lin) pronto.");
    }

  private:
    /* Attributi privati */

    // SOTTO-COMPONENTI NODO
    JointStateSubPtr joint_states_sub_;
    JointStatePubPtr joint_cmd_pub_;
    ServerPtr action_server_;

    std::vector<std::string> joint_names_;
    joint_config q0_;           // Configurazione iniziale
    bool q0_received_;          // Flag di ricezione
    double T_;                  // Periodo di campionamento


    /* Callback del subscriber */ 
    //serve a leggere q0
    void joint_states_callback(const JointStateMsg::SharedPtr msg)
    {
        for (int i = 0; i < N_JOINTS; i++) {
            for (size_t j = 0; j < msg->name.size(); j++) {
                if (msg->name[j] == joint_names_[i]) {
                    q0_[i] = msg->position[j];
                }
            }
        }
        q0_received_ = true;
    }


    // Legge oraria del polinomio quintico
    double quintic(double tau)
    {
        return 6.0 * std::pow(tau, 5) - 15.0 * std::pow(tau, 4) + 10.0 * std::pow(tau, 3);
    }

    
    /* EXECUTE DELL'AZIONE */
    void execute(const GoalHandlePtr goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Avvio interpolazione quintica per MoveJointLin...");

        //estraggo i dati del goal
        const auto goal = goal_handle->get_goal();
        joint_config q_desired = goal->q_desired;      
        double duration = goal->duration;                             


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

            double t = k * T_;
            double tau = t / duration;
            if (tau > 1.0) tau = 1.0;   //clamp


            // Interpolazione lineare asse per asse guidata dal profilo quintico morbido
            double q_hat = quintic(tau);
            joint_config q_k(N_JOINTS);
            for (int i = 0; i < N_JOINTS; i++) {

                //così mi muovo linearmente, con spostamento totale guidato da q_hat
                q_k[i] = q_start[i] + (q_desired[i] - q_start[i]) * q_hat;
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