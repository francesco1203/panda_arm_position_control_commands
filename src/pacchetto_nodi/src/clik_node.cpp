#include <memory>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"


#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>



#include "pacchetto_nodi/eigen_utilities.hpp" 
#include "pacchetto_nodi/message_utilities.hpp"
#include "pacchetto_nodi/panda_constants.hpp"
#include "pacchetto_nodi/set_bool_srv_utilities.hpp"
#include "pacchetto_nodi/topic_action_service_names.hpp"



using namespace std::chrono_literals;
using namespace std::placeholders;


class Clik : public rclcpp::Node
{
  public:

    /* Alias*/

    //per servizio di attivazione clik
    using ServiceOnOffPtr = SetBoolServerPtr;

    // per publisher e subscriber
    using JointStatePubPtr = rclcpp::Publisher<JointStateMsg>::SharedPtr;   
    using PoseSubPtr = rclcpp::Subscription<PoseStampedMsg>::SharedPtr;
    using JointStateSubPtr = rclcpp::Subscription<JointStateMsg>::SharedPtr;

    //timer
    using TimerPtr = rclcpp::TimerBase::SharedPtr;

    //altro
    using joint_config  = std::vector<double>;


    /* COSTRUTTORE */
    Clik() : Node("clik_node"),
      is_on_(false),
      desired_pose_received_(false),
      q_k_received_(false),
      Q_k_prev_(Quaternion::Identity()),  // inizializza con identità
      singularity_detected_(false),
      n_joints(N_JOINTS),                                                 //da panda_constants.hpp
      joint_names_(PANDA_JOINT_NAMES),                                    //da panda_constants.hpp
      planning_group_name_(PLANNING_GROUP),                               //da panda_constants.hpp
      last_link_name_(LAST_LINK),                                         //da panda_constants.hpp
      cartesian_desired_pose_topic_name_(CARTESIAN_DESIRED_POSE_TOPIC),   // da topic_action_service_names.hpp
      joint_states_topic_name_(READING_JOINT_STATES_TOPIC),               // da topic_action_service_names.hpp
      command_topic_name_(PUBLISH_JOINT_COMMAND_TOPIC),                   // da topic_action_service_names.hpp
      clik_service_on_off_name_(CLIK_SERVICE_ON_OFF)                      // da topic_action_service_names.hpp
    {
       // parametri da launch_file
      this->declare_parameter<double>("Tclik", 0.001);                    //default 0.001s -> 1000Hz, frequenza del clik
      this->declare_parameter<double>("gamma_on_T", 0.5);                 //default 0.5 guadagno del clik
      this->declare_parameter<double>("singularity_trshld_warn", 0.01);   //default 0.01 soglia per considerare a rischio di singolarità 
      this->declare_parameter<double>("singularity_trshld_error", 0.001); //default 0.001 soglia per considerare di essere in singolarità 

      T_clik = this->get_parameter("Tclik").as_double();
      gamma_on_T_clik_ = this->get_parameter("gamma_on_T").as_double();
      singularity_trshld_warn_ = this->get_parameter("singularity_trshld_warn").as_double();
      singularity_trshld_error_ = this->get_parameter("singularity_trshld_error").as_double();


      /* INIZIALIZZAZIONE MOVEIT */
      // Setup MoveIt — Workaround per RobotModelLoader (VEDI CARTESIAN_TRAJ_GENERATOR per spiegazione)
      robot_loader_node_ = std::make_shared<rclcpp::Node>(
          "robot_model_loader_clik",
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

      robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(robot_loader_node_);
      const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader_->getModel();
   
      joint_model_group_ = kinematic_model->getJointModelGroup(planning_group_name_);
      kinematic_state_   = std::make_shared<moveit::core::RobotState>(kinematic_model);
      last_link_         = kinematic_state_->getLinkModel(last_link_name_);


      // Subscriber posa desiderata — pubblicata da cartesian_traj_generator
      desired_pose_sub_ = this->create_subscription<PoseStampedMsg>(
        cartesian_desired_pose_topic_name_, 10,
        std::bind(&Clik::read_desired_pose_callback, this, _1)
      );


      // Subscriber configurazione attuale — pubblicata dal simulatore
      joint_states_sub_ = this->create_subscription<JointStateMsg>(
        joint_states_topic_name_, 10,
        std::bind(&Clik::read_joint_states_callback, this, _1)
      );


      // Publisher comandi al robot
      joint_cmd_pub_ = this->create_publisher<JointStateMsg>(
        command_topic_name_, 10
      );


      // Servizio on_off
      on_off_service_ = this->create_service<SetBoolSrv>(
        clik_service_on_off_name_,
        std::bind(&Clik::on_off_callback, this, _1, _2)
      );


      // Timer del clik — parte disattivato
      timer_ = this->create_wall_timer(1ms, std::bind(&Clik::clik_callback, this));
      timer_->cancel();


      //inizializza q_k_ con n_joints (7) zeri
      q_k_.resize(n_joints, 0.0);


      RCLCPP_INFO(this->get_logger(), "Nodo CLIK pronto. In attesa di attivazione...");
    }

  private:
  
    /* sottocomponenti del nodo */
    ServiceOnOffPtr on_off_service_;    //service server
    PoseSubPtr desired_pose_sub_;       //subscriber alla posa desiderata
    JointStateSubPtr joint_states_sub_; //subscriber alla configurazione attuale
    JointStatePubPtr joint_cmd_pub_;    //publisher alla configurazione da eseguire
    TimerPtr timer_;                    //timer per ciclo di controllo

    //temp variables / altro
    bool is_on_;                          // stato del clik
    bool desired_pose_received_;          // flag: ho ricevuto almeno una posa?
    bool q_k_received_;                   // flag: ho ricevuto almeno un joint_state?

    Quaternion Q_k_prev_;                 // Quaternione al passo precedente per la continuità
    PoseStampedMsg desired_pose_stamped;  // ultima posa desiderata ricevuta
    joint_config q_k_;                    // configurazione attuale del robot

    bool singularity_detected_;           // true = clik fermato per singolarità

    //parametri robot e rete ros (passati da files.hpp)
    int n_joints;
    std::vector<std::string> joint_names_;
    std::string planning_group_name_;
    std::string last_link_name_;
    std::string cartesian_desired_pose_topic_name_;
    std::string joint_states_topic_name_;
    std::string command_topic_name_;
    std::string clik_service_on_off_name_;

    //iper-parametri degli algoritmi, passati da launch file
    double T_clik;                         // periodo di campionamento del controllo CLIK
    double gamma_on_T_clik_;               // guadagno del controllo CLIK
    double singularity_trshld_warn_;       // soglia per considerare a rischio di singolarità 
    double singularity_trshld_error_;      //soglia per considerare di essere in singolarità 


    // MoveIt
    rclcpp::Node::SharedPtr robot_loader_node_;
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotStatePtr kinematic_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    const moveit::core::LinkModel* last_link_;
    

    /* CALLBACKS */

    /* Callback subscriber posa desiderata
    * Chiamata da ROS2 ogni volta che cartesian_traj_generator pubblica su /desired_cartesian_pose
    * Salva la posa in desired_pose_ per usarla in clik_callback
    */
    void read_desired_pose_callback(const PoseStampedMsg::SharedPtr msg)
    {
      desired_pose_stamped = *msg;    // salva l'ultima posa ricevuta
      desired_pose_received_ = true;  // segnala che è disponibile
    }

    /* Callback subscriber configurazione attuale
    * Chiamata da ROS2 ogni volta che il simulatore pubblica su /joint_states
    * Salva la configurazione in q_k_ per usarla in clik_callback
    */
    void read_joint_states_callback(const JointStateMsg::SharedPtr msg)
    {
      for (int i = 0; i < n_joints; i++) {
        for (size_t j = 0; j < msg->name.size(); j++) {
          if (msg->name[j] == joint_names_[i]) {
            q_k_[i] = msg->position[j];
          }
        }
      }
      q_k_received_ = true;
    }

    /* Callback del servizio on_off basato su SetBool */
    void on_off_callback(const SetBoolRequestPtr request, const SetBoolResponsePtr response)
    {
      if (request->data && !is_on_)
      {
        // sono in singolarità? Se sì, non attivare il clik e segnala l'errore
        if (singularity_detected_) {
          RCLCPP_ERROR(this->get_logger(),
              "Impossibile riattivare: singolarita' rilevata. Porta prima il robot fuori dalla singolarità.");
          
          response->success = false; // response->success sostituisce response->is_on
          response->message = "Errore: Robot in singolarità critica!";
          
          return;
        }

        // Controlla che siano disponibili posa e configurazione
        if (!q_k_received_) {
          RCLCPP_ERROR(this->get_logger(),
            "Impossibile attivare: configurazione attuale non ricevuta.");
          response->success = false;
          response->message = "Errore: /joint_states non ancora ricevuti!";
          return;
        }

        // Attivazione nominale
        RCLCPP_INFO(this->get_logger(), "CLIK attivato.");
        is_on_ = true;
        timer_->reset();   // avvia il timer

        response->success = true;
        response->message = "Clik attivato con successo.";
      }
      else if (!request->data && is_on_)
      {
        // Disattivazione nominale
        RCLCPP_INFO(this->get_logger(), "CLIK disattivato.");
        is_on_ = false;
        timer_->cancel();  // ferma il timer

        response->success = true;
        response->message = "Clik disattivato con successo.";
      }
      else
      {
        //stato già coerente con la richiesta
        RCLCPP_INFO(this->get_logger(),
          "CLIK già in stato: %s", is_on_ ? "ON" : "OFF");

        response->success = true;
        response->message = is_on_ ? "Il clik era già attivo." : "Il clik era già spento.";
      }
      
    }

    /* Callback del timer — ciclo di controllo CLIK
    * Chiamata da ROS2 ogni T_clik secondi quando il timer è attivo
    * Legge da desired_pose_ e q_k_ per calcolare la nuova configurazione da eseguire
    * 
    * Implementa l'algoritmo CLIK a tempo discreto:
    *
    *   qdot_k = J†(q_k) * (v_d + gamma * e_k)
    *   q_k+1  = q_k + T * qdot_k
    *
    * Con v_d = 0 (nessuna velocità desiderata feedforward),
    * quindi si semplifica in:
    * 
    * qdot_k = J†(q_k) * gamma * e_k
    * q_k+1  = q_k + T * qdot_k
    * 
    * L'errore e_k è 6D:
    *   - prime 3 componenti: errore di posizione   e_p = p_d - p_k
    *   - ultime 3 componenti: errore di orientamento e_o = epsilon di (Q_d * Q_k^-1)
    * 
    * Pubblica q su /cmd/joint_position
    */
    void clik_callback()
    {
      // 0. Se non è ancora arrivata nessuna posa desiderata, aspetta
      if (!desired_pose_received_) {
        RCLCPP_WARN_ONCE(this->get_logger(), "In attesa della posa desiderata su /desired_pose...");
        return;
      }


      /* 1. Spacchetto Posa desiderata */

      // Posizione desiderata p_d (vettore 3D)
      Vector3d p_d(
          desired_pose_stamped.pose.position.x,
          desired_pose_stamped.pose.position.y,
          desired_pose_stamped.pose.position.z);

      // Orientamento desiderato Q_d (quaternione)
      Quaternion Q_d(
          desired_pose_stamped.pose.orientation.w,
          desired_pose_stamped.pose.orientation.x,
          desired_pose_stamped.pose.orientation.y,
          desired_pose_stamped.pose.orientation.z);
      Q_d.normalize(); // sicurezza: forza il quaternione a essere unitario

      
      /* 2. Calcolare cinematica diretta fkine(q_k_) → posa attuale */ 
      for (size_t i = 0; i < joint_names_.size(); i++) { //aggiorna stato attuale del robot in kinematic state
          kinematic_state_->setJointPositions(joint_names_[i], &q_k_[i]);
      }

      // getGlobalLinkTransform matrice 4x4 rotoTraslazione b_T_e (base → end effector)
      const RotoTraslMatrix& b_T_e = kinematic_state_->getGlobalLinkTransform(last_link_);

      // Estrai posizione attuale p_k e orientamento come quaternione
      Vector3d p_k = b_T_e.translation();
      Quaternion Q_k(b_T_e.rotation());
      Q_k.normalize();

      // Continuità del quaternione: evita i salti di segno tra un passo e l'altro
      // Funzione fornita dal professore nella traccia
      Q_k = quaternionContinuity(Q_k, Q_k_prev_);
      Q_k_prev_ = Q_k;                             // salva per il passo successivo


      /* 3. Calcolare errore 6D: */

      Vector6d e_k;

      e_k.block<3, 1>(0, 0) = p_d - p_k;      //traslazione

      Quaternion DeltaQ = Q_d * Q_k.inverse();
      DeltaQ.normalize();
      e_k.block<3, 1>(3, 0) = DeltaQ.vec();   //orientamento



      /* 4. Calcolare jacobiano J(q_k_) con MoveIt */
      MatrixXd J(6, n_joints);
      Vector3d reference_point(0.0, 0.0, 0.0);      //se messo a 0, calcola J rispetto end effector, altrimenti a un punto traslato del riferimento

      bool ok_calcolo = kinematic_state_->getJacobian(
          joint_model_group_,
          last_link_,
          reference_point,
          J
        );                         //riempie J con il jacobiano calcolato da MoveIt

      
      if (!ok_calcolo) {
          RCLCPP_ERROR(this->get_logger(), "Calcolo Jacobiano fallito!");
          return;
      }


      /*  5. Controllo di singolarità */

      // Eigen calcola i valori singolari di J in ordine decrescente
      // jacobiSvd è il metodo più robusto ma BDCSVD è più veloce per matrici grandi

      // Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
      Eigen::BDCSVD<Eigen::MatrixXd> svd(J);
      double sigma_min = svd.singularValues().minCoeff();

      if (sigma_min < singularity_trshld_error_)
      {
          // Singolarità critica — ferma il clik
          RCLCPP_ERROR(this->get_logger(),
              "SINGOLARITA' CRITICA rilevata! sigma_min=%.6f < %.6f. Fermo il CLIK.",
              sigma_min, singularity_trshld_error_);

          singularity_detected_ = true;
          is_on_ = false;
          timer_->cancel();   // ferma il timer — il clik non gira più

          return; //blocca l'esecuzione
      }
      else if (sigma_min < singularity_trshld_warn_)
      {
          // Avvicinamento alla singolarità — avvisa ma continua
          RCLCPP_WARN(this->get_logger(),
              "Attenzione: vicino alla singolarita'. sigma_min=%.6f", sigma_min);
      }


      /* 6. Calcolare q_dot = J† * gamma * e_k */
    
      double gamma = gamma_on_T_clik_ / T_clik;   
      VectorXd q_dot = J.completeOrthogonalDecomposition().solve(gamma * e_k);
      //completeOrthogonalDecomposition() è un metodo di Eigen che calcola la pseudo-inversa di J in modo robusto e la moltiplica per gamma * e_k

      
      /* 7. Calcolare q_k+1 = q_k_ + T * q_dot */
      VectorXd q;
      kinematic_state_->copyJointGroupPositions(joint_model_group_, q);

      q = q + q_dot * T_clik;

      kinematic_state_->setJointGroupPositions(joint_model_group_, q);


      /* 8. Pubblicare q_k+1 su topic di comando */
      JointStateMsg out_msg;
      kinematic_state_->copyJointGroupPositions(joint_model_group_, out_msg.position);
      out_msg.name = joint_model_group_->getActiveJointModelNames();
      out_msg.header.stamp = this->now();
      joint_cmd_pub_->publish(out_msg);

      
      RCLCPP_INFO_ONCE(this->get_logger(), "CLIK in esecuzione...");
    }



    /* ALTRI METODI PRIVATI*/
    // evita salti di segno nel quaternione tra iterazioni successive
    Quaternion quaternionContinuity(const Quaternion& Q_k,  const Quaternion& Q_k_minus_1)
    {
        // Prodotto scalare tra le parti vettoriali dei due quaternioni
        double dot = Q_k.vec().transpose() * Q_k_minus_1.vec();

        if (dot < -0.01) {
            // Inverti il segno — rappresenta la stessa rotazione ma è continuo
            Quaternion out(Q_k);
            out.vec() = -out.vec();
            out.w()   = -out.w();
            return out;
        }
        return Q_k;
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Clik>());
  rclcpp::shutdown();
  return 0;
}