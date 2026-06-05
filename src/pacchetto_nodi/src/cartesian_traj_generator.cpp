#include <memory>
#include <vector>
#include <cmath>
#include <functional>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "pacchetto_interfacce/action/move_cartesian_lin.hpp"

//costanti
#include "pacchetto_nodi/message_alias.hpp"
#include "pacchetto_nodi/panda_constants.hpp"
#include "pacchetto_nodi/eigen_alias.hpp"
#include "pacchetto_nodi/topic_names.hpp"


using namespace std::placeholders;
using namespace std::chrono_literals;

/* COSTANTI NODO*/
constexpr double T_CAMP = 0.1;                // 10 Hz
constexpr double cdd_c_rot_CONST = 1.0;       // rad/s²  (per profilo trapezoidale)
constexpr double cdd_c_trasl_CONST = 1.0;     // m/s²  (per profilo trapezoidale)


class CartesianTrajGenerator : public rclcpp::Node
{
  public:
    /* Alias */

    //publisher e subscriber
    using JointStateSubPtr = rclcpp::Subscription<JointStateMsg>::SharedPtr;
    using PosePubPtr = rclcpp::Publisher<PoseStampedMsg>::SharedPtr;

    // azione MoveCartesianLin
    using MoveCartesianLinAct = pacchetto_interfacce::action::MoveCartesianLin;
    using ServerPtr = rclcpp_action::Server<MoveCartesianLinAct>::SharedPtr;
    using GoalHandleCartesiano = rclcpp_action::ServerGoalHandle<MoveCartesianLinAct>;
    using GoalHandleCartesianoPtr = std::shared_ptr<GoalHandleCartesiano>;
    using GoalResponse = rclcpp_action::GoalResponse;
    using CancelResponse = rclcpp_action::CancelResponse;
    using GoalUUID = rclcpp_action::GoalUUID;
    using GoalPtr = std::shared_ptr<const MoveCartesianLinAct::Goal>;

    //altro
    using joint_config  = std::vector<double>;

    /* dai .hpp 
      Alias Messaggi --> Pose, JointState
      Costanti N_JOINTS, PLANNING_GROUP, LAST_LINK, PANDA_JOINT_NAMES
    */


    /* Costruttore */
    CartesianTrajGenerator() : Node("cartesian_traj_generator"),
      T_(T_CAMP),
      cdd_c_trasl_(cdd_c_trasl_CONST),
      cdd_c_rot_(cdd_c_rot_CONST)
    {

      // =========================================================
      // Workaround RobotModelLoader:
      // RobotModelLoader non può essere costruito nel costruttore
      // della classe perché ha bisogno di uno shared_ptr al nodo
      // già completamente costruito. Per questo creiamo un nodo
      // separato interno da passare al loader.
      // =========================================================
      robot_loader_node_ = std::make_shared<rclcpp::Node>(
        "robot_model_loader_cartesian",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

      robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(robot_loader_node_);
      const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader_->getModel();

      RCLCPP_INFO(this->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());


      // Gruppo cinematico e link finale
      joint_model_group_ = kinematic_model->getJointModelGroup(PLANNING_GROUP);
      kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model);
      last_link_ = kinematic_state_->getLinkModel(LAST_LINK);


      // Subscriber a /joint_states
      joint_states_sub_ = this->create_subscription<JointStateMsg>(
        READING_JOINT_STATES_TOPIC, 10,

        // lambda callback: salva l'ultimo joint state ricevuto
        [this](const JointStateMsg::SharedPtr msg) {
            last_joint_state_ = *msg;       // salva il messaggio
            joint_state_received_ = true;   // segnala che è arrivato almeno uno
        }
      );


      // Publisher posa desiderata → letta dal clik
      cartesian_pose_pub_ = this->create_publisher<PoseStampedMsg>(
        CARTESIAN_DESIRED_POSE_TOPIC, 10
      );


      // Action server
      action_server_ = rclcpp_action::create_server<MoveCartesianLinAct>(
        this,
        "cartesian_traj_action",

        // HANDLE GOAL
        [this](const GoalUUID& uuid, GoalPtr goal) {
          (void)uuid;
          if (goal->duration <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "duration deve essere maggiore di zero.");
            return GoalResponse::REJECT;
          }

          // Controlliamo che il quaternione sia unitario (norma ≈ 1)
          // Usiamo una tolleranza di 1e-3: abbastanza stretta da rilevare
          Quaternion Q_check(
            goal->pose_desired.orientation.w,
            goal->pose_desired.orientation.x,
            goal->pose_desired.orientation.y,
            goal->pose_desired.orientation.z);

          double norma = Q_check.norm();
          if (std::abs(norma - 1.0) > 1e-3) {
            RCLCPP_ERROR(this->get_logger(),
              "Quaternione desiderato non unitario! norma=%.6f (attesa 1.0)", norma);
            return GoalResponse::REJECT;
          }

          return GoalResponse::ACCEPT_AND_EXECUTE;
        },

        // HANDLE CANCEL
        [this](const GoalHandleCartesianoPtr goal_handle) {
          (void)goal_handle;
          RCLCPP_INFO(this->get_logger(), "Cancellazione traiettoria cartesiana richiesta.");
          return CancelResponse::ACCEPT;
        },

        // HANDLE ACCEPTED — lancia execute in thread separato
        [this](const GoalHandleCartesianoPtr goal_handle) {
          std::thread{[this, goal_handle]() { this->execute(goal_handle); }}.detach();
        }
      );


      RCLCPP_INFO(this->get_logger(), "CartesianTrajGenerator pronto.");
    }


  private:

    /* SOTTO-COMPONENTI NODO */
    PosePubPtr cartesian_pose_pub_;       //publisher su /cartesian_pose_desired
    JointStateSubPtr joint_states_sub_;   // subscriber a /joint_states
    ServerPtr action_server_;

    JointStateMsg last_joint_state_;      // ultimo messaggio ricevuto
    bool joint_state_received_ = false;   // flag: ho ricevuto almeno un messaggio?

    float T_;
    float cdd_c_trasl_;
    float cdd_c_rot_;

    // MoveIt
    rclcpp::Node::SharedPtr robot_loader_node_;
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotStatePtr kinematic_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    const moveit::core::LinkModel* last_link_;



    //profilo cinematico quintico: restituisce q_hat(t) ∈ [0,1]
    double quintic(double tau)
    {
      return 6.0 * std::pow(tau, 5) - 15.0 * std::pow(tau, 4) + 10.0 * std::pow(tau, 3);
    }

    // Profilo cinematico trapezoidale: restituisce q_hat(t) ∈ [0,1]
    // t: tempo attuale,tf: durata totale, cdd_c: accelerazione costante, delta_c: spostamento cartesiano
    double trapezoidal(double t, double tf, double cdd_c, double delta_c)
    {
        // Caso degenere: nessuno spostamento richiesto
        if (std::abs(delta_c) < 1e-9) return 0.0;


        // Calcolo l'accelerazione minima assoluta (sempre positiva) per raggiungere l'obiettivo
        double cdd_c_min = 4.0 * std::abs(delta_c) / (tf * tf); 

        // Verifico se il modulo dell'accelerazione fornita è insufficiente
        if (std::abs(cdd_c) < cdd_c_min) {
            // Determino il segno corretto: deve seguire il verso dello spostamento (delta_q)
            double sign = (delta_c >= 0.0) ? 1.0 : -1.0;
            
            // Applico il valore minimo mantenendo il segno corretto
            double new_cdd_c = sign * cdd_c_min;

            RCLCPP_WARN(this->get_logger(), 
                "Rilevata accel. cartesiana troppo bassa. Clamped da %.4f a %.4f", cdd_c, new_cdd_c);

            cdd_c = new_cdd_c;
        }


        // Calcolo del tempo di raccordo
        double tc = tf / 2.0 - 0.5 * std::sqrt((tf * tf * cdd_c - 4.0 * delta_c) / cdd_c);

        // Clamping di tc (robustezza numerica al caso limite)
        if (tc < 0.0) tc = 0.0;


        //generazione profilo trapezoidale
        if (t < tc) {
            // Fase 1: accelerazione costante
            return (0.5 * cdd_c * t * t) / delta_c;
        }
        else if (t <= tf - tc) {
            // Fase 2: velocità costante
            return (cdd_c * tc * (t - tc / 2.0)) / delta_c;
        }
        else {
            // Fase 3: decelerazione costante (simmetrica alla fase 1)
            return 1.0 - (0.5 * cdd_c * (tf - t) * (tf - t)) / delta_c;
        }
    }


     /* EXECUTE DELL'AZIONE
     *
     * Genera la traiettoria da posa attuale → posa desiderata:
     *   - posizione:    interpolazione lineare  p(s) = p0 + s*(pf - p0)
     *   - orientamento: SLERP                   Q(s) = slerp(Q0, Qf, s)
     *
     * dove s = quintic(t/tf) è il profilo quintico che rende il moto morbido.
     * La traiettoria viene pubblicata a 10Hz su "desired_cartesian_pose" come PoseStampedMsg.
     */
    void execute(const GoalHandleCartesianoPtr goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Esecuzione MoveCartLin...");


      const auto goal = goal_handle->get_goal();
      double tf = goal->duration;
      char cinematic_profile = goal->cinematic_profile;  


      // ── 1. Leggi la configurazione attuale da /joint_states ───────
      RCLCPP_INFO(this->get_logger(), "Lettura configurazione attuale...");
      JointStateMsg joint_current;

      //aspetta con un attesa non bloccante, ma attiva, se è arrivato il primo joint_states
      rclcpp::Rate wait_rate(100);  // controlla 100 volte al secondo
      while (!joint_state_received_ && rclcpp::ok()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
              "In attesa di /joint_states...");
          wait_rate.sleep();
      }

      // Copia l'ultimo joint state ricevuto (quello salvato dal subscriber)
      joint_current = last_joint_state_;
      
      // Aggiorna kinematic_state con la configurazione attuale
      for (size_t i = 0; i < joint_current.name.size(); i++) {
        kinematic_state_->setJointPositions(
          joint_current.name[i], &joint_current.position[i]);
      }



      // ── 2. Calcola posa iniziale con fkine ──────
      const RotoTraslMatrix& b_T_e = kinematic_state_->getGlobalLinkTransform(last_link_);
      Vector3d p0 = b_T_e.translation();    // posa iniziale dal fkine
      Quaternion Q0(b_T_e.rotation());      // orientamento iniziale dal fkine

      RCLCPP_INFO(this->get_logger(),
        "p0: [%.3f, %.3f, %.3f]", p0.x(), p0.y(), p0.z());


      // ── 3. Leggi posa finale dal goal ────────────

      // Posizione finale dal goal
      double pf_x = goal->pose_desired.position.x;
      double pf_y = goal->pose_desired.position.y;
      double pf_z = goal->pose_desired.position.z;

      // Determina orientamento finale Qf dal goal (versione avanzata del programma)
      Quaternion Qf;

      
      if (goal->keep_prev_orientation) { // Tieni l'orientamento attuale costante:
        Qf = Q0;
        RCLCPP_INFO(this->get_logger(), 
          "Orientamento desiderato: precedente costante");
      } 
      else  // Usa il quaternione desiderato dal goal
      { 
          Qf = Quaternion(
              goal->pose_desired.orientation.w,
              goal->pose_desired.orientation.x,
              goal->pose_desired.orientation.y,
              goal->pose_desired.orientation.z
            );
          Qf.normalize();

      }

      RCLCPP_INFO(this->get_logger(),
        "pf: [%.3f, %.3f, %.3f]", pf_x, pf_y, pf_z);
      RCLCPP_INFO(this->get_logger(),
        "Qf: [w=%.3f, x=%.3f, y=%.3f, z=%.3f]",
        Qf.w(), Qf.x(), Qf.y(), Qf.z());



      // ── 4. Genera la traiettoria a 1/T (10) Hz ─────

      int N = static_cast<int>(tf / T_);   // numero di step della traiettoria

      auto feedback = std::make_shared<MoveCartesianLinAct::Feedback>();
      auto result = std::make_shared<MoveCartesianLinAct::Result>();
      rclcpp::Rate loop_rate(10);  // 10 Hz

      for (int k = 0; k <= N && rclcpp::ok(); k++)
      {
        // Controlla cancellazione
        if (goal_handle->is_canceling()) {
          result->success = false;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Traiettoria cartesiana cancellata.");
          return;
        }

        /*NOTA ── Interpolazione orientamento: SLERP(Q0, Qf, s) ───────────────────
          slerp(Q0, Qf, s) interpola sfericamente tra i due quaternioni.
          Con s=0 → Q0 (orientamento iniziale)
          Con s=1 → Qf (orientamento desiderato dal goal)
          
          Eigen gestisce automaticamente il caso Q0·Qf < 0
          (percorso più breve sulla sfera unitaria — niente salti bruschi)
        */


        // Calcola tau ∈ [0,1]: frazione di tempo trascorsa
        double t   = k * T_;
        double tau = t / tf;
        if (tau > 1.0) tau = 1.0;   // clamp finale

        //risultati, preallocazione
        double px, py, pz;
        Quaternion Q_interp;

        if(cinematic_profile == 'q')  // uso profilo cinematico del polinomio quintico
        {
          /*NOTA
          Siccome il profilo quintico dipende solo dal tempo iniziale al tempo finale, 
          avremo un solo s valido per tutto, sia traslazione che rotazione.
          */

          // Profilo quintico: s ∈ [0,1] con velocità nulla agli estremi
          double s = quintic(tau);


          // ── Interpolazione 
          px = p0.x() + (pf_x - p0.x()) * s;
          py = p0.y() + (pf_y - p0.y()) * s;
          pz = p0.z() + (pf_z - p0.z()) * s;
          Q_interp = Q0.slerp(s, Qf);
        }
        else    //cinematic profile = 't' -> uso trapezoidale
        {
          /*NOTA
          Siccome il profilo trapezoidale dipende oltre che dal tempo iniziale al tempo finale, 
          anche dalla distanza tra il target e la posa attuale, scegliamo di avere due 's', una per 
          la traslazione e una per la rotazione
          */

          // Spostamento traslazionale: norma euclidea tra posizione iniziale e finale
          double delta_p = std::sqrt(
              std::pow(pf_x - p0.x(), 2) +
              std::pow(pf_y - p0.y(), 2) +
              std::pow(pf_z - p0.z(), 2)
          );

          // Spostamento rotazionale: angolo geodetico tra Q0 e Qf sulla sfera S³
          // Il prodotto scalare Q0·Qf può essere > 1 per errori numerici → clamp a [-1, 1]
          // Il valore assoluto garantisce il percorso più breve (< π)
          double dot = std::abs(Q0.dot(Qf));
          dot = std::min(dot, 1.0);                  // clamp per robustezza numerica
          double delta_r = 2.0 * std::acos(dot);    // angolo geodetico ∈ [0, π]


          //calcolo le due s
          double s_pos = trapezoidal(t, tf, cdd_c_trasl_, delta_p);   // per px, py, pz
          double s_rot = trapezoidal(t, tf, cdd_c_rot_, delta_r);     // per SLERP

          // ── Interpolazione
          px = p0.x() + (pf_x - p0.x()) * s_pos;
          py = p0.y() + (pf_y - p0.y()) * s_pos;
          pz = p0.z() + (pf_z - p0.z()) * s_pos;
          Q_interp = Q0.slerp(s_rot, Qf);
        }



        // ── 5. Costruisci e pubblica il messaggio ───────────────────────────────
        PoseStampedMsg pose_msg;
        pose_msg.header.stamp    = this->now();
        pose_msg.header.frame_id = BASE_LINK ;    //panda_link0 coincidente con world

        pose_msg.pose.position.x = px;
        pose_msg.pose.position.y = py;
        pose_msg.pose.position.z = pz;

        // Orientamento interpolato
        pose_msg.pose.orientation.w = Q_interp.w();
        pose_msg.pose.orientation.x = Q_interp.x();
        pose_msg.pose.orientation.y = Q_interp.y();
        pose_msg.pose.orientation.z = Q_interp.z();

        cartesian_pose_pub_->publish(pose_msg);



        // ── 6. Feedback all'action client (task_node lo riceve e lo stampa) ---
        feedback->pose_current = pose_msg.pose;
        feedback->progress     = tau;
        goal_handle->publish_feedback(feedback);


        RCLCPP_INFO(this->get_logger(),
          "Progresso: %.1f%% - p: [%.3f, %.3f, %.3f]",
          tau * 100.0,
          pose_msg.pose.position.x,
          pose_msg.pose.position.y,
          pose_msg.pose.position.z);

        loop_rate.sleep();
      }

      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Traiettoria cartesiana completata.");
    }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianTrajGenerator>());
  rclcpp::shutdown();
  return 0;
}