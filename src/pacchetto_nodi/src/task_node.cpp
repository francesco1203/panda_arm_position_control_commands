/* COME LANCIARLO DA TERMINALE
 *
  ros2 run pacchetto_nodi task_node \
  $(ros2 pkg prefix pacchetto_nodi)/share/pacchetto_nodi/config/task_config.yaml
 * 
*/

#include <memory>
#include <vector>
#include <chrono>
#include <functional>
#include <thread>
#include <future>
#include <iostream>   // per std::cin, lettura tastiera

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

//azioni e servizi
#include "pacchetto_interfacce/action/move_joint_lin.hpp"  
#include "pacchetto_interfacce/action/move_cartesian_lin.hpp"
#include "pacchetto_interfacce/srv/on_off.hpp"


#include "pacchetto_nodi/panda_constants.hpp"
#include "pacchetto_nodi/message_alias.hpp"
#include "pacchetto_nodi/eigen_alias.hpp"


#include "yaml-cpp/yaml.h"   // parser YAML



using namespace std::chrono_literals;


//alias globali
using joint_config  = std::vector<double>;


// Rappresenta un singolo movimento, joint o cartesiano
struct Move {
    char type;                   // 'j' o 'J' joint / 'c' o 'C' cartesian
    joint_config q_desired;      // usato solo se type == "joint"
    double x, y, z;              // usato solo se type == "cartesian"
    Quaternion Q;                // usato solo se type == "cartesian" (versione avanzata)*
    bool keep_prev_orientation;  // usato solo se type == "cartesian" (versione avanzata)*
    double duration;
    char cinematic_profile;      // 't' o 'q' trapezoidal o cartesian
};

// NOTA* : se keep_prev_orientation è true, allora Q viene ignorato e si mantiene 
// l'orientamento precedente (quello letto dalla fkine all'inizio del movimento)


class TaskOrchestrator : public rclcpp::Node
{
  public:
    /* Alias */ //DA MODIFICARE CON I TIPI DELLA NOSTRA AZIONE (CE NE SONO 2)

    //Azione MoveJointLin
    using MoveJointLinAct         = pacchetto_interfacce::action::MoveJointLin;
    using GoalHandle_MJL          = rclcpp_action::ClientGoalHandle<MoveJointLinAct>;
    using SendGoalOptions_MJL     = rclcpp_action::Client<MoveJointLinAct>::SendGoalOptions;
    using GoalHandlePtr_MJL       = GoalHandle_MJL::SharedPtr;
    using FeebackPtr_MJL          = std::shared_ptr<const MoveJointLinAct::Feedback>;
    using WrappedResult_MJL       = GoalHandle_MJL::WrappedResult;  
    using ShrdFuture_MJL          = std::shared_future<WrappedResult_MJL>;
    using Promise_MJL             = std::promise<WrappedResult_MJL>;
    using ClientPtr_MJL           = rclcpp_action::Client<MoveJointLinAct>::SharedPtr;

    // Azione MoveCartesianLin
    using MoveCartesianLinAct     = pacchetto_interfacce::action::MoveCartesianLin;
    using GoalHandle_MCL          = rclcpp_action::ClientGoalHandle<MoveCartesianLinAct>;
    using SendGoalOptions_MCL     = rclcpp_action::Client<MoveCartesianLinAct>::SendGoalOptions;
    using GoalHandlePtr_MCL       = GoalHandle_MCL::SharedPtr;
    using FeedbackPtr_MCL         = std::shared_ptr<const MoveCartesianLinAct::Feedback>;
    using WrappedResult_MCL       = GoalHandle_MCL::WrappedResult;
    using ShrdFuture_MCL          = std::shared_future<WrappedResult_MCL>;
    using Promise_MCL             = std::promise<WrappedResult_MCL>;
    using ClientPtr_MCL           = rclcpp_action::Client<MoveCartesianLinAct>::SharedPtr;

    // Servizio OnOff
    using OnOffSrv            = pacchetto_interfacce::srv::OnOff;
    using OnOffClientPtr      = rclcpp::Client<OnOffSrv>::SharedPtr;

    
    /* COSTRUTTORE */
    TaskOrchestrator(int argc, char ** argv) : Node("task_orchestrator")
    {
        //Action Client per MoveJointLin --> parla con joint_traj_generator che è l'action server 
        joint_action_client_ = rclcpp_action::create_client<MoveJointLinAct>(
            this, "move_joint_lin_action");
      

        // Action Client per l'azione cartesiana --> parla con cartesian_traj_generator che è l'action server 
        cartesian_action_client_ = rclcpp_action::create_client<MoveCartesianLinAct>(
            this, "cartesian_traj_action");


        // Service Client per il servizio on/off -> parla con clik_node che è il service server 
        clik_client_ = this->create_client<OnOffSrv>("clik_on_off");


        // Controlla che il path sia stato passato
        if (argc < 2) {
            RCLCPP_FATAL(this->get_logger(),
                "Uso: task_node <path_al_file_yaml>");
            throw std::runtime_error("path YAML mancante");
        }

        // Legge la configurazione — riempie mosse_
        mosse_ = leggi_config_yaml(argv[1]);

        RCLCPP_INFO(this->get_logger(), "Task Orchestrator istanziato.");
    }


    /* METODI PUBBLICI */


    /**
     * Metodo pubblico per inviare il movimento giunti.
     * Restituisce un std::shared_future che si sbloccherà SOLO quando il robot avrà finito il movimento.
     */
    ShrdFuture_MJL invia_movimento_giunti(const joint_config & q_desired, double duration, char cinematic_profile)
    {
        // Creiamo una promessa che verrà mantenuta quando l'azione sarà finita
        auto promise = std::make_shared<Promise_MJL>();
        ShrdFuture_MJL future_risultato = promise->get_future().share();


        //aspetto che l'action server sia pronto (con timeout di 5 secondi)
        if (!joint_action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Action Server 'move_joint_lin_action' non trovato!");
            WrappedResult_MJL errore_result;
            errore_result.code = rclcpp_action::ResultCode::UNKNOWN;
            promise->set_value(errore_result);
            return future_risultato;
        }


        // Riempio il Goal con i parametri passati al metodo
        auto goal_msg = MoveJointLinAct::Goal();
        goal_msg.q_desired = q_desired;
        goal_msg.duration = duration;
        goal_msg.cinematic_profile = cinematic_profile;
        

        // SendGoalOptions: struttura che contiene le 3 callback
        auto send_goal_options = SendGoalOptions_MJL();

         /*CALLBACK 1: goal_response_callback*/ 
        // Chiamata quando il server accetta o rifiuta il goal
        send_goal_options.goal_response_callback = [this](const GoalHandlePtr_MJL & goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal RIFIUTATO dal server!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal ACCETTATO, robot in movimento...");
            }
        };

        /* CALLBACK 2: feedback_callback*/
        // Chiamata ogni volta che il server pubblica un feedback
        send_goal_options.feedback_callback = [this](GoalHandlePtr_MJL, const FeebackPtr_MJL feedback) {
            RCLCPP_INFO(this->get_logger(), "Progresso: %.1f%%", feedback->progress * 100.0);
        };

        /* CALLBACK 3: result_callback */ 
        // quando server ha finito (successo/cancellazione/errore)
        send_goal_options.result_callback = [this, promise](const WrappedResult_MJL & result) {
            promise->set_value(result); // Questo sblocca il .wait() nel main!
        };


        //invio richiesta di movimento al server, con le callback definite sopra
        RCLCPP_INFO(this->get_logger(), "Invio richiesta movimento giunti...");
        joint_action_client_->async_send_goal(goal_msg, send_goal_options);


        return future_risultato;
    }

    /**
     * Chiama il servizio clik_on_off per accendere o spegnere il CLIK.
     * 
     * param accendi  true = accendi il clik, false = spegnilo
     * return         true se la chiamata è andata a buon fine
     */
    bool chiama_clik_on_off(bool accendi)
    {
        // 1. Aspettiamo che il server del servizio sia disponibile (timeout 5s)
        if (!clik_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Servizio 'clik_on_off' non trovato!");
            return false;
        }

        // 2. Prepariamo la request: è una struct con il campo set_on (bool)
        auto request = std::make_shared<OnOffSrv::Request>();
        request->set_on = accendi;   // true = ON, false = OFF

        // 3. Mandiamo la request in modo asincrono.
        //    async_send_request restituisce un future che si sblocca quando il server risponde.
        auto future_risposta = clik_client_->async_send_request(request);

        // 4. Aspettiamo la risposta con un timeout di 5 secondi.
        if (future_risposta.wait_for(5s) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Timeout in attesa risposta clik_on_off.");
            return false;
        }

        // 5. Leggiamo la risposta: is_on ci dice lo stato attuale del clik
        bool stato = future_risposta.get()->is_on;
        RCLCPP_INFO(this->get_logger(),
            "CLIK ora è: %s", stato ? "ON" : "OFF");

        return true;
    }

    /**
     * Manda un goal all'action server cartesian_traj_generator.
     *
     * x, y, z   posizione target dell'end-effector
     * Quaternion Q    orientamento target dell'end-effector (versione avanzata)*
     * keep_prev_orientation  se true, ignora Q e mantieni l'orientamento precedente (versione avanzata)*
     * duration  durata del movimento in secondi
     */
    ShrdFuture_MCL invia_movimento_cartesiano(
        double x, double y, double z, Quaternion Q, bool keep_prev_orientation, double duration, char cinematic_profile)
    {
        // --- Preparazione promise/future ---
        auto promise = std::make_shared<Promise_MCL>();
        ShrdFuture_MCL future_risultato = promise->get_future().share();

        // Aspettiamo che l'action server sia pronto
        if (!cartesian_action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(),
                "Action Server 'cartesian_traj_action' non trovato!");
            WrappedResult_MCL errore_result;
            errore_result.code = rclcpp_action::ResultCode::UNKNOWN;
            promise->set_value(errore_result);
            return future_risultato;
        }

        // --- Costruiamo il Goal ---
        //   geometry_msgs/Pose pose_desired   → posizione + orientamento
        //   float64 duration
        auto goal_msg = MoveCartesianLinAct::Goal();

        goal_msg.pose_desired.position.x = x;
        goal_msg.pose_desired.position.y = y;
        goal_msg.pose_desired.position.z = z;

        goal_msg.pose_desired.orientation.w = Q.w();
        goal_msg.pose_desired.orientation.x = Q.x();
        goal_msg.pose_desired.orientation.y = Q.y();
        goal_msg.pose_desired.orientation.z = Q.z();

        goal_msg.keep_prev_orientation = keep_prev_orientation;   

        goal_msg.duration = duration;

        goal_msg.cinematic_profile = cinematic_profile;
        

        
        // --- Le 3 callback ---
        auto send_goal_options = SendGoalOptions_MCL();

        // CALLBACK 1: risposta del server al goal (accettato/rifiutato)
        send_goal_options.goal_response_callback =
            [this](const GoalHandlePtr_MCL & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(),
                        "Goal cartesiano RIFIUTATO dal server!");
                } else {
                    RCLCPP_INFO(this->get_logger(),
                        "Goal cartesiano ACCETTATO, robot in movimento...");
                }
            };

        // CALLBACK 2: feedback intermedio pubblicato da cartesian_traj_generator
        // Il feedback ha: pose_current (Pose) e progress
        send_goal_options.feedback_callback =
            [this](GoalHandlePtr_MCL, const FeedbackPtr_MCL feedback) {
                RCLCPP_INFO(this->get_logger(),
                    "Progresso cartesiano: %.1f%% — p=[%.3f, %.3f, %.3f]",
                    feedback->progress * 100.0,
                    feedback->pose_current.position.x,
                    feedback->pose_current.position.y,
                    feedback->pose_current.position.z);
            };

        // CALLBACK 3: risultato finale → sblocca il future nel main
        send_goal_options.result_callback =
            [this, promise](const WrappedResult_MCL & result) {
                // Appena arriva il risultato, lo mettiamo nella promise.
                // Questo fa sì che il .wait() nel main si sblocchi.
                promise->set_value(result);
            };

        // --- Mandiamo il goal ---
        RCLCPP_INFO(this->get_logger(),
            "Invio goal cartesiano: target=[%.3f, %.3f, %.3f], duration=%.1f s",
            x, y, z, duration);
        cartesian_action_client_->async_send_goal(goal_msg, send_goal_options);

        return future_risultato;
    }

    //altri metodi

    // Getter per la lista delle mosse (non modificabile)
    const std::vector<Move> & get_mosse() const { return mosse_; }

    // Metodo di utilità per stampare un messaggio e aspettare l'input da terminale
    void print_and_wait(const std::string & message)
    {
        // Stampa il messaggio sul logger ROS2
        RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
        RCLCPP_INFO(this->get_logger(), "Premi un tasto e INVIO per continuare...");

        std::cin >> c_in;   // blocca finché l'utente non digita qualcosa e preme INVIO
        std::cin.ignore(); // pulisce il '\n' rimasto nel buffer
    }


  private:
    /* attributi privati */

    // sottocomponenti
    ClientPtr_MJL joint_action_client_;         //action client per MoveJointLin
    ClientPtr_MCL   cartesian_action_client_;   // action client per MoveCartesianLin
    OnOffClientPtr  clik_client_;               // service client per servizio on/off clik

    //movimenti da eseguire
    std::vector<Move> mosse_;   // lista dei movimenti letti dal file YAML

    char c_in;      //input da terminale per andare avanti


    /* metodi privati*/

    // Legge il file YAML al path indicato e restituisce la lista dei movimenti.
    // Chiamato una volta sola nel costruttore.
    std::vector<Move> leggi_config_yaml(const std::string & path)
    {
        std::vector<Move> risultato;

        // Apre e parsa il file — lancia eccezione se il file non esiste
        YAML::Node config;
        try {
            config = YAML::LoadFile(path);
        }
        catch (const YAML::Exception & e) {
            RCLCPP_FATAL(this->get_logger(),
                "Impossibile leggere il file YAML: %s\nErrore: %s",
                path.c_str(), e.what());
            throw;  // rilancia — il nodo non può partire senza config
        }

        // Itera sulla lista "moves" nel YAML
        for (const auto & node : config["moves"])
        {
            Move m;

            // accetta: "j", "J",  "c", "C"
            char move_type = node["type"].as<char>();
            
            if (move_type == 'j' || move_type == 'J') {
                m.type = 'j';

                // Legge i 7 valori di q_desired
                for (const auto & val : node["q_desired"]) {
                    m.q_desired.push_back(val.as<double>());
                }

                // Validazione: il Panda ha esattamente 7 giunti
                if (m.q_desired.size() != N_JOINTS) {
                    RCLCPP_FATAL(this->get_logger(),
                        "q_desired deve avere %d valori, trovati: %zu",
                        N_JOINTS, m.q_desired.size());
                    throw std::runtime_error("q_desired size errata");
                }

                RCLCPP_INFO(this->get_logger(),
                    "Mossa %zu: JOINT — duration=%.1f, q=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    risultato.size(), m.duration,
                    m.q_desired[0], m.q_desired[1], m.q_desired[2], m.q_desired[3],
                    m.q_desired[4], m.q_desired[5], m.q_desired[6]);
            }
            else if (move_type == 'c' || move_type == 'C') {
                m.type = 'c';
                m.x = node["position"]["x"].as<double>();
                m.y = node["position"]["y"].as<double>();
                m.z = node["position"]["z"].as<double>();


                // Leggi il flag keep_orientation — default: false se non presente nel YAML
                m.keep_prev_orientation = node["keep_previous_orientation"] ?
                    node["keep_previous_orientation"].as<bool>() : false;

                
                if (m.keep_prev_orientation)
                {
                    // Il quaternione non verrà usato, ma inizializziamo a valori puliti per evitare problemi di valori spazzatura

                    m.Q = Quaternion(1.0, 0.0, 0.0, 0.0);   //identità, ma sarà ignorato dal server dell'azione cartesiana
                    RCLCPP_INFO(this->get_logger(), "Mossa cartesiana: orientamento mantenuto costante.");
                } 
                else 
                {
                    // Leggi il quaternione desiderato — obbligatorio se keep_prev_orientation=false

                    if (!node["orientation"]) { 
                        // se il campo "orientation" non è presente, è un errore perché keep_prev_orientation è false
                        RCLCPP_FATAL(this->get_logger(),
                            "Mossa cartesiana con orientazione desiderata, ma mancante nel YAML!");
                        throw std::runtime_error("orientation mancante");
                    }
                    double qw, qx, qy, qz;   

                    qw = node["orientation"]["w"].as<double>();
                    qx = node["orientation"]["x"].as<double>();
                    qy = node["orientation"]["y"].as<double>();
                    qz = node["orientation"]["z"].as<double>();

                    m.Q = Quaternion(qw, qx, qy, qz);
                }

                
                //plot risultato lettura
                 RCLCPP_INFO(this->get_logger(),
                    "Mossa %zu: CARTESIAN — duration=%.1f, pos=[%.3f, %.3f, %.3f], Q = [%.3f, %.3f, %.3f, %.3f]",
                    risultato.size(), m.duration, m.x, m.y, m.z, m.Q.w(), m.Q.x(), m.Q.y(), m.Q.z());
            }
            else    // Tipo char sconosciuto fermarsi 
            {
                RCLCPP_FATAL(this->get_logger(),
                    "Tipo di mossa sconosciuto: Usa 'j'/'J' o 'c'/'C'");
                throw std::runtime_error("tipo mossa sconosciuto");
            }

            //salvo durata
            m.duration = node["duration"].as<double>();


            //salvo profilo cinematico
            char cinematic_profile = node["cinematic_profile"].as<char>();   // accetta: "t" / "T" o "q" / "Q"

            if(cinematic_profile == 't' || cinematic_profile == 'T')
                m.cinematic_profile = 't';
            else if(cinematic_profile == 'q' || cinematic_profile == 'Q')
                m.cinematic_profile = 'q';
            else    // Tipo char sconosciuto fermarsi 
            {
                RCLCPP_FATAL(this->get_logger(),
                    "Tipo di profilo cinematico sconosciuto: Usa 't'/'T' o 'q'/'Q'");
                throw std::runtime_error("tipo profilo cinematico sconosciuto");
            }

            //salvo la mossa
            risultato.push_back(m);
        }

        RCLCPP_INFO(this->get_logger(),
            "Lette %zu mosse dal file: %s", risultato.size(), path.c_str());

        return risultato;
    }

};




int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    //  Creiamo l'oggetto della classe
    auto orchestrator = std::make_shared<TaskOrchestrator>(argc, argv);
    
    // Avviamo lo spin in background per far funzionare le comunicazioni della classe
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(orchestrator);
    std::thread executor_thread([&executor]() { executor.spin(); });

    RCLCPP_INFO(orchestrator->get_logger(), "=== START ===");


    // Ciclo su tutte le mosse lette dal file YAML
    for (const auto & mossa : orchestrator->get_mosse())
    {
        RCLCPP_INFO(orchestrator->get_logger(),
            "Mossa di tipo: %c\nDurata: %.2f\nCon profilo: %c", 
            mossa.type, mossa.duration, mossa.cinematic_profile);


        if (mossa.type == 'j' || mossa.type == 'J')
        {
            // ---- MOVIMENTO GIUNTI ----
            orchestrator->print_and_wait("Avvia movimento giunti?");

            auto future = orchestrator->invia_movimento_giunti(
                mossa.q_desired, 
                mossa.duration, 
                mossa.cinematic_profile
            );


            future.wait();

            if (future.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_ERROR(orchestrator->get_logger(),
                    "Movimento giunti fallito. Interrompo.");
                goto shutdown_sequence;
            }
            RCLCPP_INFO(orchestrator->get_logger(), "Movimento giunti completato.");

        }
        else if (mossa.type == 'c' || mossa.type == 'C')
        {
            // ---- MOVIMENTO CARTESIANO ----
            orchestrator->print_and_wait("Avvia movimento cartesiano? (+ accensione clik)");

            // Accendi il CLIK prima di mandare il goal
            if (!orchestrator->chiama_clik_on_off(true)) {
                RCLCPP_ERROR(orchestrator->get_logger(),
                    "Impossibile accendere il CLIK. Interrompo.");
                goto shutdown_sequence;
            }

            auto future = orchestrator->invia_movimento_cartesiano(
                mossa.x, mossa.y, mossa.z,
                mossa.Q,
                mossa.keep_prev_orientation,        
                mossa.duration,
                mossa.cinematic_profile
            );

            future.wait();

            // Spegni sempre il CLIK, anche se il movimento è fallito
            orchestrator->chiama_clik_on_off(false);

            if (future.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_ERROR(orchestrator->get_logger(),
                    "Movimento cartesiano fallito. Interrompo.");
                goto shutdown_sequence;
            }
            RCLCPP_INFO(orchestrator->get_logger(), "Movimento cartesiano completato. (+ spegnimento clik)");
        }
    }

    RCLCPP_INFO(orchestrator->get_logger(), "=== TUTTE LE MOSSE COMPLETATE ===");



    /* FASE DI CHIUSURA PULITA*/

shutdown_sequence:  //etichetta per il goto in caso di errore

    //chiusura pulita: fermo lo spin, distruggo l'oggetto, shutdown di ROS
    RCLCPP_INFO(orchestrator->get_logger(), "Chiusura del programma.");
    executor.cancel();
    if (executor_thread.joinable()) {
        executor_thread.join();
    }
    rclcpp::shutdown();
    return 0;
}