#pragma once   // evita inclusioni doppie — alternativa moderna agli include guard

#include <Eigen/Dense>

//Alias
using Vector3d = Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using VectorXd = Eigen::VectorXd;                    //generic Vector dinamico
using MatrixXd = Eigen::MatrixXd;                    //generic Matrix dinamica
using Quaternion = Eigen::Quaterniond;
using RotationAxis = Eigen::AngleAxisd;              //rotazione attorno ad un asse (usato per definire quaternioni)
using RotoTraslMatrix = Eigen::Isometry3d;


//assi per rotazioni
const Vector3d X_AXIS= Eigen::Vector3d::UnitX();
const Vector3d Y_AXIS = Eigen::Vector3d::UnitY();
const Vector3d Z_AXIS = Eigen::Vector3d::UnitZ();


//funzioni
namespace eigen_utils 
{
    /**
     * @brief Calcola il minimo valore singolare di una matrice generica (es. lo Jacobiano).
     * @details Utilizza l'algoritmo BDCSVD di Eigen, ottimizzato per matrici di medie/grandi dimensioni,
     * ma eccellente anche per matrici di robot seriali (es. 6x6 o 6x7).
     * @param M La matrice di ingresso di cui calcolare la SVD.
     * @return double Il valore singolare più piccolo (sigma_min). Ritorna 0.0 se la matrice è vuota.
     */
    inline double compute_minimum_singular_value(const MatrixXd& M)
    {
        // Controllo di sicurezza: se la matrice non è inizializzata, evitiamo crash
        if (M.rows() == 0 || M.cols() == 0) {
            return 0.0;
        }

        // Inizializziamo l'algoritmo BDC-SVD. 
        // Poiché ci servono solo i valori singolari (e non i vettori U e V),
        // specifichiamo 0 come flag per risparmiare tempo di calcolo
        Eigen::BDCSVD<MatrixXd> svd(M, 0);

        // singularValues() restituisce un vettore con i valori singolari in ordine decrescente.
        // minCoeff() estrae l'ultimo elemento (il più piccolo).
        double sigma_min = svd.singularValues().minCoeff();

        return sigma_min;
    }


    /**
     * @brief Risolve il sistema A * x = B calcolando la pseudo-inversa di A tramite decomposizione COD.
     * @details Ottimizzato per implementare la legge cinematica differenziale: q_dot = J^# * (k * e).
     * È estremamente robusto contro le matrici singolari o non quadrate (es. robot ridondanti a 7 DOFs).
     * @param A La matrice dei coefficienti (es. lo Jacobiano J, di dimensione m x n).
     * @param B Il vettore dei termini noti (es. il vettore errore scalato k * e, di dimensione m x 1).
     * @return Eigen::VectorXd Il vettore soluzione (es. le velocità dei giunti q_dot, di dimensione n x 1).
     */
    inline VectorXd computePinvAB(const MatrixXd& A, const VectorXd& B)
    {
        // Controllo di coerenza dimensionale tra matrice e vettore per evitare crash a runtime
        if (A.rows() != B.rows()) {
            // Se le dimensioni sono incompatibili, ritorniamo un vettore vuoto
            return VectorXd();
        }

        // Il metodo .solve() calcola implicitamente x = A^# * B sfruttando la decomposizione ortogonale completa
        VectorXd x = A.completeOrthogonalDecomposition().solve(B);
        
        return x;
    }


    /**
     * @brief Garantisce la continuità del quaternione tra iterazioni successive per evitare salti di segno.
     * @details I quaternioni Q e -Q rappresentano la stessa identica rotazione 3D. Questa funzione
     * verifica il prodotto scalare globale (4D) e inverte il segno se i quaternioni puntano in direzioni opposte,
     * garantendo il percorso geodetico minimo (percorso più breve).
     * @param Q_k Il quaternione calcolato all'iterazione corrente.
     * @param Q_k_minus_1 Il quaternione dell'iterazione precedente.
     * @return Eigen::Quaterniond Il quaternione normalizzato e con il segno corretto.
     */
    inline Quaternion quaternionContinuity(const Quaternion& Q_k, const Quaternion& Q_k_minus_1)
    {
        // 1. Calcola il prodotto scalare globale 4D (x*x + y*y + z*z + w*w)
        double dot = Q_k.dot(Q_k_minus_1);

        // 2. Se il prodotto scalare è negativo, significa che i due quaternioni si trovano
        // su emisferi opposti della sfera unitaria (ipersfera 4D).
        if (dot < 0.0) {
            // Ritorniamo il quaternione invertito in tutte le sue componenti
            return Quaternion(-Q_k.w(), -Q_k.x(), -Q_k.y(), -Q_k.z());
        }
        
        return Q_k;
    }
}