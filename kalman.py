import numpy as np

def kalman_filter(mean_t_1, covariance_t_1, u_t, z_t):
    """
    mean_t_1       - previous estimate
    covariance_t_1 - previous covariance estimate
    u_t            - control input
    z_t            - measurement/observation
    """

    # State transition matrix (x_t, mean_t) - Matriz de transicao de estado
    A_t = np.matrix([[1, 2], [3, 4]])
    # Control input matrix - Matriz de entrada de controlo
    B_t = np.matrix([[5, 6], [7, 8]])
    # Observation matrix - Matriz das observacoes
    C_t = np.matrix([[9, 10], [11, 12]])

    # Process noise covariance - Matriz covariancia do ruido da dinamica (processo)
    Q_t = np.matrix([[17, 18], [19, 20]])
    # Measurement/observation noise covariance - Matriz covariancia do ruido das observacoes
    R_t = np.matrix([[13, 14], [15, 16]])

    I = np.identity(2)
    A_t_transp = np.matrix.transpose(A_t)
    C_t_transp = np.matrix.transpose(C_t)

    ## Filter equations

    # 1 - Prediction step - predicao
    # Prediction estimate
    mean_t_est = A_t * mean_t_1 + B_t * u_t
    
    # Prediction covariance - Matriz covariancia do erro de predicao
    covariance_t_est = A_t * covariance_t_1 * A_t_transp + Q_t

    # 2 - Update step - filtragem
    # Kalman gain update - Matriz ganho de Kalman
    K_t_aux = C_t * covariance_t_est * C_t_transp + R_t
    K_t_aux_inv = np.linalg.inv(K_t_aux)
    K_t = covariance_t_est * C_t_transp * K_t_aux_inv
    
    # Update estimate
    mean_t = mean_t_est + K_t * (z_t - C_t * mean_t_est)
    
    # Update covariance
    covariance_t = (I - K_t * C_t) * covariance_t_est
    
    return mean_t, covariance_t