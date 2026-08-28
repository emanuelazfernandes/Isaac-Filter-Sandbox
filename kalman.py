def kalman_filter(mean_t_1, covariance_t_1, u_t, z_t):
    """
    mean_t_1       - Previous estimate / Estimativa de estado anterior
    covariance_t_1 - Previous covariance estimate / Estimativa de covariância anterior
    u_t            - Control input / Entrada de controlo
    z_t            - Measurement (Observation) / Medição (Observação)
    """
    # State transition matrix (x_t, mean_t) - Matriz de transição de estado
    A_t = [[1, 2], [3, 4]]
    # Control input matrix - Matriz de entrada de controlo
    B_t = [[5, 6], [7, 8]]
    # Observation matrix - Matriz de observação
    C_t = [[9, 10], [11, 12]]

    # Process noise covariance - Matriz de covariância do ruído da dinâmica (processo)
    Q_t = [[17, 18], [19, 20]]
    # Measurement/observation noise covariance - Matriz de covariância do ruído das observações
    R_t = [[13, 14], [15, 16]]

    # Assuming these helper functions are defined elsewhere
    I = identity_matrix(2, 2)
    A_t_transp = transpose(A_t)
    C_t_transp = transpose(C_t)

    ## Filter equations
    
    # 1 - Prediction step - Predição
    # Prediction estimate
    mean_t_est = A_t * mean_t_1 + B_t * u_t
    
    # Prediction covariance - Matriz de covariância do erro de predição
    covariance_t_est = A_t * covariance_t_1 * A_t_transp + Q_t

    # 2 - Update step - Atualização (Filtragem)
    # Kalman gain update - Matriz de ganho de Kalman
    K_t_aux = C_t * covariance_t_est * C_t_transp + R_t
    K_t_aux_inv = matrix_invert(K_t_aux)
    K_t = covariance_t_est * C_t_transp * K_t_aux_inv
    
    # Update estimate
    mean_t = mean_t_est + K_t * (z_t - C_t * mean_t_est)
    
    # Update covariance
    covariance_t = (I - K_t * C_t) * covariance_t_est
    
    return mean_t, covariance_t