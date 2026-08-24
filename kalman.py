def kalman_filter(mean_t_1, covariance_t_1, u_t, z_t):
    """
    mean_t_1 - Previous estimate
    covariance_t_1 - Previous covariance estimate
    u_t - 
    z_t - 
    """
    # State matrix (x_t, mean_t)
    A_t = [[1, 2], [3, 4]]
    # 
    B_t = [[5, 6], [7, 8]]
    #  - P(k|k_1)
    C_t = [[9, 10], [11, 12]]

    # 
    R_t = [[13, 14], [15, 16]]
    # 
    Q_t = [[17, 18], [19, 20]]

    I = identity_matrix(2, 2)

    A_t_transp = transpose(A_t)
    C_t_transp = transpose(C_t)




    ## Filter equations
    # 1 - Prediction step - predição
    # Prediction estimate
    mean_t_est = A_t * mean_t_1 + B_t * u_t
    # Prediction covariance
    covariance_t_est = A_t * covariance_t_1 * A_t_transp + R_t

    # 2 - Update step - filtragem
    # Kalman gain update
    K_t_aux = [C_t * covariance_t_est * C_t_transp + Q_t]
    K_t_aux_inv = matrix_invert(K_t_aux)
    K_t = covariance_t_est * C_t_transp * K_t_aux_inv
    # Update estimate
    mean_t = mean_t_est + K_t * (z_t - C_t * mean_t_est)
    # Update covariance
    covariance_t = (I - K_t * C_t) * covariance_t_est



    return mean_t, covariance_t