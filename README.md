# Isaac-Filter-Sandbox

Estimation filters implementation in Nvidia Isaac Sim

---

## Kalman Filter

Implementation of Kalman Filter from scratch

### SCT cap. 4 vs. Probabilistic Robotics chap. 3

#### SCT:
- **5.2: Estimação de máxima verosimilhança** - Dado um parâmetro $\theta$ e observações ruidosas do parâmetro `z`, a estimativa do parâmetro $\hat{\theta}$ obtém-se usando um *estimador*.
- **5.3: Medidas da qualidade dos estimadores** - Média com enviesamento: $E\lbrace\hat{\theta}\rbrace = \theta + b$, $b$ - *bias*.
- **5.5: Estimadores de variância mínima linear** - O método da variância mínima linear baseia-se no 1.º (*mean*) e 2.º (*covariance*) momentos estatístico do parâmetro e das perturbações. Consideremos estimadores lineares da forma $\hat{\theta} = b + A*z$ (estimador é combinação linear pesada das observações). As observações são uma função linear de $\theta$ da forma: $z = H*\theta + n$, H - matriz das observações, n - perturbações do tipo ruído branco gaussiano. $\hat{\theta}_{lmv} = (H^T H)^{-1} H^T z$, *lmv - linear minimum variance*.
- **6.1: Estimação recursiva** - Forma recursiva de um estimador linear. Não é necessário guardar as observações passadas para calcular as estimativas presentes.
- **6.2: Filtro de Kalman discreto** - Solução recursiva para o problema da estimação linear. 2 passos: 1-*predição* (estima-se **x_k** usando o conjunto de observações passadas $Z_{k-1} = \lbrace z_0, z_1, ..., z_{k-1}\rbrace$) e 2-*filtragem* (estima-se $x_k$ baseado na estimativa de predição e na presente observação $z_k$).


Forma recursiva de um estimador

#### PR - chapter 3



## Markov Chain

## Alpha-Beta Filter

## IMM Filter


---

# GNC - Guidance, Navigation and Control

Intro

## Guidance

Intro

### PN - Proportional Navigation

Pure Proportional Navigation

$ \mathbf{a}_{PN} = N_PN \mathbf{v}_I \times {\omega}_{LOS} $



### Augmented PN


## Navigation

Intro

### EKF - Extended Kalman Filter

### Particle Filter


## Control

Intro

### PID - Proportional, Integrative and Derivative Controller

### LQR - Optimal Control (Least Squares)

### MPC - Model Predictive Control

### TVC - Thrust Vectoring Control

---

# MDP - Markov Decision Processes

Intro

## POMDP - Partial Observable MDP

## RL - Reinforcement Learning

---

## Navigation - Extended

### IMM - Interacting Multiple Model

Intro

The IMM algorithm implements multiple estimation filters under a Markov Chain paradigm. Each estimation model under a Markov state assumes a dynamical model, and the final estimate for the noisy observation is a combined weighted estimation of the probability of the underlying models.

#### State of the Art - IMM

##### RFS - Random Finite Sets

Intro

PMBM (Poisson Multi-Bernoulli Mixture filter), bypass traditional heuristics data association.

##### Deep RL and Transformer-Based Tracking

MT3 - Multi-Target Tracking Transformer

DeepAF - Deep data Association and track Filtering network

---

# References

<a id="1">[1]</a> 
Moreira, M., Papp, E., Ventura, R. "Interception of non-cooperative UAVs." 2019 IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR). IEEE, 2019.

<a id="2">[2]</a> 
Thrun, S., Burgard, W., Fox, D. "Probabilistic Robotics" MIT Press, 2005

<a id="3">[3]</a> 
Nunes, F. "Sistemas de Controlo de Trafego: Apontamentos da Cadeira" IST, 2010

<a id="4">[4]</a> 
Bar-Shalom, Y., Li, X.R., Kirubarajan, T. "Estimation with Applications to Tracking and Navigation: Theory Algorithms and Software" John Wiley & Sons, 2004
