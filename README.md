# Isaac-Filter-Sandbox

Estimation filters implementation in Nvidia Isaac Sim

---

## Kalman Filter

Implementation of Kalman Filter from scratch

### SCT cap. 4 vs. Probabilistic Robotics chap. 3

SCT:
- **5.2: Estimação de máxima verosimilhança** - Dado um parâmetro $\theta$ e observações ruidosas do parâmetro (`z`), a estimativa do parâmetro `$\^{\theta}$` obtém-se usando um *estimador*.
- **5.3: Medidas da qualidade dos estimadores** - Média com enviesamento: `E{theta_til} = theta + b`, `b` - *bias*.
- **5.5: Estimadores de variância mínima linear** - O método da variância mínima linear baseia-se no 1.º (*mean*) e 2.º (*covariance*) momentos estatístico do parâmetro e das perturbações. Consideremos estimadores lineares da forma `\theta_til = b + A*z` (estimador é combinação linear pesada das observações). As observações são uma função linear de `\theta` da forma: `z = H*\theta + n`, H - matriz das observações, n - perturbações do tipo ruído branco gaussiano. -> `\theta_til_lmv = (H^T * H)^-1 * H^T * z`, *lmv - linear minimum variance*.
- **6.1: Estimação recursiva** - Forma recursiva de um estimador linear. Não é necessário guardar as observações passadas para calcular as estimativas presentes.
- **6.2: Filtro de Kalman discreto** - Solução recursiva para o problema da estimação linear. 2 passos: 1-predição (estima-se **x_k** usando o conjunto de observações passadas **Z_k_1** = {**z0**, **z1**, ..., **zk_1**}) e 2-filtragem (estima-se **x_k** baseado na estimativa de predição e na presente observação **z_k**).


Forma recursiva de um estimador


