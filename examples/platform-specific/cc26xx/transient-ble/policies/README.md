# Profile pools

* `pool_deterministic.pkl`: profile pool with receiver active for 60s. Power levels with deltas of [2, 3, 4, 5, 6] atus 

* `pool_deterministic2.pkl`: profile pool with receiver active for 60s. Power levels with deltas of [2, 4, 6, 8, 10] atus 

* `pool_stochastic.pkl`: profile pool with receiver active time following a binomial distribution with a mean of 15s. power levels with deltas of [2, 3, 4, 5, 6] atus

* `pool_stochastic2.pkl`: profile pool with receiver active time following a bimodal distribution: total prob of 0.5 for [2, 5) minutes, total probability of 0.5 for [30, 45) minutes.  Power levels with deltas of [2, 4, 6, 8, 10] atus. 'power' entries of the profile pool dict is incorrect. Most likely need to be divided by 20

* `pool_stochastic2a.pkl`: `pool_stochastic2.pkl` with scaled power levels

* `pool_stochastic3.pkl`: profile pool with receiver active time following a binomial distribution with a mean of 15s. power levels with deltas of [2, 4, 6, 8, 10] atus

* `pool_synthetic.pkl`: profile pool with receiver active for 60s. power levels with deltas of [2, 4, 6, 8, 10] atus
