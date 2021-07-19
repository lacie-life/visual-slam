# -*- coding: utf-8 -*-
"""
Particle Filter
(IMUPF model version)

Difference between IMUPF and IMUPF2:
IMUPF -> IMU data is regarded as observation
IMUPF2 -> IMU data is regarded as control
"""

import numpy
import copy
import math

class ParticleFilterIMU:

	def __init__(self):
		self.var_sys = 1.0
		self.var_obs = 1.0
		
	def setParameter(self, param1, param2):
		self.var_sys = param1 * param2
		self.var_obs = param1
	

	def f(self, dt, X):
		""" Transition model
		- 状態方程式
			x_t = f(x_t-1) + w
			w ~ N(0, sigma)
		"""

		w_mean = numpy.zeros(3) # mean of noise
		w_cov_a = numpy.eye(3) * self.var_sys # covariance matrix of noise (accel)
		w_a = numpy.random.multivariate_normal(w_mean, w_cov_a) # generate random
		w_cov_o = numpy.eye(3) * self.var_sys # covariance matrix of noise (ori)
		w_o = numpy.random.multivariate_normal(w_mean, w_cov_o) # generate random

		dt2 = dt*dt
		dt3 = dt*dt*dt

		X_new = copy.deepcopy(X)
		X_new.x = X.x + dt*X.v + 0.5*dt2*X.a + 0.166666*dt3*w_a
		X_new.v = X.v + dt*X.a + 0.5*dt2*w_a
		X_new.a = X.a + dt*w_a
		X_new.o = X.o + dt*w_o

		return X_new
					

	def likelihood(self, accel, ori, X):
		""" Likelihood function
		- 尤度関数
			p(y|x) ~ exp(-1/2 * (|y-h(x)|.t * sigma * |y-h(x)|)
		- 観測モデル
			z = h(x) + v
			v ~ N(0, sigma)
		Parameters
		----------
		accel : 観測 (加速度) Observation set (accel)
		ori : 観測 (姿勢) Observation set (orientation)
		X : 予測　Predicted particle
		Returns
		-------
		likelihood : 尤度 Likelihood
		"""
		sigma_a_inv = 1.0/self.var_obs # 1.0 / sigma_a
		sigma_o_inv = 1.0/self.var_obs # 1.0 / sigma_o
		v_cov_a = numpy.eye(3) * sigma_a_inv # inv of covariance matrix
		v_cov_o = numpy.eye(3) * sigma_o_inv # inv of covariance matrix

		y_X_a = accel - X.a # y - X (accel)
		y_X_o = ori - X.o # y - X (orientation)
					
		return numpy.exp(-0.5 * (numpy.dot(y_X_a, numpy.dot(v_cov_a, y_X_a)) + numpy.dot(y_X_o, numpy.dot(v_cov_o, y_X_o)))) / (((2*math.pi)**3) * math.sqrt(self.var_obs*6))
		#return numpy.exp(-0.5 * (numpy.dot(y_X_a, numpy.dot(v_cov_a, y_X_a)) + numpy.dot(y_X_o, numpy.dot(v_cov_o, y_X_o))))
					

	def resampling(self, X, W, M):
		""" Resampling
		- 等間隔サンプリング
		M : パーティクルの数 num of particles
		"""
		X_resampled = []
		Minv = 1/float(M)
		r = numpy.random.rand() * Minv
		c = W[0]
		i = 0
		for m in range(M):
			U = r + m * Minv
			while U > c:
				i += 1
				c += W[i]
			X_resampled.append(X[i])
		return X_resampled
					

	def pf_step(self, X, dt, accel, ori, M):
		""" One Step of Sampling Importance Resampling for Particle Filter
			for IMU sensor
		Parameters
		----------
		X : 状態 List of state set
		dt: 時刻の差分 delta of time
		accel : 観測 (加速度) Observation set (accel)
		ori : 観測 (姿勢) Observation set (orientation)
		M : パーティクルの数 num of particles
		Returns
		-------
		X_resampled : 次の状態 List updated state
		"""

		# 初期化
		X_predicted = range(M)
		weight = range(M)

		for i in range(M):
			# 推定 prediction
			X_predicted[i] = self.f(dt, X[i])
			# 更新 update
			weight[i] = self.likelihood(accel, ori, X_predicted[i])
		# 正規化 normalization
		weight_sum = sum(weight) # 総和 the sum of weights
		for i in range(M):
			weight[i] /= weight_sum
		# リサンプリング re-sampling (if necessary)
		X_resampled = self.resampling(X_predicted, weight, M)

		return X_resampled
