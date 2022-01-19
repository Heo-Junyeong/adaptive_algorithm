%--------------------------------------------------------------------------
% Let's start the code :)
%
% Developed by Heo Junyeong Heo
%--------------------------------------------------------------------------
clear;

function [w, e] = normalized_LMS(x, d, N, eps, beta)
% ##############################################################################
% This function generate adaptive weights by NLMS(normalized Least Mean Square) algorithms
%
% -NLMS Update equation
%  init condition w_i(0) = 0, p_x(0) = 1, i = [0, 1, 2, ... , L-1] 
%  w_i(n + 1) = w_i(n) + ((1-Beta)/(p_x(k) + delta)) * e(n) * x(n-i)
%  p_x(n) = {
%  (0 < Beta < 1) , Beta*(p_x(n-1) + (1-Beta) * x^2(n))
%  otherwise , eps
%  }
% Usage:
%  [weights, error] = normalized_LMS(reference_signal, eps, beta)
%
% Parameters:
%  - x : reference_signal(input_signal)
%  - d : desired signal(wanted signal)
%  - N : filter tap size
%  - eps : epsilon, its scale may depend on the power of the input signal, i.e., eps = 1.e-6 * (input power)
%  - beta : 0 < beta < 1
%
% Output :
%  - w : weights of adaptive filter
%  - e : error signal
% ##############################################################################

  % set defualt parameters
  if nargin < 5,
    beta = 0.999;  % 0 < beta < 1
  end
  if nargin < 4,
    eps = 1e-6;  % a small conastant as 1.e-6
  end
  
L = length(x);
w = zeros(1, N);
e = zeros(1, L);

p_x = 1;

  % NLMS algorithms
  for n = N : L,
    x_vect = x(n : -1 : n - N + 1);
    e(n) = d(n) - w * x_vect';
    w = w + ((1 - beta)/p_x) * e(n) * x_vect;
    p_x = (beta * p_x + (1 - beta)*(x(n)^2)) * (p_x > eps) + (p_x <= eps) * eps;
  end
end

%set simulation parameter
primary_system_impulse = [0.01 0.25 0.5 1 0.5 0.25 0.01];
secondary_system_impulse = 0.25 * primary_system_impulse; % arbitary value

mu_secondary = 0.1; % LMS parameter (for secondary path estimation)

L_secondary_filter = 16; % tap size of secondary system
L_control_filter = 16; % tap size of control system

T_secondary_size = 10000; % input samples size of secondary path estimation
T_control_filter = 10000; % input samples size of control system estimation

%%%----------------------------------------------------------------
% secondary path estimation
Sw = zeros(1, L_secondary_filter);
Sx = zeros(1, L_secondary_filter);

x_iden = rand(1, T_secondary_size);

y_secondary_iden = filter(Sw, 1, x_iden);

