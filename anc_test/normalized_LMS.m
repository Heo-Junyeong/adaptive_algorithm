%-------------------------------------------------------------------------------
% Normalized_LMS algorithms demo
%
% Developed by Junyeong Heo
%-------------------------------------------------------------------------------

clear

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
%  - N : filter tap size (== filter order)
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

L = 7;
N = 10000;

x = randn(1,N);
h = [0.921 0.107 0.336 1 0.336 0.107 0.921];
d = filter(h,1,x);

[w, e] = normalized_LMS(x, d, L);

figure, plot(e)
title('Error');
xlabel('Sample');
##saveas(gcf,'e1.png');
figure, stem(w)
title('Adaptive filter weights');

