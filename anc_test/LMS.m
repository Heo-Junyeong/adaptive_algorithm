%-------------------------------------------------------------------------------
% LMS algorithms demo
%
% Developed by Junyeong Heo
%-------------------------------------------------------------------------------
L = 7;
N = 10000;

x = randn(1,N);
h = [0.921 0.107 0.336 1 0.336 0.107 0.921];
d = filter(h,1,x);

function [w, e] = LMS(x, d, N, mu)
% ##############################################################################
% This function generate adaptive weights by LMS(Least Mean Square) algorithms
%
% -LMS Update equation
%  init condition w_i(0) = 0, i = [0, 1, 2, ... , L-1] 
%  w_i(n) = w_i(n-1) + mu * e(n) * x(n)
%
% Usage:
%  [weights, error] = LMS(reference_signal, mu)
%
% Parameters:
%  - x : reference_signal(input_signal)
%  - d : desired signal(wanted signal)
%  - N : filter tap size
%  - mu : step size
%
% Output :
%  - w : weights of adaptive filter
%  - e : error signal
% ##############################################################################

  % set defualt parameters
  if nargin < 4,
    mu = 0.001;
  end
  
  L = length(x);
  w = zeros(1, N);
  e = zeros(1, L);

  % LMS algorithms
  for n = N : L,
    x_vect = x(n : -1 : n - N + 1);
    e(n) = d(n) - w * x_vect';
    w = w + mu * e(n) * x_vect;
  end
end

[w, e] = LMS(x, d, L);

figure, plot(e)
title('Error');
xlabel('Sample');
##saveas(gcf,'e1.png');
figure, stem(w)
title('Adaptive filter weights');