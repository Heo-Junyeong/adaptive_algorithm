
%-------------------------------------------------------------------------------
% multi-chennel FxLMS algorithm test
%
% Developed by Junyeong Heo
%-------------------------------------------------------------------------------

clear

x = 0;

%% local functions
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

%%% const variable
pi2 = 2*pi;

%%% data preprocessing

noise_source = audioread('tone_500_1000_1500.wav');

secondary_path.u11 = audioread('s_ir.wav');
secondary_path.u12 = audioread('s_ir.wav');
secondary_path.u13 = audioread('s_ir.wav');

secondary_path.u21 = audioread('s_ir.wav');
secondary_path.u22 = audioread('s_ir.wav');
secondary_path.u23 = audioread('s_ir.wav');

secondary_path.u31 = audioread('s_ir.wav');
secondary_path.u32 = audioread('s_ir.wav');
secondary_path.u33 = audioread('s_ir.wav');

%%% parameters
L = 128;    % control filter order
N = 10;  % reference signal size

%%% reference signal generate
Fs = 4096;

n = [0 : 1/Fs : N - 1/Fs];
x = sin(pi2*500*n)+sin(pi2*1000*n)+sin(pi2*1500*n);

##h = [0.921 0.107 0.336 1 0.336 0.107 0.921];
d = filter(secondary_path,1,x);

[w, e] = normalized_LMS(d, noise_source, L);

figure, plot(e, 'k-');
title('Error');
xlabel('Sample');
##saveas(gcf,'e1.png');
figure, plot(w);%stem(w)
title('Adaptive filter weights');