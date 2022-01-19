% Copyright (c) 2010, Agustinus Oey
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:
% 
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in
%       the documentation and/or other materials provided with the distribution
%     * Neither the name of the  nor the names
%       of its contributors may be used to endorse or promote products derived
%       from this software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.

%--------------------------------------------------------------------------
% One of my friends asked me about the FxLMS algorithm. So, in return, 
% I provided him a little example of a single channel feed-forward active 
% noise control system based on the FxLMS. You can find many good 
% information in "Active Noise Control Systems - Algorithms and DSP 
% Implementations," written by S. M. Kuo and D. R. Morgan in 1996.
%
% Here is the sketch of the system. 
%
%              +-----------+                       +   
% x(k) ---+--->|   P(z)    |--yp(k)----------------> sum --+---> e(k)
%         |    +-----------+                          ^-   |
%         |                                           |    |
%         |        \                                ys(k)  |     
%         |    +-----------+          +-----------+   |    |
%         +--->|   C(z)    |--yw(k)-->|   S(z)    |---+    |
%         |    +-----------+          +-----------+        |
%         |            \                                   |
%         |             \----------------\                 |
%         |                               \                |
%         |    +-----------+          +-----------+        |
%         +--->|   Sh(z)   |--xs(k)-->|    LMS    |<-------+
%              +-----------+          +-----------+        
% 
% I used FIR filter to model P(z), C(z), S(z), and Sh(z).
% 
% Imagine that the noise x(k) is propagating from the source to the sensor,
% through the fluid medium P(z). The sensor measures the arriving noise as 
% yp(k). 
%
% To reduce noise, we generate another 'noise' yw(k) using the controller 
% C(z). We hope that it destructively interferes x(k). It means that the 
% controller has to be a model of the propagation medium P(z). Least mean 
% square algorithm is applied to adjust the controller coefficient/weight.
%
% However, there is also fluid medium S(z) that stay between the actuator 
% and sensor. We called it the secondary propagation path. So, to make the 
% solusion right, we need to compensate the adjustment process using Sh(z), 
% which is an estimate of S(z).
% 
% Let's start the code :)
%
% Developed by Heo Junyeong Heo
%--------------------------------------------------------------------------

% Set simulation duration (normalized) 
clear
T=10000; 

% We do not know P(z) and S(z) in reality. So we have to make dummy paths
Pw=[0.01 0.25 0.5 1 0.5 0.25 0.01];
Sw=Pw*0.25;

% Remember that the first task is to estimate S(z). So, we can generate a
% white noise signal,
x_iden=randn(1,T);

% send it to the actuator, and measure it at the sensor position, 
y_iden=filter(Sw, 1, x_iden);

% Then, start the identification process
Shx=zeros(1,16);     % the state of Sh(z)
Shw=zeros(1,16);     % the weight of Sh(z)
e_iden=zeros(1,T);   % data buffer for the identification error

% and apply least mean square algorithm
mu=0.1;                         % learning rate
for k=1:T,                      % discrete time k
    Shx=[x_iden(k) Shx(1:15)];  % update the state
    Shy=sum(Shx.*Shw);	        % calculate output of Sh(z)
    e_iden(k)=y_iden(k)-Shy;    % calculate error         
    Shw=Shw+mu*e_iden(k)*Shx;   % adjust the weight
end

% Lets check the result
figure
subplot(2,1,1);
plot([1:T], e_iden);
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Identification error');
subplot(2,1,2)
stem(Sw) 
hold on 
stem(Shw, 'r*')
ylabel('Amplitude');
xlabel('Numbering of filter tap');
legend('Coefficients of S(z)', 'Coefficients of Sh(z)')


% The second task is the active control itself. Again, we need to simulate 
% the actual condition. In practice, it should be an iterative process of
% 'measure', 'control', and 'adjust'; sample by sample. Now, let's generate 
% the noise: 
X=randn(1,T);

% and measure the arriving noise at the sensor position,
Yd=filter(Pw, 1, X);
  
% Initiate the system,
Cx=zeros(1,16);       % the state of C(z)
Cw=zeros(1,16);       % the weight of C(z)
Sx=zeros(size(Sw));   % the dummy state for the secondary path
e_cont=zeros(1,T);    % data buffer for the control error
Xhx=zeros(1,16);      % the state of the filtered x(k)

% and apply the algorithm
mu=0.1;                            % learning rate
for k=1:T,                         % discrete time k
    Cx=[X(k) Cx(1:15)];            % update the controller state    
    Cy=sum(Cx.*Cw);                % calculate the controller output	
    Sx=[Cy Sx(1:length(Sx)-1)];    % propagate to secondary path
    e_cont(k)=Yd(k)-sum(Sx.*Sw);   % measure the residue
    Shx=[X(k) Shx(1:15)];          % update the state of Sh(z)
    Xhx=[sum(Shx.*Shw) Xhx(1:15)]; % calculate the filtered x(k)
    Cw=Cw+mu*e_cont(k)*Xhx;        % adjust the controller weight
end

% Report the result
figure
subplot(2,1,1)
plot([1:T], e_cont)
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Noise residue')
subplot(2,1,2)
plot([1:T], Yd);
hold on 
plot([1:T], Yd-e_cont, 'r:')
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Noise signal', 'Control signal')