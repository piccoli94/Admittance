close all
clear all
% addpath("00_unstable/");
% addpath("01_unstable_ks1_kp70/");
% addpath("02_stable_ks1_kp10/");
% addpath("03_stable_same_gain02_away_ref/");
% addpath("04 k_pinv0_01/");
% addpath("05_fs200_kp5_ks05/");
% addpath("06_kp150_ks01/");
% addpath("07_kp200_ks_01/");
%  addpath("09_t70_kp300_ks02_alpha05/")
filename1="Delta_x.txt";
filename2="tau.txt";
filename3="tau_a.txt";
filename4="tau_b.txt";
filename5="tau_c.txt";
filename6="q.txt";
filename7="dq.txt";
filename8="q_r_dot_dot.txt";
filename9="q_r_dot.txt";
Delta_x=importdata(filename1,' ');
tau=importdata(filename2,' ');
tau_a=importdata(filename3,' ');
tau_b=importdata(filename4,' ');
tau_c=importdata(filename5,' ');
q=importdata(filename6,' ');
dq=importdata(filename7,' ');
ddqr=importdata(filename8,' ');
dqr=importdata(filename9,' ');
%% ------------------------------------------------
t1=1:1:length(Delta_x);
hold on
for k=1:3
    plot(t1,Delta_x(:,k))
end
hold off
xlabel("Samples");
ylabel("Position Error[m]");
legend("E_X","E_Y","E_Z")
%% --------------------------------------------------
figure(2)
t2=1:1:length(tau);
hold on
for k=1:7
    plot(t1,tau(:,k))
end
hold off
xlabel("Samples");
ylabel("Control torque[mN]");
legend("Tau1","Tau2","Tau3","Tau4","Tau5","Tau6","Tau7")

figure(3)
t3=1:1:length(tau_a);
hold on
for k=1:7
    plot(t1,tau_a(:,k))
end
hold off
xlabel("Samples");
ylabel("Control torque[mN]");
legend("Tau1","Tau2","Tau3","Tau4","Tau5","Tau6","Tau7")

figure(4)
t4=1:1:length(tau_b);
hold on
for k=1:7
    plot(t1,tau_b(:,k))
end
hold off
xlabel("Samples");
ylabel("Control torque[mN]");
legend("Tau1","Tau2","Tau3","Tau4","Tau5","Tau6","Tau7")


figure(5)
t5=1:1:length(tau_c);
hold on
for k=1:7
    plot(t1,tau_c(:,k))
end
hold off
xlabel("Samples");
ylabel("Control torque[mN]");
legend("Tau1","Tau2","Tau3","Tau4","Tau5","Tau6","Tau7")

figure(6)
t5=1:1:length(tau_c);
hold on
for k=1:7
    plot(t1,q(:,k))
end
hold off
xlabel("Samples");
ylabel("Control torque[mN]");
legend("q1","q2","q3","q4","q5","q6","q7")

figure(7)
t5=1:1:length(tau_c);
hold on
for k=1:7
    plot(t1,dq(:,k))
end
hold off
xlabel("Samples");
ylabel("Control torque[mN]");
legend("dq1","dq2","dq3","dq4","dq5","dq6","dq7")

figure(8)
t5=1:1:length(tau_c);
hold on
for k=1:7
    plot(t1,dqr(:,k))
end
hold off
xlabel("Samples");
ylabel("Control torque[mN]");
legend("dqr1","dqr2","dqr3","dqr4","dqr5","dqr6","dqr7")

figure(9)
t5=1:1:length(tau_c);
hold on
for k=1:7
    plot(t1,ddqr(:,k))
end
hold off
xlabel("Samples");
ylabel("Control torque[mN]");
legend("ddqr1","ddqr2","ddqr3","ddqr4","ddqr5","ddqr6","ddqr7")