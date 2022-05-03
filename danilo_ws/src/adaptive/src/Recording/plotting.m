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
Delta_x=importdata(filename1,' ');
tau=importdata(filename2,' ');
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
