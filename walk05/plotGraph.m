M = dlmread('plotoutput.txt');


x = M(:,1);
figure
plot(x,M(:,2:end));


title('Walking Variables')
xlabel('time')
ylabel('value')

legend('HEAD_X','HEAD_Z','HIP_X','HIP_Z','L_KNEE_X','L_KNEE_Z','R_KNEE_X',...
'R_KNEE_Z','L_FOOT_X','L_FOOT_Z','R_FOOT_X','R_FOOT_Z','TORSO_ABS_ANGLE',...
'L_THIGH_ABS_ANGLE','R_THIGH_ABS_ANGLE','L_CALF_ABS_ANGLE','R_CALF_ABS_ANGLE');


