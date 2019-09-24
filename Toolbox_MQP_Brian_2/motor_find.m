% Aerospace MQP VTOL UAV Optimizer motor_find
function motor_info=motor_find(motor_input);
%m_info=[Current (A),Power(W),RPM,Efficency (G/W),Weight (Kg), Thrust(Kg)]
%per single motor. %Ideally load in spreadsheet and index into it. 
m_info=readtable('C:\Users\eve\Documents\WPI\MQP\Toolbox_MQP_Brian_2\Motor_Comparison.csv','ReadVariableNames',false);
%{ 
 BACK UP HARD CODED MOTOR DATA
m_info=
%}
motor_options=['MN2212KV780V2.0','EMAXMT3510','admiral-gp5-4220-770kv','LHILHM029EMAXCF2822KV1200','DYSD3536910Kv1999kv','EMAXMT2213-935KV','EMAXMT4008-600KV','EMAXMT3110-700KV','EMAXMT3510-600KV','ScorpionM3011-760KV','ScorpionM4010-400KV'];
index=find(strcmp(motor_options,motor_input)~=0,1);
motor_info=m_info{index,:};
end