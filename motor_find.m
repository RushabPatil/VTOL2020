% Aerospace MQP VTOL UAV Optimizer motor_find
function motor_info=motor_find(motor_input);
%m_info=[Current (A),Power(W),RPM,Efficency (G/W),Weight (Kg), Thrust(Kg)]
%per single motor. %Ideally load in spreadsheet and index into it. 
m_info=[0];
index=strfind(m_info(:,1),motor_input);
motor_info=m_info(index,:);
end