% Aerospace MQP VTOL UAV Optimizer battery_find
function battery_data=battery_find(battery_input)
%b_Info=[Discharge (C), Current(A),Max Power(W),Mass (Kg)]
b_info=[35,77,854.7,.181;... %ZIPPY Compact 35C
        35,77,854.7,.199;... %Turnigy 35C
        40,88,976.8,.196;... %ZIPPY Compact 40C
        40,88,976.8,.204;... %Turnigy 40C
        45,99,1098.9,.201;... %Turnigy Nano-Tech 45C
        45,99,1098.9,.211]%Turnigy Graphene 45C

 if isequal(battery_input,'ZIPPYCompact35C') == 1
             battery_data=b_info(1,:);
    elseif isequal(battery_input,'Turnigy35C') == 1
             battery_data=b_info(2,:);
    elseif isequal(battery_input,'ZIPPYCompact40C') == 1
             battery_data=b_info(3,:);
    elseif isequal(battery_input,'Turnigy40C') == 1
             battery_data=b_info(4,:)     
    elseif isequal(battery_input,'TurnigyNano-Tech45C') == 1
             battery_data=b_info(5,:);
    elseif isequal(battery_input,'TurnigyGraphene45C') == 1
             battery_data=b_info(6,:);
  end
end