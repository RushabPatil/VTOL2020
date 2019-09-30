% Aerospace MQP VTOL UAV Optimizer battery_find
function battery_data=battery_find(battery_input)
%b_Info=[Discharge (C), Current(A),Max Power(W),Mass (Kg)]
b_info=[35,77,854.7,.181;... %ZIPPY Compact 35C
        35,77,854.7,.199;... %Turnigy 35C
        40,88,976.8,.196;... %ZIPPY Compact 40C
        40,88,976.8,.204;... %Turnigy 40C
        45,99,1098.9,.201;...%Turnigy Nano-Tech 45C
        45,99,1098.9,.211;...   %Turnigy Graphene 45C
        150,487.5,7215,.327];   %MaxAmps 150C (2018 MQP; 4S 3250 MaH)
battery_options = {'ZIPPYCompact35C','Turnigy35C','ZIPPYCompact40C','Turnigy40C','TurnigyNano-Tech45C','TurnigyGraphene45C','MaxAmps150C'};

battery_choice = find(strcmp(battery_options,battery_input)~=0,1);

battery_data = b_info(battery_choice,:);     
if isequal(battery_input,'MaxAmps150C')
    fprintf('Note: this battery does not follow the 2019 WPI Competition requirements')
end
end