% Aerospace MQP VTOL UAV Optimizer air_foil_find
%The following two matericies are hard coded using data from xflr5
%Edits to the cord or speed of aircraft result in these values needing to be recalcuated.
function air_foil_data=air_foil_find(air_foil_input,V_input)
%Cl=[10,15,20,25]
air_foil_Cl=[.4085,.4164,.422,.4261;... %NACA4412
             .7955,.8169,.8333,.8459;...%S1223;
             .8733,.8961,.914,.9279;... %E423
             .6397,.6572,.6719,.6839;...%MH114
             .3504,.3577,.3629,.3667];  %SD7026
%Cm=[10,15,20,25]
air_foil_Cm=[-.0911,-.0925,-.0937,-.0944;...%NACA4412
             -.1972,-.201,-.2039,-.2059;... %S1223
             -.2013,-.205,-.208,-.2103;...  %E423
             -.1607,-.1639,-.1667,-.169;... %MH114
             -.0756,-.0769,-.0779,-.0786];  %SD7026
air_foil_max_Cl=[1.2604,1.2915,1.3147,1.3328;...    %NACA4412
             1.4993,1.5363,1.5638,1.5854;...        %S1223;
             1.5187,1.56,1.59,1.6144;...            %E423
             1.3705,1.4,1.4318,1.453;...            %MH114
             1.2195,1.2515,1.2758,1.2943];          %SD7026

V_options = {'10', '15', '20', '25'};
Foil_options = {'NACA4412', 'S1223', 'E423', 'MH114', 'SD7026'};

V_choice = find(strcmp(V_options,V_input)~=0,1);
Foil_choice = find(strcmp(Foil_options,air_foil_input)~=0,1);

air_foil_data = [air_foil_Cl(Foil_choice,V_choice), air_foil_Cm(Foil_choice,V_choice), air_foil_max_Cl(Foil_choice,V_choice)];     
end