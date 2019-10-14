function [tau,rho] = Std_Atm_Model(altitude,metric)
% Uses the Standard Atmospheric Model to calculate temperature and density
% as a function of altitude above sea level

% INPUTS
% altitude  - height above sea level (ft OR m)
% metric    - 0: no (imperial units); 1: yes (metric units)

% OUTPUTS
% tau       - temperature (K)
% rho       - density (slug/ft^3 OR kg/m^3)

% VARIABLES
% a1        - rate of change of temperature with altitude (K/ft OR K/m)
% R         - ideal gas constant (lbf*ft/slug*K OR J/kg*K)
% temp_sl   - temperature at sea level (K)
% dens_sl   - density at sea level (slugs/ft^3 OR kg/m^3)
% g         - acceleration due to gravity (ft/s^2 OR m/s^2)

% temp_11   - temperature at 11 km altitude (K)
% dens_11   - density at 11 km altitude (slugs/ft^3 OR kg/m^3)

temp_sl = 288.16;
if metric == 1  % metric units
    dens_sl = 1.225;
    a1 = -6.5e-3;
    g = 9.8;
    R = 287;
    if altitude <= 11000
        tau = temp_sl + a1*altitude;
        rho = dens_sl * (tau/temp_sl)^(-1 -g/(a1*R));
    else
        temp_11 = temp_sl + a1*11000;
        dens_11 = dens_sl * (temp_11/temp_sl)^(-1 -g/(a1*R));
        tau = temp_11;
        rho = dens_11*exp(-g*(altitude - 11000)/(R*temp_11));
        if altitude > 25000
            disp('error: altitude too high')
        end
    end
else            % imperial units
    dens_sl = 0.0023769;
    R = 3089.2;
    a1 = -1.9812e-3;
    g = 32.15;
    m_to_ft = 3.2808;
    cutoff = 11000*m_to_ft; % transition (11km) in feet
    limit = 25000*m_to_ft;
    if altitude <= cutoff
        tau = temp_sl + a1*altitude;
        rho = dens_sl * (tau/temp_sl)^(-1 -g/(a1*R));
    else
        temp_11 = temp_sl + a1*cutoff;
        dens_11 = dens_sl * (temp_11/temp_sl)^(-1 -g/(a1*R));
        tau = temp_11;
        rho = dens_11*exp(-g*(altitude - cutoff)/(R*temp_11));
        if altitude > limit
            disp('error: altitude too high')
        end
    end 
end
end