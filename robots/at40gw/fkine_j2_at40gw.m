function fk2 = fkine_j2_at40gw(Theta_1,Theta_2,Theta_3,D_4)
%FKINE_J2_AT40GW
%    FK2 = FKINE_J2_AT40GW(THETA_1,THETA_2)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    02-Jun-2016 13:54:20

t2 = Theta_1.*pi.*(1.0./1.8e2);
t3 = cos(t2);
t4 = Theta_2.*pi.*(1.0./1.8e2);
t5 = sin(t4);
t6 = cos(t4);
t7 = sin(t2);
fk2 = reshape([1.0,t3.*6.095e2,t7.*6.095e2,7.113e2,0.0,-t3.*t6+t5.*t7.*6.123233995736766e-17,t3.*t5.*(-6.123233995736766e-17)-t6.*t7,t5,0.0,t3.*t5+t6.*t7.*6.123233995736766e-17,t3.*t6.*(-6.123233995736766e-17)+t5.*t7,t6,0.0,-t7,t3,6.123233995736766e-17],[4,4]);
