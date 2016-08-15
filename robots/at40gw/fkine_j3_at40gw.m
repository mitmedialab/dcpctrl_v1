function fk3 = fkine_j3_at40gw(Theta_1,Theta_2,Theta_3,D_4)
%FKINE_J3_AT40GW
%    FK3 = FKINE_J3_AT40GW(THETA_1,THETA_2)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    02-Jun-2016 13:54:36

t2 = Theta_1.*pi.*(1.0./1.8e2);
t3 = cos(t2);
t4 = Theta_2.*pi.*(1.0./1.8e2);
t5 = cos(t4);
t6 = sin(t2);
t7 = sin(t4);
t8 = pi.*(1.0./4.0);
t9 = -t4+t8;
t10 = cos(t9);
t11 = t3.*t7;
t12 = t5.*t6.*6.123233995736766e-17;
t13 = t11+t12;
t14 = sin(t9);
t15 = t3.*t5;
t16 = t15-t6.*t7.*6.123233995736766e-17;
t17 = t3.*t5.*6.123233995736766e-17;
t18 = t17-t6.*t7;
t19 = t3.*t7.*6.123233995736766e-17;
t20 = t5.*t6;
t21 = t19+t20;
fk3 = reshape([1.0,t3.*6.095e2-t3.*t5.*2.53365e3+t6.*t7.*1.551413181329846e-13,t6.*6.095e2-t3.*t7.*1.551413181329846e-13-t5.*t6.*2.53365e3,t7.*2.53365e3+7.113e2,0.0,-t10.*t16+t13.*t14,-t10.*t21-t14.*t18,t7.*t10+t5.*t14,0.0,t10.*t13+t14.*t16,-t10.*t18+t14.*t21,t5.*t10-t7.*t14,0.0,-t6,t3,6.123233995736766e-17],[4,4]);