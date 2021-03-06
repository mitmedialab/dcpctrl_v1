function fk4 = fkine_j4_at40gw(Theta_1,Theta_2,Theta_3,D_4)
%FKINE_J4_AT40GW
%    FK4 = FKINE_J4_AT40GW(THETA_1,THETA_2,THETA_3)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    02-Jun-2016 14:01:05

t2 = Theta_1.*pi.*(1.0./1.8e2);
t3 = cos(t2);
t4 = sin(t2);
t5 = Theta_2.*pi.*(1.0./1.8e2);
t6 = cos(t5);
t7 = sin(t5);
t8 = pi.*(1.0./4.0);
t9 = -t5+t8;
t10 = cos(t9);
t11 = t3.*t6;
t19 = t4.*t7.*6.123233995736766e-17;
t12 = t11-t19;
t13 = sin(t9);
t14 = t3.*t7;
t15 = t4.*t6.*6.123233995736766e-17;
t16 = t14+t15;
t17 = Theta_3.*pi.*(1.0./1.8e2);
t18 = t8+t17;
t20 = cos(t18);
t21 = t10.*t16;
t22 = t12.*t13;
t23 = t21+t22;
t24 = sin(t18);
t25 = t10.*t12;
t26 = t25-t13.*t16;
t27 = t3.*t7.*6.123233995736766e-17;
t28 = t4.*t6;
t29 = t27+t28;
t30 = t3.*t6.*6.123233995736766e-17;
t32 = t4.*t7;
t31 = t30-t32;
t33 = t10.*t31;
t34 = t33-t13.*t29;
t35 = t10.*t29;
t36 = t13.*t31;
t37 = t35+t36;
t38 = t6.*t10;
t39 = t38-t7.*t13;
t40 = t6.*t13;
t41 = t7.*t10;
t42 = t40+t41;
fk4 = reshape([1.0,t3.*6.095e2+t4.*3.2941e2-t3.*t6.*2.53365e3+t4.*t7.*1.551413181329846e-13-t10.*t12.*1.7194e2+t13.*t16.*1.7194e2,t3.*(-3.2941e2)+t4.*6.095e2-t3.*t7.*1.551413181329846e-13-t4.*t6.*2.53365e3-t10.*t29.*1.7194e2-t13.*t31.*1.7194e2,t7.*2.53365e3+t7.*t10.*1.7194e2+t6.*t13.*1.7194e2+7.113e2,0.0,-t20.*t26+t23.*t24,-t20.*t37-t24.*t34,t20.*t42+t24.*t39,0.0,t20.*t23+t24.*t26,-t20.*t34+t24.*t37,t20.*t39-t24.*t42,0.0,-t4,t3,6.123233995736766e-17],[4,4]);
