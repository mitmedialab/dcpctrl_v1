function robot = config_at40gw
%{
AT40GW_MachineDefn_V3
Julian Leland
2016-03-24

This function generates a struct defining the AT40GW's mechanical
parameters. It sorts the data structures more intelligently so that
numerical indexing of the structure can be used. It also includes values
needed for FK and IK calculations (voltage/angle mappings, etc.)

Final data structure is organized as follows:
MachineData
- Name
- Joint(1...4)
-- Name
-- Kp
-- Kd
-- moveThresh
-- pctMaxV
-- PosLim
--- Max
--- Min
-- PWMLim
--- Max
--- Min
--- DBUpper
--- DBLower
-- RegNames
-- AIN measurement registers (unused registers have no value)
-- Rad Enc measurement registers
-- PWM registers
- RefDH
- Output

Examples:
 - To read the J1 PWM deadband upper limit:
 MachineData.Joint(1).PWMLimit.DBUpper
 - To read the J4 movement threshold: MachineData.Joint(4).moveThresh
 - To refer to an element using a for loop (variable i) to iterate through axes: 
for i = 1:4
    MachineData.Joint(i).PWMLimit.DBLower
end

INPUTS: None

OUTPUTS: None

DEPENDENCIES: None

TO-DO:

CHANGES:
- 2016-03-25: Added DH parameters. Added line at end to delete extraneous values.
- 2016-05-16: Added additional DIO/DAC channels, per Damien's
modifications; added PWMZero value
- 2016-05-25: Added digital input channels (to enable KUKA to send
"program finished" notifications to truck.
- 2016-05-26: Added analog input channels (to enable data recording from
external sensors
%}

%% Control Parameters
% Proportional gains: Best practice is to set in following ranges:
% - J1: 50 to 5000
% - J2 thru J4: 45 to 175
% -- Note: J4 gain needs to be inverted - J4 has its PWM mapping backwards
% Kp = struct('J1',0.3,'J2',0,'J3',0.3,'J4',0.5);
% Kd = struct('J1',0.4,'J2',0,'J3',0.1,'J4',0.4);
% Ki = struct('J1',0,'J2',0,'J3',0,'J4',0);
% Kv = struct('J1',0.3,'J2',0,'J3',0.8,'J4',0.8);
Kp = struct('J1',0.3,'J2',0,'J3',0.6,'J4',0.5);
Kd = struct('J1',0,'J2',0,'J3',0.3,'J4',0);
Ki = struct('J1',0,'J2',0,'J3',0,'J4',0);
Kv = struct('J1',0,'J2',0,'J3',0.5,'J4',0);

%% DH Parameters
% Create symbolic variables for joint angles/positions
syms Theta_1 Theta_2 Theta_3 D_4;

Base_DH = struct('a',0,'alph',0,'d',0,'the',0);
J1_DH = struct('a',609.50,'alph',-(pi/2),'d',711.30,'the', 0 + (Theta_1*(pi/180)));
J2_DH = struct('a',2533.65,'alph',0,'d',0,'the',pi + (Theta_2*(pi/180)));
InnerLink_DH = struct('a',171.94,'alph',0,'d',0,'the',(pi/4) + -(Theta_2*(pi/180)));
J3_DH = struct('a',59.02,'alph',-(pi/2),'d',-329.41,'the',(pi/4) + (Theta_3*(pi/180)));
J4_DH = struct('a',0,'alph',0,'d',4633.10 + D_4,'the',0);

% Some of our axes are inverted: negative PWM duty cycles correspond to
% positive sensor changes. GainInv accounts for this by multiplying the
% gains set above by 1 (normal) or -1 (inverted)
GainInv = struct('J1',1,'J2',1,'J3',-1,'J4',-1);

% moveThresh is different for J1 than other axes - it is defined in counts
% instead of volts.
moveThresh = struct('J1',500,'J2',0.04,'J3',0.04,'J4',0.04);
pctMaxV = struct('J1',1,'J2',1,'J3',1,'J4',1);

%% Machine geometric, controller & valve parameters
J1PosLim = struct('Max',1000000,'Min',-1000000);
% NOTE: J2PosLim has an opposite sense from the other joints. This is
% because of the way that the J2 sensor is oriented relative to the
% positive angle direction (as defined by the DH parameters), which is
% different from the other axes. Don't change this!
J2PosLim = struct('Max',-9.508,'Min',8.075);
J3PosLim = struct('Max',9.912,'Min',-9.337);
J4PosLim = struct('Max',9.488,'Min',-9.925);

% Values for LabJack T7 interface.
% Diesel
%J1PWMLim = struct('Max',155500,'Min',5000,'DBUpper',87500,'DBLower',75500);
%J2PWMLim = struct('Max',155500,'Min',5000,'DBUpper',87500,'DBLower',75500);
%J3PWMLim = struct('Max',155500,'Min',5000,'DBUpper',87500,'DBLower',74500);
%J4PWMLim = struct('Max',155500,'Min',5000,'DBUpper',87500,'DBLower',73000);

% Electric
% J1PWMLim = struct('Max',155500,'Min',5000,'DBUpper',89000,'DBLower',71000);
% J2PWMLim = struct('Max',155500,'Min',5000,'DBUpper',87500,'DBLower',75500);
% J3PWMLim = struct('Max',155500,'Min',5000,'DBUpper',89000,'DBLower',73500);
% % Anything less than 43000 all gives full speed
% J4PWMLim = struct('Max',155500,'Min',43000,'DBUpper',89000,'DBLower',71500);

% Electric - Guaranteed to move
J1PWMLim = struct('Max',155500,'Min',5000,'DBUpper',89000,'DBLower',69000);
%J1PWMLim = struct('Max',155500,'Min',5000,'DBUpper',90000,'DBLower',62000);
%J1PWMLim = struct('Max',155500,'Min',5000,'DBUpper',95000,'DBLower',63000);
J2PWMLim = struct('Max',155500,'Min',5000,'DBUpper',93000,'DBLower',70000);
J3PWMLim = struct('Max',155500,'Min',5000,'DBUpper',91000,'DBLower',72000);
% Anything less than 43000 all gives full speed
J4PWMLim = struct('Max',155500,'Min',43000,'DBUpper',91000,'DBLower',69000);


%% Generate coefficients for mappings between joint sensor velocity and joint PWM DC
syms x

J1VelMap = struct('A_pos',-3.033e-09,'B_pos',-1.003e-05,'C_pos',3.908,'D_pos',8.75e+04,'A_neg',2.052e-09,'B_neg',0.0001714,'C_neg',5.287,'D_neg',7.543e+04,'PosKnee',17200,'NegKnee',-17200);
f_J1Pos = @(x) J1VelMap.A_pos*x^3 + J1VelMap.B_pos*x^2 + J1VelMap.C_pos*x + J1VelMap.D_pos;
g_J1Pos = @(x) J1PWMLim.DBUpper;
J1VelMap.PosVelLim = fzero(@(x)f_J1Pos(x)-g_J1Pos(x),0); % These limits are measured in counts/sec - they are limits on the MINIMUM joint velocity that can be input.
f_J1Neg = @(x) J1VelMap.A_neg*x^3 + J1VelMap.B_neg*x^2 + J1VelMap.C_neg*x + J1VelMap.D_neg;
g_J1Neg = @(x) J1PWMLim.DBLower;
J1VelMap.NegVelLim = fzero(@(x)f_J1Neg(x)-g_J1Neg(x),0);

J2VelMap = struct('A_pos','','B_pos','','C_pos','','D_pos','','A_neg','','B_neg','','C_neg','','D_neg','','PosKnee',1000,'NegKnee',-1000); % Dummy values
J2VelMap.NegVelLim = -1000; % More dummy values
J2VelMap.PosVelLim = 1000; 

% Velocity map coefficients for J3 - also calculates threshold velocities
% from PWM deadband limits.
J3VelMap = struct('A_pos',-1.03e+05,'B_pos',0.2169,'C_pos',1.413e+05,'D_pos','','A_neg',-1.286e+04,'B_neg',-5.004e+04,'C_neg',8.475e+04,'D_neg','','PosKnee',1.39,'NegKnee',-1.39);
f_J3Pos = J3VelMap.A_pos * x^J3VelMap.B_pos + J3VelMap.C_pos;
g_J3Pos = finverse(f_J3Pos);
J3VelMap.PosVelLim = eval(subs(g_J3Pos,J3PWMLim.DBLower));
f_J3Neg = J3VelMap.A_neg*x^2 + J3VelMap.B_neg*x + J3VelMap.C_neg;
g_J3Neg = finverse(f_J3Neg);
J3VelMap.NegVelLim = eval(subs(g_J3Neg,J3PWMLim.DBUpper));

J4VelMap = struct('A_pos',1.79e+04,'B_pos',-5.085e+04,'C_pos',7.450e+04,'D_pos','','A_neg',-5.794e+04,'B_neg',8.75e+04,'C_neg','','D_neg','','PosKnee',.91,'NegKnee',-.77);
J4VelMap.NegVelLim = 0; % These are dummy values 
J4VelMap.PosVelLim = 0; 

%% 
J1Names = struct('AIN_NegCh','','AIN_Range','','AIN_ResIndx','','AIN_Read','','REnc_EnA','DIO6_EF_ENABLE','REnc_EnB','DIO7_EF_ENABLE','REnc_IndxA','DIO6_EF_INDEX','REnc_IndxB','DIO7_EF_INDEX','REnc_ReadAF','DIO6_EF_READ_A_F','PWM_En','DIO5_EF_ENABLE','PWM_Indx','DIO5_EF_INDEX','PWM_Opts','DIO5_EF_OPTIONS','PWM_CfgA','DIO5_EF_CONFIG_A');
J2Names = struct('AIN_NegCh','AIN2_NEGATIVE_CH','AIN_Range','AIN2_RANGE','AIN_ResIndx','AIN2_RESOLUTION_INDEX','AIN_Read','AIN2','REnc_EnA','','REnc_EnB','','REnc_IndxA','','REnc_IndxB','','REnc_ReadAF','','PWM_En','DIO2_EF_ENABLE','PWM_Indx','DIO2_EF_INDEX','PWM_Opts','DIO2_EF_OPTIONS','PWM_CfgA','DIO2_EF_CONFIG_A');
J3Names = struct('AIN_NegCh','AIN3_NEGATIVE_CH','AIN_Range','AIN3_RANGE','AIN_ResIndx','AIN3_RESOLUTION_INDEX','AIN_Read','AIN3','REnc_EnA','','REnc_EnB','','REnc_IndxA','','REnc_IndxB','','REnc_ReadAF','','PWM_En','DIO3_EF_ENABLE','PWM_Indx','DIO3_EF_INDEX','PWM_Opts','DIO3_EF_OPTIONS','PWM_CfgA','DIO3_EF_CONFIG_A');
J4Names = struct('AIN_NegCh','AIN4_NEGATIVE_CH','AIN_Range','AIN4_RANGE','AIN_ResIndx','AIN4_RESOLUTION_INDEX','AIN_Read','AIN4','REnc_EnA','','REnc_EnB','','REnc_IndxA','','REnc_IndxB','','REnc_ReadAF','','PWM_En','DIO4_EF_ENABLE','PWM_Indx','DIO4_EF_INDEX','PWM_Opts','DIO4_EF_OPTIONS','PWM_CfgA','DIO4_EF_CONFIG_A');

J1VtoAngVals = struct('ZeroVoltage','','SensLen','','a_len','','b_len','','c_len','','CountsRev',737280);
J2VtoAngVals = struct('ZeroVoltage',6.79,'SensLen',400,'a_len',283.978,'b_len',754.053,'c_len',666.334,'CountsRev',''); % This is a revolute joint, with piston in a linkage.
J3VtoAngVals = struct('ZeroVoltage',-3.48,'SensLen',600,'a_len',1163.725,'b_len',387.085,'c_len',1082.457,'CountsRev',''); % This is a revolute joint, with piston in a linkage.
J4VtoAngVals = struct('ZeroVoltage',-9.925,'SensLen',3350,'a_len','','b_len','','c_len',0,'CountsRev',''); % This is a prismatic joint, with piston NOT in a linkage - direct drive.

% Configure digital inputs.
In1Names = struct('DAC_Name','','DIO_Name','DIO0'); % FIO0
In2Names = struct('DAC_Name','','DIO_Name','DIO1'); % FIO1

% Configure digital outputs. Requires extended DIOs be available
% https://labjack.com/support/datasheets/t7/digital-io/extended-features
Out1Names = struct('DAC_Name','','DIO_Name','DIO8'); % EIO0
Out2Names = struct('DAC_Name','','DIO_Name','DIO9'); % EIO1
Out3Names = struct('DAC_Name','','DIO_Name','DIO10'); % EIO2
Out4Names = struct('DAC_Name','','DIO_Name','DIO11'); % EIO3
Out5Names = struct('DAC_Name','','DIO_Name','DIO12'); % EIO4 - KUKA IN 23
Out6Names = struct('DAC_Name','','DIO_Name','DIO13'); % EIO5
Out7Names = struct('DAC_Name','','DIO_Name','DIO14'); % EIO6
Out8Names = struct('DAC_Name','','DIO_Name','DIO15'); % EIO7 - KUKA IN 22
Out9Names = struct('DAC_Name','','DIO_Name','DIO16'); % CIO0 - KUKA IN 20
Out10Names = struct('DAC_Name','','DIO_Name','DIO17'); % CIO1 - KUKA IN 21

% Configure analog inputs
AIn1Names = struct('AIN_NegCh','AIN10_NEGATIVE_CH','AIN_Range','AIN10_RANGE','AIN_ResIndx','AIN10_RESOLUTION_INDEX','AIN_Read','AIN10'); 
AIn2Names = struct('AIN_NegCh','AIN11_NEGATIVE_CH','AIN_Range','AIN11_RANGE','AIN_ResIndx','AIN11_RESOLUTION_INDEX','AIN_Read','AIN11');

% Configure analog outputs
AOut1Names = struct('DAC_Name','DAC0','DIO_Name',''); 
AOut2Names = struct('DAC_Name','DAC1','DIO_Name','');

% Create final structure
robot = struct('Name', 'AT40GW_V3');
robot.PWMZero = 80000;
robot.Joint(1) = struct('Name','Joint 1','DH',J1_DH,'EncType','Rotary','Kp',Kp.J1,'Kd',Kd.J1,'Ki',Ki.J1,'Kv',Kv.J1,'GainInv',GainInv.J1,'moveThresh',moveThresh.J1,'pctMaxV',pctMaxV.J1,'PosLim',J1PosLim,'PWMLim',J1PWMLim,'VelMap',J1VelMap,'RegNames',J1Names,'PWM_zero',80000,'PosSensParams',J1VtoAngVals);
robot.Joint(2) = struct('Name','Joint 2','DH',J2_DH,'EncType','Linear','Kp',Kp.J2,'Kd',Kd.J2,'Ki',Ki.J2,'Kv',Kv.J2,'GainInv',GainInv.J2,'moveThresh',moveThresh.J2,'pctMaxV',pctMaxV.J2,'PosLim',J2PosLim,'PWMLim',J2PWMLim,'VelMap',J2VelMap,'RegNames',J2Names,'PWM_zero',80000,'PosSensParams',J2VtoAngVals);
robot.Joint(3) = struct('Name','Joint 3','DH',J3_DH,'EncType','Linear','Kp',Kp.J3,'Kd',Kd.J3,'Ki',Ki.J3,'Kv',Kv.J3,'GainInv',GainInv.J3,'moveThresh',moveThresh.J3,'pctMaxV',pctMaxV.J3,'PosLim',J3PosLim,'PWMLim',J3PWMLim,'VelMap',J3VelMap,'RegNames',J3Names,'PWM_zero',80000,'PosSensParams',J3VtoAngVals);
robot.Joint(4) = struct('Name','Joint 4','DH',J4_DH,'EncType','Linear','Kp',Kp.J4,'Kd',Kd.J4,'Ki',Ki.J4,'Kv',Kv.J4,'GainInv',GainInv.J4,'moveThresh',moveThresh.J4,'pctMaxV',pctMaxV.J4,'PosLim',J4PosLim,'PWMLim',J4PWMLim,'VelMap',J4VelMap,'RegNames',J4Names,'PWM_zero',80000,'PosSensParams',J4VtoAngVals);
robot.DH(1) = struct('Name','Base_RefDH','Params',Base_DH);
robot.DH(2) = struct('Name','J1_DH','Params',J1_DH);
robot.DH(3) = struct('Name','J2_DH','Params',J2_DH);
robot.DH(4) = struct('Name','InnerLink_RefDH','Params',InnerLink_DH);
robot.DH(5) = struct('Name','J3_DH','Params',J3_DH);
robot.DH(6) = struct('Name','J4_DH','Params',J4_DH);

% Include digital outputs and analog outputs
robot.DigIn(1) = struct('Name','Digital Input 1 ','Type','Digital','RegNames',In1Names);
robot.DigIn(2) = struct('Name','Digital Input 2','Type','Digital','RegNames',In2Names);
robot.DigOut(1) = struct('Name','Digital Output 1','Type','Digital','RegNames',Out1Names,'DefaultVal',0);
robot.DigOut(2) = struct('Name','Digital Output 2','Type','Digital','RegNames',Out2Names,'DefaultVal',0);
robot.DigOut(3) = struct('Name','Digital Output 3','Type','Digital','RegNames',Out3Names,'DefaultVal',0);
robot.DigOut(4) = struct('Name','Digital Output 4','Type','Digital','RegNames',Out4Names,'DefaultVal',0);
robot.DigOut(5) = struct('Name','Digital Output 5','Type','Digital','RegNames',Out5Names,'DefaultVal',0);
robot.DigOut(6) = struct('Name','Digital Output 6','Type','Digital','RegNames',Out6Names,'DefaultVal',0);
robot.DigOut(7) = struct('Name','Digital Output 7','Type','Digital','RegNames',Out7Names,'DefaultVal',0);
robot.DigOut(8) = struct('Name','Digital Output 8','Type','Digital','RegNames',Out8Names,'DefaultVal',0);
robot.DigOut(9) = struct('Name','Digital Output 9','Type','Digital','RegNames',Out9Names,'DefaultVal',0);
robot.DigOut(10) = struct('Name','Digital Output 10','Type','Digital','RegNames',Out10Names,'DefaultVal',0);
robot.AnOut(1) = struct('Name','Analog Output 1','Type','Digital','RegNames',AOut1Names,'DefaultVal',2.5);
robot.AnOut(2) = struct('Name','Analog Output 2','Type','Digital','RegNames',AOut2Names,'DefaultVal',2.5);

% Include analog inputs
robot.AnIn(1) = struct('Name','Analog Input 1','Type','Analog','RegNames',AIn1Names);
robot.AnIn(2) = struct('Name','Analog Input 2','Type','Analog','RegNames',AIn2Names);



end