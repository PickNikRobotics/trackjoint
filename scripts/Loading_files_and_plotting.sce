M1 = fscanfMat("output_joint1.csv")
M2 = fscanfMat("output_joint2.csv")
M3 = fscanfMat("output_joint3.csv")

t = M1(1:size(M1)(1),1)

pos1 = M1(1:size(M1)(1),2)
vel1 = M1(1:size(M1)(1),3)
acc1 = M1(1:size(M1)(1),4)

pos2 = M2(1:size(M2)(1),2)
vel2 = M2(1:size(M2)(1),3)
acc2 = M2(1:size(M2)(1),4)

pos3 = M3(1:size(M3)(1),2)
vel3 = M3(1:size(M3)(1),3)
acc3 = M3(1:size(M3)(1),4)

figure();
subplot(3,1,1)
plot(t,pos1)
xtitle( "Joint 1" , "" , "Pos. [m]" );
subplot(3,1,2)
plot(t,vel1)
xtitle( "" , "" , "Vel. [m/s]" );
subplot(3,1,3)
plot(t,acc1)
xtitle( "" , "t [s]" , "Acc. [m/s^2]" );

figure();
subplot(3,1,1)
plot(t,pos2)
xtitle( "Joint 2" , "" , "Pos. [m]" );
subplot(3,1,2)
plot(t,vel2)
xtitle( "" , "" , "Vel. [m/s]" );
subplot(3,1,3)
plot(t,acc2)
xtitle( "" , "t [s]" , "Acc. [m/s^2]" );

figure();
subplot(3,1,1)
plot(t,pos3)
xtitle( "Joint 3" , "" , "Pos. [m]" );
subplot(3,1,2)
plot(t,vel3)
xtitle( "" , "" , "Vel. [m/s]" );
subplot(3,1,3)
plot(t,acc3)
xtitle( "" , "t [s]" , "Acc. [m/s^2]" );
