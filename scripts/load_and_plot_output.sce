// Commands to install SciLab in Linux:
// sudo apt update
// sudo apt upgrade
// sudo apt install scilab
//
// Command to run Scilab in Linux:
// scilab

joint1 = fscanfMat("output_joint1.csv")
joint2 = fscanfMat("output_joint2.csv")
joint3 = fscanfMat("output_joint3.csv")

t = joint1(1:size(joint1)(1),1)

pos1 = joint1(1:size(joint1)(1),2)
vel1 = joint1(1:size(joint1)(1),3)
acc1 = joint1(1:size(joint1)(1),4)

pos2 = joint2(1:size(joint2)(1),2)
vel2 = joint2(1:size(joint2)(1),3)
acc2 = joint2(1:size(joint2)(1),4)

pos3 = joint3(1:size(joint3)(1),2)
vel3 = joint3(1:size(joint3)(1),3)
acc3 = joint3(1:size(joint3)(1),4)

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
