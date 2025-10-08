DX0 = [0; 0; -Va0*sin(gamma_trim); 0; 0; 0; 0; 0; Va0/R_trim; 0; 0; 0];
IDX = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];
X0 = [0; 0; -h0; Va0; 0; 0; 0; gamma_trim; 0; 0; 0; 0];
IX0 = [];
U0 = [0; 0; 0; 1];
IU0 = [];
IY0 = [1,3];
Y0 = [Va0; gamma_trim; 0];
[x_trim,u_trim,y_trim,dx_trim] = trim('uav_trim_sim',X0,U0,Y0,IX0,IU0,IY0,DX0,IDX);

clearvars DX0 IDX X0 IX0 U0 IU0 Y0 IY0 dx_trim