%%CIRCLE

addpath utilities

%% Initialize Workspace
clear all;
close all;
clc;

global Quad;

%% Initialize the plot
init_plot;
plot_quad_model;

%% Initialize Variables

quad_variables;
quad_dynamics_nonlinear;   
arttirma=1;
errx = 0;
erry = 0;
errz = 0;
errtoplam= 0;

%% Run The Simulation Loop
while Quad.t_plot(Quad.counter-1)< max(Quad.t_plot);    
    
    % Measure Parameters (for simulating sensor errors)
      sensor_meas;

    if(mod(Quad.counter, 60) == 0 && arttirma<40)
        Quad.X_des_GF = Quad.XLEMNISCATE(arttirma);
        Quad.Y_des_GF = Quad.YLEMNISCATE(arttirma);
        Quad.Z_des_GF = Quad.ZLEMNISCATE(arttirma);
        arttirma=arttirma+1;
    end

    % Implement Controller
    position_PID;
    attitude_PID;
    rate_PID;
    
    % Calculate Desired Motor Speeds
    quad_motor_speed;
    
    % Update Position With The Equations of Motion
    quad_dynamics_nonlinear;    
    % Plot the Quadrotor's Position
    if(mod(Quad.counter, 60) == 0)
        errx = errx+(Quad.X_des_GF - Quad.X)^2;
        erry = erry+(Quad.Y_des_GF - Quad.Y)^2;
        errz = errz+(Quad.Z_des_GF - Quad.Z)^2;
        errtoplam = (errtoplam+sqrt(errx+erry+errz));
        
    end
    if(mod(Quad.counter,3)==0)
        plot_quad 
        test = text(0,0,num2str(Quad.counter));
        plot3(Quad.X,-Quad.Y,-Quad.Z,".","color","r","Markersize",4);
       % test3 = text(0,0.5,num2str(errtoplam));
        plot3(Quad.eksen_XLEMNISCATE,Quad.eksen_YLEMNISCATE,-1*Quad.eksen_ZLEMNISCATE,"-o","color","b","Markersize",5);

        Quad.counter;
        drawnow
        delete(test)
        %delete(test3)
    end


        
    Quad.init = 1;  %Ends initialization after first simulation iteration
end

%% Plot Data
plot_data
