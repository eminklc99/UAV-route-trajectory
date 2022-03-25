%%CIRCLE
addpath utilities
clear all;
close all;
clc;

global Quad;

init_plot;
plot_quad_model;
quad_variables;
quad_dynamics_nonlinear;   
arttirma=1;
errx = 0;
erry = 0;
errz = 0;
errtoplam= 0;

%% Run The Simulation Loop
while Quad.t_plot(Quad.counter-1)< max(Quad.t_plot);    
    
      sensor_meas;
    if(mod(Quad.counter, 115) == 0 && arttirma<24)
        Quad.X_des_GF = Quad.XMATRISCIRCLE(arttirma);
        Quad.Y_des_GF = Quad.YMATRISCIRCLE(arttirma);
        Quad.Z_des_GF = Quad.ZMATRISCIRCLE(arttirma);
        arttirma=arttirma+1;
    end

    position_PID;
    attitude_PID;
    rate_PID;
    quad_motor_speed;
    quad_dynamics_nonlinear;    

    if(mod(Quad.counter,115) == 0)
        errx = errx+(Quad.X_des_GF - Quad.X)^2;
        erry = erry+(Quad.Y_des_GF - Quad.Y)^2;
        errz = errz+(Quad.Z_des_GF - Quad.Z)^2;
        errtoplam = (errtoplam+sqrt(errx+erry+errz));
    end
    
    if(mod(Quad.counter,3)==0)
        plot_quad 
        test = text(0,0,num2str(Quad.counter));
        plot3(Quad.X,-Quad.Y,-1*Quad.Z,".","color","r","Markersize",4);
        plot3(Quad.eksen_CIRCLEX,Quad.eksen_CIRCLEY,-1*Quad.eksen_CIRCLEZ,"-o","color","b","Markersize",5);
        Quad.counter;
        drawnow
        delete(test)
      
    end
    Quad.init = 1;  
end

%% Plot Data
plot_data