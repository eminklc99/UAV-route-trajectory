%%Rectangular

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
    if(mod(Quad.counter, 100) == 0 && arttirma<32)
        Quad.X_des_GF = Quad.XMATRISREC(arttirma);
        Quad.Y_des_GF = Quad.YMATRISREC(arttirma);
        Quad.Z_des_GF = Quad.ZMATRISREC(arttirma);
        arttirma=arttirma+1;
    end


    position_PID;
    attitude_PID;
    rate_PID;
    quad_motor_speed;
    quad_dynamics_nonlinear;    


    if(mod(Quad.counter, 100) == 0)
        errx = errx+(Quad.X_des_GF - Quad.X)^2;
        erry = erry+(Quad.Y_des_GF - Quad.Y)^2;
        errz = errz+(Quad.Z_des_GF - Quad.Z)^2;
        errtoplam = (errtoplam+sqrt(errx+erry+errz));
    end
    
    if(mod(Quad.counter,3)==0)
        
        %h = animatedline(Quad.X,Quad.Y,"Marker","^","color","r","Markersize",1);
        plot_quad 
        test = text(0,0,num2str(Quad.counter));
        %addpoints(h,Quad.X,Quad.Y)
        %plot3(Quad.X,-Quad.Y,-Quad.Z,"^","color","r","Markersize",1.2);
        plot3(Quad.X,-Quad.Y,-1*Quad.Z,".","color","r","Markersize",6);
        plot3(Quad.eksenrecX,Quad.eksenrecY,-1*Quad.eksenrecZ,"-o","color","b","Markersize",5);
       % plot3(Quad.X_des_GF,Quad.Y_des_GF,-1*Quad.Z_des_GF,"-o","color","b","Markersize",5);
        %test3 = text(0,0.5,num2str(errtoplam));
        %legend("x="+num2str(Quad.X), "y="+num2str(Quad.Y), "z="+num2str(Quad.Z))
%         campos([A.X+2 A.Y+2 A.Z+2])
%         camtarget([A.X A.Y A.Z])
%         camroll(0);
        Quad.counter;
        drawnow
        delete(test)
      %  delete(test3)
      
    end
    Quad.init = 1;  %Ends initialization after first simulation iteration
end


%% Plot Data
plot_data