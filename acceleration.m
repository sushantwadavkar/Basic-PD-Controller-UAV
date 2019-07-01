function a = acceleration(inputs,angles,velocities,m,g,k,kd)


%m*a=[0 0 -mg].' + T_I +F_D =[0 0 -mg].' + R*T_B + F_D therefore a = [0 0 -g].' + 1/m*(R*[0 0 k*sum(angular accelerations^2)] + kd*v)



    gravity = [0; 0; -g];
    R = rotation(angles);
    thrust_FB=[0;0;k*sum(inputs)];
    thrust_FI=R*thrust_FB;
    
    Fd = -kd*velocities;
    
    a = gravity +1/m*thrust_FI +1/m*Fd;
    

end