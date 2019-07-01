function result = simulate(controller, start_time, end_time, dt)

    %SIMULATION TIMES

    %This creates an array of times from the start time to the end time in
    %increments of dt. Therefore times(5) will, in the case that the start
    %time=0, end time=0 and dt=0.005, yield the value 0.02 as the first value
    %in the array is 0. The default times are set below in case the user
    %does not input them. The 'nargin' function counts the number of
    %arguments inputted by the user.

    if nargin<4
        start_time = 0;
        end_time = 30;
        dt = 0.005;
    end

    times = start_time:dt:end_time;
    
       
    g = 9.81;
    m = 0.5;
    L = 0.25;
    k = 3e-6;
    b = 1e-7;
    I = diag([5e-3, 5e-3, 10e-3]);
    kd = 0.25;
    
   
    N = numel(times);

    %x= [0;0;10];

    x=[0 0 10].';


    xdot=zeros(3,1);
    theta=zeros(3,1);

    %Euler angles between 90 deg/s and -90deg/s

    if nargin==0
        thetadot=zeros(3,1);
    else
        thetadot=deg2rad(randi([-90,90],3,1));
    end

    
    xout=zeros(3,N);
    xdotout=zeros(3,N);
    thetaout=zeros(3,N);
    thetadotout=zeros(3,N);
    inputout=zeros(4,N);
    
    %PARAMETERS FOR THE CONTROLLER
    controller_params = struct('dt',dt,'I',I,'k',k,'L',L,'b',b,'m',m,'g',g);  

    %STEP THROUGH SIMULATION, UPDATING STATE OF QUADCOPTER

    %We do this using a for-loop telling MatLab to run through the calculations
    %for every time. 
    
    %First create a counter that keeps track of which iteration the program
    %is in.
    
    count=0;

    for t = times
        count=count+1;
        
        controller_params.xdot=xdot;
        
        %Here we need to take an input from the controller, or if no
        %controller is specified then we can use some default input. The
        %inputs are values of gamma (angular velocity^2) for each propeller
             
        if nargin==0
            in=zeros(4,1);
            in(1)=700;
            in(2)=750;
            in(3)=700;
            in(4)=750;
            in=in.^2;
        else
            [in,controller_params] = controller(controller_params,thetadot);
        end

        %COMPUTE FORCES, TORQUES AND ACCELERATIONS
        
        %Define angular velocity, omega
        omega = thetadot2omega(thetadot,theta);
        %Define acceleration, a
        a=acceleration(in, theta, xdot, m, g, k, kd);
        %Define anglular acceleration, omegadot
        omegadot=angular_acceleration(in,omega,I,L,b,k);
        
        %ADVANCE SYSTEM STATE
        
        %This is coded in the reverse order of what would seem intuitive so
        %that the code has updated versions of each variable that it needs.
        
        omega = omega + omegadot*dt;
        thetadot = omega2thetadot(omega,theta);
        theta = theta + thetadot*dt;
        xdot = xdot + a*dt;
        x = x + xdot*dt;
        
        %The for-loop will cycle through this N times for every time
        %increment. We now need to store these values so that they can be
        %plotted:
        
        %STORE SIMULATION STATE FOR OUTPUT
        xout(:,count)=x;
        xdotout(:,count)=xdot;
        thetaout(:,count)=theta;
        thetadotout(:,count)=thetadot;
        inputout(:,count)=in;

        
        
        
        
        
        
    end
        
    
    %PUT OUTPUT INFORMATION INTO A STRUCTURED ARRAY:
    result = struct('x',xout,'theta',thetaout,'vel',xdotout,'angvel',thetadotout,'t',times, 'dt',dt,'input',inputout);  
    
    grapher(result)
    
    
    

    
    

         
       
        
 end