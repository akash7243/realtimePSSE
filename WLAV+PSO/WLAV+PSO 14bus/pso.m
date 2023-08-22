clear all 
clc 
for times = 1:1
    %PSO initialisation 
    Vmax = 1;
    Xmax = 1.0;
    Xmin = -1.0; 
    Num_P =50;
    W = 0.729;
    c1= 1.49445;
    c2= c1; 
    dim = 27;
    %position and velocity initialisation
    particle_x = Xmin + rand(dim,Num_P); 
    best_particle_x = particle_x; 
    Vx = Vmax*(rand(dim, Num_P) - 0.5)*2;
    % evaluation of the global best position
    for i = 1:Num_P
        fit_fun(i)=state_WLAV14B(particle_x(:,i));
    end
    Pbest = fit_fun; 
    temp = min(Pbest);
    Pgb = min(find(Pbest == temp));
    for iter=1:2500 
        tic; 
        for i = 1:Num_P
            fit_fun(i) =state_WLAV14B(1*particle_x(:,i));
            if Pbest(i) > fit_fun(i) 
                Pbest(i) = fit_fun(i); 
                best_particle_x(:,i) = particle_x(:,i); 
            end
        end
        temp = min(Pbest);
        Pgb = min(find(Pbest == temp));
        local_temp = []; 
        % Swarm movement 
        for i = 1:dim 
            local_temp = [local_temp; best_particle_x(i,Pgb) - particle_x(i,:)];
        end
        Vx = W.*Vx + c1.*rand(dim,Num_P).*(best_particle_x - particle_x) +... 
            c2.*rand(dim,Num_P).* local_temp;
        particle_x = particle_x + Vx;
        fitness(iter) = Pbest(Pgb);
        t(iter)=toc 
    end
    % output plot
    times 
    fitness(iter) = Pbest(Pgb); 
    best_particle = best_particle_x(:,Pgb)
    particle_x = particle_x + Vx; 
    %plot(1:length(particle_x), particle_x); 
    figure(1)
    %hold on 
    plot(1:length(fitness), fitness, 'b')
    %title('PSO convergence for Optimal STATE for IEEE 14-bus system') 
    xlabel('Number of iteration') 
    ylabel('Total Objective function') 
end