%05/03/2020
function final_final2

%*****     NETTOYAGE DE LA FENETRE     *****

clc;
close all;
clear all;

%*****     INITIALISATION DE LA FENETRE     *****

figure(1); %creer 
clf; %supprimer 
hold on; %maintenir
set(1,'position',[110 220 600 400]); %position de la fen�tre
axis([-15 15 -15 15]); %taille des axes
axis equal; %axes orthonorm�s

%*****     DESSIN DES OBSTACLES     *****

i = 1;
%obstacle = [-8 4 -8 5 -2 -3 5 10 3 7; -8 5 1 5 1 2 15 15 8 -2]; %xB yB...; xA yA...
obstacle = [-15 -15 15 -15 15 15 -15 15 3 7; -15 15 -15 -15 15 -15 15 15 8 -2]; %xB yB...; xA yA...

while(i <= 2*length(obstacle))
    line([obstacle(i) obstacle(i+1)], [obstacle(i+2) obstacle(i+3)],'color', 'b', 'linewidth', 1);
    i = i+4;
end

%*****     INTERACTION UTILISATEUR - POSITIONNER L'ARRIVEE PUIS LE DEPART     *****

A = ginput(1) %clic pour l'arriv�e;
%A = [10, 5];
viscircles(A, 0.5, 'Color', 'b');
P = ginput(1) %clic pour poser le robot
%P = [-5, -5];

%*****     CARACTERISTIQUES DU ROBOT - CONDITIONS INITIALES     *****

rob.R = 0.05; %rayon des roues
rob.L = 0.28; %entraxe roues
rob.X = P(1); %coordonn�e x du centre de l'arri�re du robot
rob.Y = P(2); %coordonn�e y du centre de l'arri�re du robot 
rob.theta = 0; %angle du robot
rob.H = 0.8; %longueur
rob.wD = 10; %vitesse angulaire de la roue droite
rob.wG = 10; %vitesse angulaire de la roue gauche
rob.radarY = rob.Y+0.8; %coordonn�e y du capteur
u = [0 0]; %[vitesse_lin�aire vitesse_angulaire] du robot

erreur_angle = 0.2;

%*****     INITIALISATION DES VARIABLES     *****

%matrice des coordonn�es des 3 points du robot
coord_rob = [rob.X+(rob.L/2)*cos(pi/2-rob.theta),rob.Y-(rob.L/2)*sin(pi/2-rob.theta);rob.X-(rob.L/2)*cos(pi/2-rob.theta),rob.Y+(rob.L/2)*sin(pi/2-rob.theta);rob.X+cos(rob.theta)*rob.H,rob.Y+rob.H*sin(rob.theta)];
hys_col = 0.2; %intervalle de l'hyst�r�sis
hys_abs_col_ms = 0.0; %intervalle de l'hyst�r�sis
hys_abs_col_mt = 0.2; %intervalle de l'hyst�r�sis
%matrices des distances entre le robot et les obstacles
ms = zeros(1, 3*length(obstacle)/2); %capteur 
mt = zeros(1, 3*length(obstacle)/2); %obstacle
nb_capteurs = 3;
obstacle_precedent = -1;
vitesse_roue = 2;

%*****     DESSINS DU ROBOT ET DES 3 RADARS     *****

%affichage de la position initiale
face_rob = [1 2 3];
rob.ptr = patch('vertices',coord_rob,'faces',face_rob);
rob.ptr2 = line([rob.X rob.X+cos(rob.theta)*3*rob.H], [rob.Y rob.Y+3*rob.H*sin(rob.theta)], 'color', 'g', 'linewidth', 1);
rob.ptr3 = line([rob.X rob.X+cos(rob.theta+pi/6)*3*rob.H], [rob.Y rob.Y+3*rob.H*sin(rob.theta+pi/6)], 'color', 'g', 'linewidth', 1);
rob.ptr4 = line([rob.X rob.X+cos(rob.theta-pi/6)*3*rob.H], [rob.Y rob.Y+3*rob.H*sin(rob.theta-pi/6)], 'color', 'g', 'linewidth', 1);

%*****     PARAMETRES DE SIMULATION     ***** 

Tmax =300; %temps de simu
dt = 0.1; %incr�ment de temps

%*****     BOUCLE DE SIMULATION     *****

%m�morisation de la position initiale
x(1) = rob.X;
x(2) = rob.Y;
x(3) = rob.theta;

for t=0:dt:Tmax
    
    %*****     MISE A JOUR DES COORDONNEES     *****
    rob.X = x(1);
    rob.Y = x(2);
    rob.theta = x(3);
    
    %conversion de rob.theta mod 2*pi
    if (rob.theta > 0)
        rob.theta = mod(rob.theta, 2*pi);
    elseif (rob.theta < 0)
        rob.theta = -mod(-rob.theta, 2*pi);
    end
    
%     rob.wD = 10;
%     rob.wG = 10;
    
    %*****     AFFICHAGES DES POSITIONS     *****
 
    set(rob.ptr,'xdata',[rob.X+(rob.L/2)*cos(pi/2-rob.theta) rob.X-(rob.L/2)*cos(pi/2-rob.theta) rob.X+cos(rob.theta)*rob.H],'ydata',[rob.Y-(rob.L/2)*sin(pi/2-rob.theta) rob.Y+(rob.L/2)*sin(pi/2-rob.theta) rob.Y+rob.H*sin(rob.theta)]);
    set(rob.ptr2, 'xdata', [rob.X rob.X+3*rob.H*cos(rob.theta)], 'ydata', [rob.Y rob.Y+3*rob.H*sin(rob.theta)]);
    set(rob.ptr3, 'xdata', [rob.X rob.X+3*rob.H*cos(rob.theta+pi/6)], 'ydata', [rob.Y rob.Y+3*rob.H*sin(rob.theta+pi/6)]);
    set(rob.ptr4, 'xdata', [rob.X rob.X+3*rob.H*cos(rob.theta-pi/6)], 'ydata', [rob.Y rob.Y+3*rob.H*sin(rob.theta-pi/6)]);
    drawnow; %actualiser l'affichage � chaque tour de boucle
    
    %*****     CHANGEMENT DE REPERE     *****
    
    %T = [cos(rob.theta) -sin(rob.theta) rob.X;sin(rob.theta) cos(rob.theta) rob.Y; 0 0 1];
    %MR = inv(T)*[rob.X; rob.Y; 1]
    
    %*****     CALCUL DES NOUVELLES POSITIONS DES RADARS     *****
    
    coord_rayons = [rob.X+3*rob.H*cos(rob.theta) rob.Y+3*rob.H*sin(rob.theta) rob.X+3*rob.H*cos(rob.theta+pi/6) rob.Y+3*rob.H*sin(rob.theta+pi/6) rob.X+3*rob.H*cos(rob.theta-pi/6) rob.Y+3*rob.H*sin(rob.theta-pi/6); rob.X rob.Y rob.X rob.Y rob.X rob.Y];
    coord_rayons
    %*****     CALCUL DES MATRICES D'INTERSECTION     *****
    
    i = 1;
    j = 1;
    k = 1;
    while (i <= 2*length(obstacle)) %obstacles
       while (j <= 12)
          M = [coord_rayons(j)-coord_rayons(j+1) -(obstacle(i)-obstacle(i+1)); coord_rayons(j+2)-coord_rayons(j+3) -(obstacle(i+2)-obstacle(i+3))];
          if (det(M) ~= 0) 
              inter = inv(M)*[obstacle(i+1)-coord_rayons(j+1); obstacle(i+3)-coord_rayons(j+3)];
              ms(k) = inter(1);
              mt(k) = inter(2);
          else %cas obstacle et capteur //
              ms(k) = 10;
              mt(k) = 10;
          end
          k = k+1;
          j = j+4;
       end       
       i = i+4;
       j = 1;
    end    

    %*****     RECHERCHE DE L'OBSTACLE LE PLUS PROCHE     *****
    
    min_ms = 1;
    min_index_ms = 0;
    
    for i = 1:1:3*length(obstacle)/2
        if ((ms(i) >= 0 && ms(i) <= 1-hys_col) && (0 <= mt(i) && mt(i) <= 1)) %condition de collision
            if (ms(i) < min_ms)
                min_ms = ms(i);
                min_index_ms = i;
            end
        end
    end
    %min_index_ms = 0 --> pas de collision
    
    %*****     RECHERCHE DE L'ABSENCE DE COLLISION     *****

    absence_col = 3*length(obstacle)/2;
    for i = 1:1:3*length(obstacle)/2
        if ((ms(i) < 0-hys_abs_col_ms || ms(i) > 1+hys_abs_col_ms) || (mt(i) > 1+hys_abs_col_mt || mt(i) < 0-hys_abs_col_mt))
            absence_col = absence_col - 1;
        end
    end  
    
    %*****     REACTIONS SELON LES COLLISIONS     *****
    
    % V�rification robot en zone d'arriv�e
    if((rob.X <= A(1)+0.25 && rob.X >= A(1)-0.25) && (rob.Y <= A(2)+0.25 && rob.Y >= A(2)-0.25)) %robot en zone d'arriv�e
        rob.wD = 0;
        rob.wG = 0;
    elseif (min_index_ms ~= 0) %d�tection d'obstacle tout capteur           
        type_capteur = mod(min_index_ms, nb_capteurs); %1 = central / 2 = gauche / 0 = droite
        if (type_capteur == 1)
            if (ms(min_index_ms+1) == ms(min_index_ms+2)) %gauche = droite / obstacle_prec = 1 -> gauche -1 -> droite
                rob.wD = vitesse_roue*obstacle_precedent;
                rob.wG = -vitesse_roue*obstacle_precedent;
            elseif (ms(min_index_ms+1) > ms(min_index_ms+2)) %gauche > droite --> tourne � gauche
                rob.wD = vitesse_roue;
                rob.wG = -vitesse_roue;
                obstacle_precedent = 1;
            else  %droite > gauche --> tourne � droite
                rob.wD = -vitesse_roue;
                rob.wG = vitesse_roue;
                obstacle_precedent = -1;
            end
        elseif (type_capteur == 2) %d�tection d'obstacle gauche
            rob.wD = -vitesse_roue;
            rob.wG = vitesse_roue;
            obstacle_precedent = -1;
        else %d�tection d'obstacle droite
            rob.wD = vitesse_roue;
            rob.wG = -vitesse_roue;
            obstacle_precedent = 1;
        end
    elseif ((absence_col == 0)) % aucun obstacle d�tect� et pas le bon angle
        %calcul de l'angle pour rejoindre la cible cod� de 0 � 2pi
        if (A(1)-rob.X == 0)
            if (A(2)-rob.Y <= 0)
                theta_A = -pi/2;
            else
                theta_A = pi/2;
            end
        elseif (A(1)-rob.X > 0)
            theta_A = atan((A(2)-rob.Y)/(A(1)-rob.X)); % angle du robot
            %    angle_arrivee =pi-atan(-(A(2)-P(2))/(A(1)-P(1)));
        elseif (A(2)-rob.Y >= 0)
            theta_A = pi+atan((A(2)-rob.Y)/(A(1)-rob.X)); % angle du robot
            %     angle_arrivee = atan((A(2)-P(2))/(A(1)-P(1)));
        else
            theta_A = -pi+atan((A(2)-rob.Y)/(A(1)-rob.X)); % angle du robot
        end

        %rotation pour avoir le bon angle
        if ((rob.theta-theta_A < -erreur_angle) || (rob.theta-theta_A > erreur_angle))
            if (((rob.theta-theta_A <= 0) && (rob.theta-theta_A >= -pi)) || (rob.theta-theta_A >= pi))
                rob.wD = vitesse_roue;
                rob.wG = -vitesse_roue;
            elseif (((rob.theta-theta_A > 0) && (rob.theta-theta_A < pi)) || (rob.theta-theta_A < -pi))
                rob.wD = -vitesse_roue;
                rob.wG = vitesse_roue;
            else
                rob.wD = 10;
                rob.wG = 10;                
            end
        else
            rob.wD = 10;
            rob.wG = 10;            
        end
    else
        rob.wD = 10;
        rob.wG = 10;
    end
            
    %*****     CALCUL DE LA NOUVELLE COMMANDE     *****
    
    u(1) = (rob.R/2)*(rob.wD+rob.wG); %vitesse lin�aire
    u(2) = (rob.R/(2*rob.L))*(rob.wD-rob.wG); %vitesse angulaire
    
    %*****     SIMULATION     *****
    
    [t45 x45] = ode45(@(t,x) Modele(t,x,u,rob)',[0 dt],x);
    L45 = length(t45);
    x = x45(L45,:);
    
end
