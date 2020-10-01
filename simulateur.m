function [Etat,Capteurs] = simulateur(Etat, commande)

    global Nmin Cible R L Longueur wmax dmax dt angle_radar;

    %*****     INITIALISATION DE LA FENETRE     *****
    figure(1);  %creer 
    clf;        %supprimer 
    hold on;    %maintenir
    set(1,'position',[500 500 600 400]);    %position de la fenêtre
    axis([-15 15 -15 15]);                  %coordonnées des axes
    axis equal;                             %axes orthonormés

    %*****     DESSIN DES OBSTACLES    *****
    obstacle = [-8 4 -8 5 -2 -3 5 10 3 7; -8 5 1 5 1 2 15 15 8 -2]; %xB yB...; xA yA...
    %obstacle = [-15 -15 15 -15 15 15 -15 15 3 7; -15 15 -15 -15 15 -15 15 15 8 -2]; %xB yB...; xA yA...
    
    i = 1;
    while(i <= 2*length(obstacle))
        line([obstacle(i) obstacle(i+1)], [obstacle(i+2) obstacle(i+3)],'color', 'b', 'linewidth', 1);
        i = i+4;
    end

    %*****     DESSIN DE LA CIBLE    *****
    viscircles([Cible(1) Cible(2)], 2*Nmin, 'Color', 'r');
    
    %*****     CARACTERISTIQUES DU ROBOT - CONDITIONS INITIALES     *****   
    nb_capteurs = 3;  % nombre de radar du robot
    Capteurs = zeros(1, nb_capteurs); %capteur
    
    %*****     CALCUL DES VISTESSES LINEAIRES ET ANGULAIRE LIEES A LA COMMANDE     *****    
    u(1) = (R/2)*(commande(1)+commande(2)); %vitesse linéaire
    u(2) = (R/(2*2*L))*(commande(1)-commande(2)); %vitesse angulaire
    
    %*****     CALCUL DE LA NOUVELLE POSITION DU ROBOT ET     *****
    x(1) = Etat(1);
    x(2) = Etat(2);
    x(3) = Etat(3);
    t = 0;
    
    %*****     MISE A JOUR DE LA POSITION [ETAT]     *****
    
    [t45 x45] = ode45(@(t,x) Modele(t,x,u,Etat)',[0 dt],x);
    L45 = length(t45);
    Etat = x45(L45,:);

    %*****     AFFECTATION DE ETAT A ROB POUR LISIBILITE     *****      
    rob.X = Etat(1);
    rob.Y = Etat(2);
    rob.theta = Etat(3);
     
    %*****     INITIALISATION DES VARIABLES     *****
    % matrice des coordonnées des 3 points du corps du robot
    coord_rob = [rob.X+L*cos(pi/2-rob.theta),rob.Y-L*sin(pi/2-rob.theta);rob.X-L*cos(pi/2-rob.theta),rob.Y+L*sin(pi/2-rob.theta);rob.X+cos(rob.theta)*Longueur,rob.Y+Longueur*sin(rob.theta)];
    
    % matrices des intersections entre le robot et les obstacles
    ms = zeros(1, 3*length(obstacle)/2); %capteur
    mt = zeros(1, 3*length(obstacle)/2); %obstacle 
    
    %*****     DESSINS DU ROBOT ET DE SES 3 CAPTEURS     *****

    % affichage de la position initiale
    face_rob = [1 2 3];
    ptr = patch('vertices',coord_rob,'faces',face_rob);
    ptr2 = line([rob.X rob.X+cos(rob.theta)*dmax*Longueur], [rob.Y rob.Y+dmax*Longueur*sin(rob.theta)], 'color', 'g', 'linewidth', 1);
    ptr3 = line([rob.X rob.X+cos(rob.theta+angle_radar)*dmax*Longueur], [rob.Y rob.Y+dmax*Longueur*sin(rob.theta+angle_radar)], 'color', 'g', 'linewidth', 1);
    ptr4 = line([rob.X rob.X+cos(rob.theta-angle_radar)*dmax*Longueur], [rob.Y rob.Y+dmax*Longueur*sin(rob.theta-angle_radar)], 'color', 'g', 'linewidth', 1);

    drawnow; % actualisation de l'affichage à chaque tour de boucle
    
    %*****     CALCUL DES NOUVELLES POSITIONS DES CAPTEURS     *****
    
    %coord_rayons = [rob.X+dmax*Longueur*cos(rob.theta) rob.Y+dmax*Longueur*sin(rob.theta) rob.X+dmax*Longueur*cos(rob.theta+angle_radar) rob.Y+dmax*Longueur*sin(rob.theta+angle_radar) rob.X+dmax*Longueur*cos(rob.theta-angle_radar) rob.Y+dmax*Longueur*sin(rob.theta-angle_radar); rob.X rob.Y rob.X rob.Y rob.X rob.Y];
    % droite, centre, gauche
    coord_rayons = [rob.X+dmax*Longueur*cos(rob.theta-angle_radar) rob.Y+dmax*Longueur*sin(rob.theta-angle_radar) rob.X+dmax*Longueur*cos(rob.theta) rob.Y+dmax*Longueur*sin(rob.theta) rob.X+dmax*Longueur*cos(rob.theta+angle_radar) rob.Y+dmax*Longueur*sin(rob.theta+angle_radar); rob.X rob.Y rob.X rob.Y rob.X rob.Y];
    
    %*****     CALCUL DES MATRICES D'INTERSECTION     *****
        
    i = 1;
    j = 1;
    k = 1;
    while (i <= 2*length(obstacle)) % nombre d'obstacles
       while (j <= 6)
          M = [coord_rayons(1,j)-coord_rayons(2,j) -(obstacle(i)-obstacle(i+1)); coord_rayons(1,j+1)-coord_rayons(2,j+1) -(obstacle(i+2)-obstacle(i+3))];
          if (det(M) ~= 0) 
              inter = inv(M)*[obstacle(i+1)-coord_rayons(2,j); obstacle(i+3)-coord_rayons(2,j+1)];
              ms(k) = inter(1);
              mt(k) = inter(2);
          else % cas obstacle et capteur //
              ms(k) = dmax+1;
              mt(k) = dmax+1;
          end
          k = k+1;
          j = j+2;
       end       
       i = i+4;
       j = 1;
    end
    
    %*****     RECHERCHE DE L'OBSTACLE LE PLUS PROCHE     *****
    nb_obstacles_collision = 0;
    
    Capteurs_collision = zeros(1, nb_capteurs*length(obstacle)/2);
    for i = 1:1:nb_capteurs*length(obstacle)/2
        if (ms(i) > 0 && (mt(i) >= 0 && mt(i) <= 1))    % matrice des capteurs positifs en condition de collision (sinon =0)
            Capteurs_collision(i) = ms(i);
            nb_obstacles_collision = nb_obstacles_collision + 1;
        end
    end
    
    %*****     RECHERCHE DU CAPTEUR LE PLUS PROCHE     *****
    
    min_index_collision = 0;
    if (nb_obstacles_collision ~= 0) % au minimum un obstacle en collision
    
        [min_Capteurs_collision, min_index_collision] = max(Capteurs_collision); % initialisation au max pour recherche du min.

        for i = 1:1:nb_capteurs*length(obstacle)/2          % recherche du capteur le plus proche parmi les capteurs en collision 
            if (Capteurs_collision(i) > 0 && Capteurs_collision(i) <= min_Capteurs_collision)
                min_Capteurs_collision = Capteurs_collision(i);
                min_index_collision = i;
            end
        end

        %*****     IDENTIFICATION DU CAPTEUR LE PLUYS PROCHE (CENTRE / GAUCHE / DROITE)     *****

        %type_capteur = mod(min_index_collision, nb_capteurs); % 1 = central / 2 = gauche / 0 = droite
         type_capteur = mod(min_index_collision, nb_capteurs); % 1 = droite / 2 = centre / 0 = gauche

    
    else % aucun obstacle en collision
        type_capteur = -1;
    end
    
    %*****     AFFECTATION DES DISTANCES CAPTEURS A L'OBSTACLE LE PLUS PROCHE     *****
    
    if (type_capteur == -1) % aucun capteur en collision
        Capteurs(1,1) = dmax+1;     %droite
        Capteurs(2,1) = dmax+1;     %central
        Capteurs(3,1) = dmax+1;     %gauche
 
    elseif (type_capteur == 1) %capteur droite
        Capteurs(1,1) = ms(min_index_collision)*dmax; %droite
        Capteurs(2,1) = ms(min_index_collision+1)*dmax; %central
        Capteurs(3,1) = ms(min_index_collision+2)*dmax;   %gauche

    elseif (type_capteur == 2) %capteur central
        Capteurs(1,1) = ms(min_index_collision-1)*dmax; %droite
        Capteurs(2,1) = ms(min_index_collision)*dmax;   %central
        Capteurs(3,1) = ms(min_index_collision+1)*dmax; %gauche
        
    else %capteur gauche
        Capteurs(1,1) = ms(min_index_collision-2)*dmax;   %droite
        Capteurs(2,1) = ms(min_index_collision-1)*dmax; %central
        Capteurs(3,1) = ms(min_index_collision)*dmax; %gauche
    end
    
    %*****     AFFECTATION DES ANGLES DES CAPTEUR LE PLUYS PROCHE A [CAPTEURS]     *****

    Capteurs(2,2) = 0;
    Capteurs(1,2) = -angle_radar;
    Capteurs(3,2) = angle_radar;
    
   
end