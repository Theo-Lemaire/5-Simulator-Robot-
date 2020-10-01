function commande = controle(Etat,Cible,Capteurs)


    global wmax dmax;

    hys_col = 0.0;      % hysteresis de distance par rapport à la détection d'obstacle
    hys_abs_col = 0.0;  % hysteresis de distance par rapport à l'absence d'obstacle
    
    theta_C = 0;

    nombre_capteurs = length(Capteurs); % nombre de capteurs du robot

    vitesse_evitement = (3/10)*wmax;  % vitesse pour manoeuvre en cas d'obstacle (par defaut 3)
    vitesse_correction = (2/10)*wmax; % vitesse de rotation pour correction d'angle robot / cible (par defaut 2); vitesse_correction doit rester < à vitesse_evitement
    diff_vitesse_roue = min(3,(wmax/(max(vitesse_evitement,vitesse_correction))));  % différentiel de vitesse pour ne pas que le robot tourne sur place

    erreur_angle = 0.05; % erreur max de direction du robot vers la cible

    Capteurs_dist = [Capteurs(1,1)/dmax; Capteurs(2,1)/dmax; Capteurs(3,1)/dmax];   % afin d'être indépendant de dmax pour identifier l'état du robot
    
    %conversion de rob.theta mod 2*pi
    if (Etat(3) > 0)
        Etat(3) = mod(Etat(3), 2*pi);
    elseif (Etat(3) < 0)
        Etat(3) = -mod(-Etat(3), 2*pi);
    end
    
    %*****     RECHERCHE DU TYPE DE CAPTEUR EN COLLISON LE PLUS PROCHE(DROITE / CENTRE / GAUCHE)     *****    
    
    min_dist = max(Capteurs_dist);
    min_index_dist = 0;   
    
    for i = 1:1:nombre_capteurs
        if (Capteurs_dist(i) > 0 && Capteurs_dist(i) < (1-hys_col)) % distance à l'obstacle compris en ]0; dmax*(1-hys_col)[
            if (Capteurs_dist(i) <= min_dist)                       % recherche du minimum
            min_dist = Capteurs_dist(i);
            min_index_dist = i;                                     % 0 = pas de collision; 1 = capteur droit; 2 = capteur centre; 3 = capteur gauche
            end
        end
    end
    
    if (min_index_dist ~= 0)
        collision = 1;
    else
        collision = 0;
    end
     
   
    %*****     RECHERCHE DE L'ABSENCE DE COLLISION     *****
    
    abs_collision = 1-nombre_capteurs;
    for i = 1:1:nombre_capteurs
        if (abs(Capteurs_dist(i)) > 1+hys_abs_col)                  % si toutes les distances sont > dmax*(1+hys_abs_col)
            abs_collision = abs_collision +1;
        end
    end    

    %*****     COMMANDE EN FONCTION DES OBSTACLES     *****

    if (collision == 1) % collision imminente

            if (min_index_dist == 2) %central
                if (Capteurs_dist(min_index_dist+1) == Capteurs_dist(min_index_dist-1))     % cpt gauche = cpt droite, par défaut choix de tourner à gauche (wd > wg)
                    commande(1) = vitesse_evitement*diff_vitesse_roue;
                    commande(2) = vitesse_evitement;
                elseif (Capteurs_dist(min_index_dist+1) > Capteurs_dist(min_index_dist-1))  % cpt gauche > cpt droite --> tourne à gauche (wd > wg)
                    commande(1) = vitesse_evitement*diff_vitesse_roue;
                    commande(2) = vitesse_evitement;
                else                                                                        % cpt gauche < cpt droite --> tourne à droite (wd < wg)
                    commande(1) = vitesse_evitement;
                    commande(2) = vitesse_evitement*diff_vitesse_roue;
                end
            elseif (min_index_dist == 3)    % détection d'obstacle à gauche -> tourne à droite (wd < wg)
                commande(1) = vitesse_evitement;
                commande(2) = vitesse_evitement*diff_vitesse_roue;
            else                % détection d'obstacle à droite -> tourne à gauche (wd > wg)
                commande(1) = vitesse_evitement*diff_vitesse_roue;
                commande(2) = vitesse_evitement;
            end
            
    elseif (abs_collision == 1) % aucun obstacle détecté
        
    %*****     CALCUL ANGLE ROBOT / CIBLE     *****
    
             
        if (Cible(1)-Etat(1) == 0)            % Delta x = 0
            if (Cible(2)-Etat(2) == 0)
                theta_C = 0;                  % Delta x =0, Delta y =0    --> theta_C = 0
            elseif (Cible(2)-Etat(2) > 0)
                theta_C = pi/2;               % Delta x =0, Delta y >0    --> theta_C = pi/2
            else
                theta_C = -pi/2;              % Delta x =0, Delta y <0    --> theta_C = -pi/2
            end
            
        elseif (Cible(1)-Etat(1) > 0)                               % Delta x >0                --> theta_C = atan(dy/dx)  ( compris ]-pi/2;+pi/2[ )
            theta_C = atan((Cible(2)-Etat(2))/(Cible(1)-Etat(1)));
        
        elseif (Cible(2)-Etat(2) == 0)                              % Delta x <0, Delta y =0    --> theta_C = -pi
            theta_C = -pi;
        
        elseif (Cible(2)-Etat(2) > 0)                               % Delta x <0, Delta y >0    --> theta_C = pi + atan(dy/dx) ( compris ]pi/2;pi[ )
            theta_C = pi+atan((Cible(2)-Etat(2))/(Cible(1)-Etat(1)));
        
        else                                                        % Delta x <0, Delta y <0    --> theta_C = pi + atan(dy/dx) ( compris ]-pi;-pi/2[
            theta_C = -pi+atan((Cible(2)-Etat(2))/(Cible(1)-Etat(1)));
        end
        

        
        %*****     CALCUL DES COMMANDES EN FONCTION DE THETA_C     *****

        if ((Etat(3)-theta_C < -erreur_angle) || (Etat(3)-theta_C > erreur_angle))              % erreur d'angle de direction acceptable
            
            if (((Etat(3)-theta_C <= 0) && (Etat(3)-theta_C >= -pi)) || ((Etat(3)-theta_C >= pi) && (Etat(3)-theta_C <= 2*pi)))

                commande(1) = vitesse_correction*diff_vitesse_roue; % tourne à gauche
                commande(2) = vitesse_correction;
            elseif (((Etat(3)-theta_C > 0) && (Etat(3)-theta_C < pi)) || ((Etat(3)-theta_C <- pi) && (Etat(3)-theta_C > -2*pi)))

                commande(1) = vitesse_correction; % tourne à droite
                commande(2) = vitesse_correction*diff_vitesse_roue;
            else

                commande(1) = wmax;
                commande(2) = wmax;
            end
        else

            commande(1) = wmax;
            commande(2) = wmax;
        end   
    else
        %evitement = 1
        commande(1) = wmax;
        commande(2) = wmax; 
    end
    
    theta_Robot = Etat(3)
    theta_C
    Err_angle = Etat(3) - theta_C

end